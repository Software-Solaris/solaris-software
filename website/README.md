# Solaris web

The public site at [softwaresolaris.com](https://softwaresolaris.com), built with
[MkDocs](https://www.mkdocs.org/) + [Material for MkDocs](https://squidfunk.github.io/mkdocs-material/),
with the Doxygen API reference nested under `/doxygen/`. Both are served by a single
**nginx** container on `raspi`, reached from the internet through a Cloudflare Tunnel —
no public port is ever opened on the router for it.

---

## Request flow

```
  Browser
  https://softwaresolaris.com
        │  HTTPS
        ▼
  Cloudflare Edge — DNS, TLS, DDoS/WAF, CDN cache
        │  Cloudflare Tunnel (outbound-only connection, no public port)
        ▼
  ┌─────────────────────────── raspi (192.168.20.236) ───────────────────────────┐
  │                                                                              │
  │  cloudflared container ── joined to the solaris-web docker network ──►      │
  │                                                          nginx container      │
  │                                                          (solaris-web,       │
  │                                                           nginx:alpine)      │
  │                                                          host 9173 → :80     │
  │                                                                              │
  │  Volumes (read-only), from /home/solaris/solaris-web/:                      │
  │    nginx.conf → /etc/nginx/conf.d/default.conf                              │
  │    html/      → /usr/share/nginx/html                                       │
  └──────────────────────────────────────────────────────────────────────────────┘
```

`html/` holds the MkDocs build (`website/site/`) with the Doxygen output copied into
`html/doxygen/` alongside it — one deploy, one rsync, so neither can clobber the other.

### A network quirk worth knowing

`raspi`'s own network segment has **no direct route to the internet** — every outbound
connection is forced through a WireGuard tunnel (`valenciawg`) by the router's policy
routing, on purpose, for every host on that segment. Cloudflare's Tunnel edge network
(`198.41.192.0/24`, `198.41.200.0/24`) is the one exception: the router has a scoped
rule that lets `raspi` specifically reach *just* those two ranges directly over the
normal WAN, because the VPN's egress didn't reliably reach them. If `cloudflared` ever
stops connecting again with `DialContext ... i/o timeout` in its logs, check that
bypass first before assuming it's a Cloudflare-side or container problem.

---

## Repository structure

```
website/
├── mkdocs.yml            # theme, nav, colors
├── requirements.txt      # mkdocs + mkdocs-material, pinned
└── docs/                 # source pages (MkDocs docs_dir)
    ├── index.md
    ├── services/          # per-service API reference, mirrors solaris-v2/spp/services/*/README.md
    └── ...
```

Doxygen's own source lives outside `website/`, under `../doxygen/` and `../Doxyfile`
at the repo root — see `doxygen/README.md`.

---

## Managing the containers on raspi

```bash
ssh raspi
cd ~/solaris-web
docker compose ps
docker compose logs -f

docker restart masenfermeria-cloudflared   # the cloudflared container
```

---

## Deploying changes

**Deployment is automatic.** Any push to `main` that touches `website/**`,
`doxygen/**`, `Doxyfile`, or the SPP source runs
`.github/workflows/deploy-website.yml` on the self-hosted runner (which runs
directly on `raspi`), which:

1. Generates the Doxygen API reference (`doxygen Doxyfile`, via the `solaris-ci`
   Docker image)
2. Builds the MkDocs site
3. Copies the Doxygen output into `website/site/doxygen/`
4. `rsync -av --delete website/site/ /home/solaris/solaris-web/html/`

The nginx container picks up the new files immediately — its volumes are a live
bind mount, no restart needed.

### Manual deploy

```bash
gh workflow run deploy-website.yml --ref main
```
