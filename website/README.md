# Solaris Web

Static website for the Solaris project, served by an **nginx** container on a remote VPS
and published at [softwaresolaris.com](https://softwaresolaris.com) via a Cloudflare Tunnel.

---

## Request flow

```
  ┌─────────────────────────────────────────────┐
  │  Browser                                    │
  │  https://softwaresolaris.com                │
  └──────────────────────┬──────────────────────┘
                         │  HTTPS
                         ▼
  ╔══════════════════════════════════════════════╗
  ║  Cloudflare Edge                             ║
  ║                                              ║
  ║  · DNS resolution                            ║
  ║  · TLS termination (HTTPS certificate)       ║
  ║  · DDoS protection & WAF                     ║
  ║  · CDN cache for static assets               ║
  ╚══════════════════════╤═══════════════════════╝
                         │
                         │  Cloudflare Tunnel
                         │  Outbound connection from VPS
                         │  No public ports exposed
                         │
  ───────────────────────┼──────────── VPS ────────
                         │
                         ▼
             ┌───────────────────────┐
             │  cloudflared  daemon  │
             │  (persistent tunnel)  │
             └───────────┬───────────┘
                         │  localhost:9173
                         ▼
  ╔══════════════════════════════════════════════╗
  ║  Podman container · solaris-web              ║
  ║  Image : nginx:alpine                        ║
  ║  Port  : 9173 (host)  →  80 (container)      ║
  ╠══════════════════════════════════════════════╣
  ║  Volumes (read-only)                         ║
  ║  ./html/       →  /usr/share/nginx/html      ║
  ║  ./nginx.conf  →  /etc/nginx/conf.d/         ║
  ╠══════════════════════════════════════════════╣
  ║  Static files served                         ║
  ║  index.html  ·  logo.svg  ·  banner.png      ║
  ╚══════════════════════════════════════════════╝
```

> The VPS exposes **no public HTTP/HTTPS ports**. The `cloudflared` daemon maintains a
> persistent outbound connection to Cloudflare's edge — all traffic is routed inward
> through that tunnel.

---

## Repository structure

```
website/
├── docker-compose.yml   # Container definition  (restart: always)
├── nginx.conf           # nginx config: gzip, security headers, cache policy
└── html/
    ├── index.html           # Main page
    ├── logo.svg             # Solaris logo
    ├── banner.png           # Banner image
    └── bannerPixelart.png   # Pixel-art banner variant
```

The `website/` folder in this repository is the **source of truth**.
The live files run from `~/solaris-web/` on the VPS.

---

## Managing the container

```bash
ssh raspi
cd ~/solaris-web

docker compose up -d      # start (or restart after a config change)
docker compose down       # stop
docker compose logs -f    # follow logs
```

> `docker` and `docker-compose` are aliased to `podman` / `podman compose` on the VPS.

---

## Deploying changes

**Deployment is automatic.** Any push to `main` that modifies files under `website/` triggers the CI/CD pipeline (`.github/workflows/deploy-website.yml`), which:

1. Verifies privacy directives in `nginx.conf` are intact (blocks deploy if tampered)
2. Scans for secrets and misconfigurations (Trivy)
3. Copies the files to `~/solaris-web/` on the server
4. Restarts the container

No manual steps needed — merge to `main` and the pipeline handles the rest.

### Manual deploy (if needed)

To trigger the pipeline without a code change:

```bash
gh workflow run deploy-website.yml
```

Or to deploy directly to the server, bypassing the pipeline:

```bash
scp -r website/html/* raspi:~/solaris-web/html/
ssh raspi "cd ~/solaris-web && docker compose up -d"
```
