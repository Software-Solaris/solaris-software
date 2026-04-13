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

1. Edit the files under `html/` in this repository.
2. Push them to the VPS:

```bash
scp website/html/* raspi:~/solaris-web/html/
```

The nginx container mounts `html/` as a read-only volume — **changes are live
immediately**, no container restart needed. Only `nginx.conf` or `docker-compose.yml`
changes require `docker compose up -d`.
