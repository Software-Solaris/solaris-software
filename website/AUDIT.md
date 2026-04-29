# Privacy & Audit Guide

## No logs. No tracking. No privacy policy needed.

This website collects nothing. No cookies, no analytics, no access logs, no error logs. There is nothing to disclose because there is nothing being stored.

This document explains how to verify that independently.

---

## 1. Verify the server logs nothing

Open `website/nginx.conf` in this repository. You will find:

```nginx
access_log off;
error_log /dev/null;
```

`access_log off` disables HTTP access logs entirely. `error_log /dev/null` discards any error output. No visitor request is ever written to disk.

---

## 2. Verify what is deployed matches the source code

Every deployment generates a `version.json` file served at:

```
http://<server>/version.json
```

Example response:

```json
{
  "commit": "b880f4d...",
  "ref": "main",
  "deployed_at": "2026-04-29T18:55:00Z",
  "files_hash": "a3f8c2d1..."
}
```

To verify the deployed files match the commit:

```bash
# 1. Clone the repository and check out the commit shown in version.json
git clone https://github.com/Software-Solaris/solaris-software.git
git checkout <commit>

# 2. Reproduce the hash locally
find website/html -type f | sort | xargs sha256sum | sha256sum
```

The result must match `files_hash` in `version.json`. If it does, the files running on the server are exactly what is in the repository at that commit — nothing added, nothing removed.

---

## 3. Pipeline enforcement of privacy directives

Every push that touches `website/` — on any branch, before any deploy — runs an automated check that verifies:

- `access_log off` is present in `nginx.conf`
- `error_log /dev/null` is present in `nginx.conf`
- `real_ip_header` is **not** present (ensures visitor IPs are never restored from Cloudflare headers)

If any of these checks fail, the pipeline aborts and deploy is blocked. This means no version of the site can ever be deployed with logging enabled or with IP restoration active. You can inspect this step in `.github/workflows/deploy-website.yml` under **Verify privacy directives**.

---

## 4. Verify the deployment pipeline

The pipeline that builds and deploys this site is defined in:

```
.github/workflows/deploy-website.yml
```

It runs a security scan (Trivy) on every push that touches `website/`, and only deploys to the server on merges to `main`. The pipeline runs on a self-hosted runner — you can inspect every step in the GitHub Actions tab of this repository.

---

## 4. IP address anonymization

This site sits behind Cloudflare. The origin server (nginx) never receives the visitor's real IP — it only ever sees Cloudflare's proxy IPs. This is because nginx deliberately does not process the `CF-Connecting-IP` or `X-Forwarded-For` headers that would restore the real IP. You can verify this in `nginx.conf`: there is no `real_ip_header` directive.

As an additional safeguard, even those Cloudflare IPs are anonymized before reaching any log via the `map` directive at the top of `nginx.conf`, which truncates the last octet (e.g. `192.168.1.123 → 192.168.1.0`). Combined with logging being fully disabled, no IP ever touches disk at any point.

---

## No privacy policy

There is no privacy policy because there is no data collection of any kind.
