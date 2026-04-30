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

Every deployment generates two integrity files served alongside the site:

| URL | Purpose |
|---|---|
| `/version.json` | Commit, branch, timestamp, and `files_hash` (hash of source files) |
| `/manifest.sha256` | Hash of **all** deployed files, including `version.json` itself |

### 2a. Verify source files (reproducible from git)

`files_hash` inside `version.json` is a SHA-256 over the source files that exist in the
repository. Because `version.json` does not exist in git, this hash can be reproduced from
any clean checkout:

```bash
# 1. Check the commit currently deployed
curl -s https://softwaresolaris.com/version.json

# 2. Clone the repository and check out that commit
git clone https://github.com/Software-Solaris/solaris-software.git
cd solaris-software
git checkout <commit>

# 3. Reproduce the hash
find website/html -type f | sort | xargs sha256sum | awk '{print $1}' | sha256sum
```

The result must match `files_hash`. If it does, every source file on the server is exactly
what is in the repository — nothing added, nothing removed.

### 2b. Verify the complete deployment (including version.json)

`manifest.sha256` covers **all** files in `html/`, including `version.json`. Because
`version.json` is generated at deploy time (it contains the commit SHA and timestamp), you
cannot reproduce it from a plain git checkout — you need to fetch it from the live server.

```bash
# 1. Clone and check out the commit (same as above)
git clone https://github.com/Software-Solaris/solaris-software.git
cd solaris-software
git checkout <commit>

# 2. Download version.json from the live site into the html/ directory
curl -s https://softwaresolaris.com/version.json -o website/html/version.json

# 3. Compute the full deployment hash
find website/html -type f | sort | xargs sha256sum | awk '{print $1}' | sha256sum

# 4. Compare with the manifest on the server
curl -s https://softwaresolaris.com/manifest.sha256
```

Steps 3 and 4 must produce the same hash. This confirms that both the source files **and**
`version.json` are exactly what the pipeline deployed — no file has been swapped or altered
on the server after deploy.

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
