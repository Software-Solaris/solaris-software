# Documentation site (Doxygen)

This is the **C API reference** only — every file, struct, function and macro,
generated straight from the source comments. The narrative guides (roadmap,
philosophy, architecture, SPP design) live on the main website
(`website/`, built with MkDocs) instead, to avoid keeping the same content in
two places. This Doxygen output is published nested under `/doxygen/` on that
site, and opens in its own browser tab — it's Doxygen's own default template
(dark mode, red hue to match the logo), deliberately not restyled to look
like the rest of the site.

## Layout

| Path | What |
|------|------|
| `../Doxyfile`  | configuration |
| `pages/`       | `index.md` (front page) and `spp-api.md` (API tour) — the only two guide pages left here |
| `assets/`      | `logo.png`, used as `PROJECT_LOGO` |

The API side is generated from the source comments in `solaris-v2/main` and
`solaris-v2/spp` (see `INPUT` in the Doxyfile).

## Adding / editing a page here

Only add a page under `pages/` if it's genuinely about the generated API
(cross-referencing files/structs/functions with `@ref`). Anything narrative
belongs in `website/docs/` instead.

## Building locally

From the repository root, with `doxygen` (>= 1.9.5, for `HTML_COLORSTYLE = DARK`):

```bash
doxygen Doxyfile          # output in doc/html/ ; open doc/html/index.html
```

`doc/` is git-ignored.

## CI

`.github/workflows/deploy-website.yml`, on every push to `main` (and on manual
dispatch): generates this Doxygen output, builds the MkDocs site, copies the
Doxygen output into `website/site/doxygen/`, then rsyncs that combined tree to
the web server in one shot — so neither publish step can clobber the other.

## Notes

- `*/README.md` and `*/LICENSE.md` are excluded so stray source-tree markdown
  doesn't turn into pages.
- Source doc-comment warnings (mismatched `@param`, etc.) are pre-existing and
  don't fail the build (`WARN_AS_ERROR = NO`).
