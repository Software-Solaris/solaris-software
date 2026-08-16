## Build the website

The site is built with [MkDocs](https://www.mkdocs.org/) and the
[Material for MkDocs](https://squidfunk.github.io/mkdocs-material/) theme.

```
pip install -r requirements.txt
mkdocs build
```

This generates the static site in `site/`. Source pages live in `docs/`
and the site configuration (theme, navigation, colors) is in `mkdocs.yml`.

To preview locally with live reload:

```
source .venv/bin/activate
mkdocs build
mkdocs serve
```
