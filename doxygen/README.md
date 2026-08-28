# Doxygen API docs

This folder is a placeholder for the generated API reference (every function, struct and macro, pulled from the source comments). It's not published anywhere yet — these are just the instructions to generate it locally in the meantime.

## Requirements

- `doxygen` installed (`sudo apt install doxygen`, or `brew install doxygen` on macOS).
- Optional: `graphviz`, if you want call graphs.

## Generating it

From the repository root:

```bash
doxygen Doxyfile
```

Output goes to `doc/html/`. Open `doc/html/index.html` in a browser.

## Known issue

The root `Doxyfile`'s `INPUT` still points at `solaris-v1/...` paths, which don't exist anymore — the active firmware is `solaris-v2`. Update `INPUT` to `solaris-v2/main` and `solaris-v2/spp` before running it, otherwise the generated docs will be nearly empty.

## TODO

- Fix the `INPUT` paths above.
- Generate and publish the HTML somewhere.
- Update the "API (Doxygen)" link in the website nav to point at it.
