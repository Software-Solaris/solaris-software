## Build the webiste 

```
cmake -S md4c -B build
cmake --build build
make
```

The `cmake` steps build the `md2html` converter (from the `md4c` submodule).
The `make` step uses that converter to generate `output/*.html` from `src/*.md`,
and copies `style.css` and `assets/` into `output/`.
