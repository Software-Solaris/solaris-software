#!/usr/bin/env python3
"""Enumerate header directories so cppcheck can resolve local #includes.

The Solaris SPP tree keeps headers next to their sources (there are no
central ``include/`` folders), so cppcheck needs an ``-I`` for practically
every module directory.  This script discovers them for you.

Discovery combines two passes:

* **Directory pass** - any directory that physically contains a header
  (``*.h``, ``*.hpp``, ``*.hh``, ``*.hxx``) becomes an include candidate.
* **Quoted-include pass** - every ``#include "..."`` in the sources is
  resolved against the tree.  When the quoted path has more than one
  segment (``#include "core/pubsub/bus.h"``) the *base* directory that
  makes it resolve is added as well.

Examples
--------
Splice straight into a cppcheck invocation::

    cppcheck $(python3 scripts/cppcheck_includes.py solaris-v2/spp solaris-v2/main) ...

Write a file for ``cppcheck --includes-file=``::

    python3 scripts/cppcheck_includes.py solaris-v2/spp \\
        --format lines --output build/cppcheck-includes.txt
"""

from __future__ import annotations

import argparse
import os
import re
import sys
from pathlib import Path
from typing import Iterable

HEADER_SUFFIXES = (".h", ".hpp", ".hh", ".hxx")
SOURCE_SUFFIXES = (".c", ".cc", ".cpp", ".cxx", *HEADER_SUFFIXES)

# Directory names that never hold first-party headers we want to analyse.
DEFAULT_EXCLUDES = (
    ".git", ".cache", ".clangd", ".vscode", ".idea",
    "build", "cmake-build-debug", "cmake-build-release",
    "managed_components", "esp-idf", "archive",
)

_QUOTED_INCLUDE = re.compile(r'^\s*#\s*include\s*"([^"]+)"')


def iter_dirs(root: Path, excludes: frozenset) -> Iterable[Path]:
    """Yield every directory under *root*, pruning *excludes* by name."""
    for dirpath, dirnames, _ in os.walk(root):
        dirnames[:] = sorted(d for d in dirnames if d not in excludes)
        yield Path(dirpath)


def parse_quoted_includes(path: Path) -> set:
    """Return the set of quoted include targets found in one source file."""
    targets: set = set()
    try:
        text = path.read_text(encoding="utf-8", errors="ignore")
    except OSError:
        return targets
    for line in text.splitlines():
        match = _QUOTED_INCLUDE.match(line)
        if match:
            targets.add(match.group(1))
    return targets


def discover(roots: list, excludes: frozenset) -> set:
    """Return the set of directories cppcheck should get as ``-I``."""
    header_dirs: set = set()
    header_paths: list = []
    quoted: set = set()

    for root in roots:
        for directory in iter_dirs(root, excludes):
            has_header = False
            for entry in directory.iterdir():
                if not entry.is_file():
                    continue
                if entry.suffix in HEADER_SUFFIXES:
                    has_header = True
                    header_paths.append(entry)
                if entry.suffix in SOURCE_SUFFIXES:
                    quoted |= parse_quoted_includes(entry)
            if has_header:
                header_dirs.add(directory)

    # Resolve multi-segment quoted includes back to their base directory.
    for target in quoted:
        norm = target.replace("\\", "/").lstrip("./")
        if "/" not in norm:
            continue  # bare name -> already covered by the directory pass
        suffix = "/" + norm
        for header in header_paths:
            posix = header.as_posix()
            if posix.endswith(suffix):
                base = Path(posix[: -len(suffix)])
                if base.is_dir():
                    header_dirs.add(base)
    return header_dirs


def format_output(dirs: list, style: str) -> str:
    if style == "lines":
        return "\n".join(dirs)
    if style == "args":
        return " ".join(f"-I {d}" for d in dirs)
    if style == "newline-args":
        return "\n".join(f"-I{d}" for d in dirs)
    raise ValueError(f"unknown format: {style}")


def main(argv: list | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("roots", nargs="+", type=Path,
                        help="source trees to scan")
    parser.add_argument("-f", "--format", default="args",
                        choices=("args", "lines", "newline-args"),
                        help="output style (default: args)")
    parser.add_argument("-o", "--output", type=Path,
                        help="write here instead of stdout")
    parser.add_argument("-x", "--exclude", action="append", default=[],
                        metavar="NAME",
                        help="extra directory name to prune (repeatable)")
    parser.add_argument("--absolute", action="store_true",
                        help="emit absolute paths (default: relative to CWD)")
    args = parser.parse_args(argv)

    missing = [str(r) for r in args.roots if not r.is_dir()]
    if missing:
        parser.error("not a directory: " + ", ".join(missing))

    excludes = frozenset(DEFAULT_EXCLUDES) | frozenset(args.exclude)
    found = discover([r.resolve() for r in args.roots], excludes)

    cwd = Path.cwd()
    rendered: list = []
    for directory in sorted(found):
        if args.absolute:
            rendered.append(str(directory))
            continue
        try:
            rendered.append(str(directory.relative_to(cwd)))
        except ValueError:
            rendered.append(str(directory))

    payload = format_output(rendered, args.format)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload + "\n", encoding="utf-8")
        print(f"{len(rendered)} include dirs -> {args.output}", file=sys.stderr)
    else:
        print(payload)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
