#!/usr/bin/env python3
"""Build a cppcheck ``--rule-texts`` file for the MISRA addon.

cppcheck ships the MISRA addon but *not* the guideline wording, because the
MISRA C:2012 text is copyrighted by the MISRA Consortium and may not be
redistributed.  This script produces the rule-text file in either of two
ways, and never contains any guideline text itself:

* ``--from-xml FILE``  - **public-safe.**  Reads a cppcheck result XML and
  emits, for every ``misra-c2012-*`` rule it finds, a neutral pointer line
  ("See MISRA C:2025 section X.Y ...").  No copyrighted text, so the output
  can live in a public repo.  The report then shows the rule number plus a
  "look it up" note instead of cppcheck's generic placeholder.

* ``SOURCE``           - **licensed use only.**  Reformats a plain-text
  extract that *you* copied from *your* licensed MISRA C:2012/2023/2025
  PDF ("Appendix A - Summary of guidelines") into the same format.  Do not
  commit the result to a public repo.

Output format
-------------
``cppcheck``'s ``misra.py`` only reads text that appears *after* a literal
``Appendix A Summary of guidelines`` line, in ``Rule X.Y`` blocks, so the
file is always wrapped that way.  Directives (``Dir X.Y``) are dropped -
the addon has no directive-text support.

Usage
-----
Public pointer file from a cppcheck run::

    cppcheck --addon=misra --xml ... 2> cppcheck-result.xml
    python3 scripts/misra_ruletexts.py --from-xml cppcheck-result.xml \\
        --pointer-edition "MISRA C:2025" -o misra_rules.generated.txt

From your licensed copy::

    python3 scripts/misra_ruletexts.py misra_src.txt -o misra_rules.txt   # keep private
"""

from __future__ import annotations

import argparse
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

# misra.py starts parsing only after this exact marker; it stops at "Appendix B".
_HEADER = "Appendix A Summary of guidelines"
_FOOTER = "Appendix B"

# Start-of-guideline gate for a licensed-copy extract.
_STRICT = re.compile(
    r"^\s*(?:(?:Rule|Dir|Directive)\s+)?\d{1,2}\.\d{1,2}(?![.\d])"
)
_MARKER = re.compile(
    r"""^\s*
        (?:(?P<kind>Rule|Dir|Directive)\s+)?
        (?P<num>\d{1,2}\.\d{1,2})
        \s*
        (?:[\(\[]?\s*(?P<cat>Required|Advisory|Mandatory|R|A|M)\s*[\)\]]?)?
        \s*[:.\-–]?\s*
        (?P<text>.*)$
    """,
    re.VERBOSE,
)
_CATEGORY = {"R": "Required", "A": "Advisory", "M": "Mandatory"}
_MISRA_ID = re.compile(r"^misra-c2012-(?P<dir>dir-)?(?P<num>\d{1,2}\.\d{1,2})$")

# Every MISRA C:2012 rule the bundled cppcheck ``misra`` addon can emit
# (derived from misra.py: getAddonRules() + getCppcheckRules()).  Used by
# --all-rules so any finding gets a pointer, not just the ones seen so far.
_ALL_RULES = (
    "1.2", "1.3", "1.4", "2.1", "2.2", "2.3", "2.4", "2.5", "2.6", "2.7",
    "3.1", "3.2", "4.1", "4.2", "5.1", "5.2", "5.3", "5.4", "5.5", "5.6",
    "5.7", "5.8", "5.9", "6.1", "6.2", "7.1", "7.2", "7.3", "7.4", "8.1",
    "8.2", "8.3", "8.4", "8.5", "8.6", "8.7", "8.8", "8.9", "8.10", "8.11",
    "8.12", "8.13", "8.14", "9.1", "9.2", "9.3", "9.4", "9.5", "10.1", "10.2",
    "10.3", "10.4", "10.5", "10.6", "10.7", "10.8", "11.1", "11.2", "11.3",
    "11.4", "11.5", "11.6", "11.7", "11.8", "11.9", "12.1", "12.2", "12.3",
    "12.4", "13.1", "13.2", "13.3", "13.4", "13.5", "13.6", "14.1", "14.2",
    "14.3", "14.4", "15.1", "15.2", "15.3", "15.4", "15.5", "15.6", "15.7",
    "16.1", "16.2", "16.3", "16.4", "16.5", "16.6", "16.7", "17.1", "17.2",
    "17.3", "17.4", "17.5", "17.6", "17.7", "17.8", "18.1", "18.2", "18.3",
    "18.4", "18.5", "18.6", "18.7", "18.8", "19.1", "19.2", "20.1", "20.2",
    "20.3", "20.4", "20.5", "20.6", "20.7", "20.8", "20.9", "20.10", "20.11",
    "20.12", "20.13", "20.14", "21.1", "21.2", "21.3", "21.4", "21.5", "21.6",
    "21.7", "21.8", "21.9", "21.10", "21.11", "21.12", "21.13", "21.14",
    "21.15", "21.16", "21.17", "21.18", "21.19", "21.20", "21.21", "22.1",
    "22.2", "22.3", "22.4", "22.5", "22.6", "22.7", "22.8", "22.9", "22.10",
)

# Base MISRA C:2012 (pre-Amendment) guideline count, for a sanity hint only.
_EXPECTED_RULES = 143
_EXPECTED_DIRECTIVES = 17


def _normalise_ws(value: str) -> str:
    return re.sub(r"\s+", " ", value).strip()


def _num_key(num: str) -> tuple:
    major, minor = num.split(".")
    return int(major), int(minor)


def parse_extract(lines: list, stop_blank: bool) -> tuple:
    """Return (rules, n_directives) from a licensed-copy extract.

    ``rules`` is a list of ``(num, category, text)``; directives are counted
    only (the addon cannot use their text).
    """
    parsed: list = []
    cur_num = None
    cur_kind = "Rule"
    cur_cat = ""
    buf: list = []

    def flush() -> None:
        if cur_num is not None:
            parsed.append((cur_kind, cur_num, cur_cat, _normalise_ws(" ".join(buf))))

    for raw in lines:
        if _STRICT.match(raw):
            match = _MARKER.match(raw)
            flush()
            kind = match.group("kind") or "Rule"
            cur_kind = "Dir" if kind.startswith(("Dir", "Direc")) else "Rule"
            cur_num = match.group("num")
            raw_cat = match.group("cat") or ""
            cur_cat = _CATEGORY.get(raw_cat.upper(), raw_cat.title()) if raw_cat else ""
            buf = [match.group("text")] if match.group("text") else []
            continue
        if stop_blank and not raw.strip():
            flush()
            cur_num, buf = None, []
            continue
        if cur_num is not None:
            buf.append(raw.strip())
    flush()

    seen: dict = {}
    for kind, num, cat, text in parsed:  # last definition wins, order kept
        seen[(kind, num)] = (kind, num, cat, text)

    rules = [(num, cat, text) for (kind, num), (_, _, cat, text) in seen.items()
             if kind == "Rule"]
    n_dirs = sum(1 for kind, _ in seen if kind == "Dir")
    rules.sort(key=lambda item: _num_key(item[0]))
    return rules, n_dirs


def parse_xml(path: Path, edition: str) -> tuple:
    """Return (rules, n_directives) of neutral pointer entries from a cppcheck XML."""
    root = ET.parse(path).getroot()
    rule_nums: set = set()
    dir_nums: set = set()
    for error in root.iter("error"):
        match = _MISRA_ID.match(error.get("id", ""))
        if not match:
            continue
        (dir_nums if match.group("dir") else rule_nums).add(match.group("num"))

    rules = [
        (num, "",
         f"See {edition} section {num} for the guideline wording, "
         f"category and rationale.")
        for num in sorted(rule_nums, key=_num_key)
    ]
    return rules, len(dir_nums)


def render(rules: list) -> str:
    out = [_HEADER, ""]
    for num, cat, text in rules:
        out.append(f"Rule {num}")
        if cat:
            out.append(cat)
        out.append(text or "TODO paste guideline wording from your licensed copy")
        out.append("")
    out.append(_FOOTER)
    return "\n".join(out)


def main(argv: list | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("source", type=Path, nargs="?",
                        help="plain-text extract from your licensed MISRA copy "
                             "('-' for stdin)")
    parser.add_argument("--from-xml", type=Path, metavar="FILE",
                        help="cppcheck result XML; emit neutral pointer lines "
                             "(public-safe, no copyrighted text)")
    parser.add_argument("--all-rules", action="store_true",
                        help="emit a neutral pointer line for every rule the "
                             "cppcheck misra addon can report (public-safe; "
                             "no XML needed, single-pass)")
    parser.add_argument("--pointer-edition", default="MISRA C:2023",
                        help="edition named in pointer lines (default: MISRA C:2023)")
    parser.add_argument("-o", "--output", type=Path,
                        help="write here instead of stdout")
    parser.add_argument("--stop-blank", action="store_true",
                        help="end a guideline's text at the first blank line")
    args = parser.parse_args(argv)

    chosen = sum(bool(x) for x in (args.source, args.from_xml, args.all_rules))
    if chosen != 1:
        parser.error("give exactly one of SOURCE, --from-xml or --all-rules")

    if args.all_rules:
        rules = [
            (num, "",
             f"See {args.pointer_edition} section {num} for the guideline "
             f"wording, category and rationale.")
            for num in _ALL_RULES
        ]
        n_dirs = 0
    elif args.from_xml:
        if not args.from_xml.is_file():
            parser.error(f"no such file: {args.from_xml}")
        rules, n_dirs = parse_xml(args.from_xml, args.pointer_edition)
        if not rules:
            parser.error("no 'misra-c2012-<rule>' findings in the XML")
    else:
        if str(args.source) == "-":
            raw = sys.stdin.read()
        elif args.source.is_file():
            raw = args.source.read_text(encoding="utf-8", errors="ignore")
        else:
            parser.error(f"no such file: {args.source}")
        rules, n_dirs = parse_extract(raw.splitlines(), stop_blank=args.stop_blank)
        if not rules:
            parser.error("no 'Rule X.Y' markers found in the input")

    payload = render(rules)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload + "\n", encoding="utf-8")
        dest = str(args.output)
    else:
        print(payload)
        dest = "stdout"

    print(f"{len(rules)} rule texts -> {dest}", file=sys.stderr)
    if n_dirs:
        print(f"note: {n_dirs} directive(s) dropped - the cppcheck misra addon "
              "has no directive-text support.", file=sys.stderr)
    if not args.from_xml and len(rules) < _EXPECTED_RULES:
        print(f"note: base MISRA C:2012 has {_EXPECTED_RULES} rules "
              "(Amendments 1-4 add more); the extract looks incomplete.",
              file=sys.stderr)
    if "TODO " in payload:
        print("note: some rules have no text - fill the TODO lines from your "
              "licensed copy.", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
