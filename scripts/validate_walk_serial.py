#!/usr/bin/env python3
"""
Validate serial CSV from main.cpp walking debug line, e.g.:
  St:42 idle:312 Hz:1.2 E:0.0001 Ea:0.0001 mg:1.023 WALKING

Usage:
  pio device monitor | python3 scripts/validate_walk_serial.py
  python3 scripts/validate_walk_serial.py < capture.txt

Checks:
  - Step counter St is non-decreasing (allows 16-bit wrap once).
  - idle resets to 0 when St increases.
  - Optional: warn if state is WALKING while idle > 4000 (tune STEP_IDLE_STOPPED_MS if noisy).
"""

from __future__ import annotations

import re
import sys

LINE_RE = re.compile(
    r"^St:(?P<st>\d+)\s+idle:(?P<idle>\d+)\s+Hz:"
    r"(?P<hz>[-\d.]+)\s+E:(?P<e>[-\d.eE+]+)\s+Ea:(?P<ea>[-\d.eE+]+)\s+mg:"
    r"(?P<mg>[-\d.eE+]+)\s+(?P<state>\S+)"
)


def main() -> int:
    prev_st: int | None = None
    errs = 0
    lines = 0
    for raw in sys.stdin:
        line = raw.strip()
        if not line:
            continue
        m = LINE_RE.match(line)
        if not m:
            continue
        lines += 1
        st = int(m.group("st"))
        idle = int(m.group("idle"))
        state = m.group("state")

        if prev_st is not None:
            if st < prev_st and prev_st < 65000:
                print(f"ERR step decreased: {prev_st} -> {st}", file=sys.stderr)
                errs += 1
            if st > prev_st and idle != 0:
                print(
                    f"WARN idle not 0 after step bump: st {prev_st}->{st} idle={idle}",
                    file=sys.stderr,
                )
                errs += 1

        if state == "WALKING" and idle > 4000:
            print(
                f"WARN WALKING with idle={idle} ms (expect idle < STEP_IDLE_STOPPED_MS)",
                file=sys.stderr,
            )

        prev_st = st

    print(f"Parsed {lines} data lines, {errs} issues", file=sys.stderr)
    return 1 if errs else 0


if __name__ == "__main__":
    sys.exit(main())
