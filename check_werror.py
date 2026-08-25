#!/usr/bin/env python3
# Copyright 2026 Davide Faconti
# SPDX-License-Identifier: MPL-2.0
"""Re-compile the project with -Werror restored, to catch on macOS what Linux CI would reject.

The root CMakeLists deliberately drops -Werror on Apple Clang (it surfaces diagnostics
GCC does not, which would have blocked the macOS bring-up). The cost is that a warning
which is FATAL on Linux is merely printed here — and printed warnings are easy to miss
in a build log, so a clean-looking macOS build can still break the Linux build. That
has happened.

This replays each entry of build/compile_commands.json verbatim, with -Werror appended
and the object file discarded, so it needs no reconfigure and cannot disturb the build
tree. It is Apple-Clang-accurate, NOT GCC-accurate: it catches the shared warning set
(-Wconversion, -Wunused-result, -Wunused-lambda-capture, ...), which is where the
cross-platform failures have actually come from. It cannot catch a GCC-only diagnostic.

Usage:
    ./check_werror.py                 # whole project
    ./check_werror.py pj_scene3D      # only paths containing this substring
    ./check_werror.py -j4 pj_app
"""

import argparse
import json
import pathlib
import re
import shlex
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor


# Present iff the target was given PJ_WARNING_FLAGS; nothing vendored sets it.
PJ_GOVERNED_FLAG = "-Wold-style-cast"


def compile_entries(db_path: pathlib.Path, needle: str | None):
    entries = json.loads(db_path.read_text())
    seen: set[str] = set()
    out = []
    skipped = 0
    for entry in entries:
        if needle and needle not in entry["file"]:
            continue
        if PJ_GOVERNED_FLAG not in entry["command"]:
            skipped += 1
            continue
        # One entry per file: a source compiled into several targets would otherwise
        # be checked (and reported) repeatedly.
        if entry["file"] in seen:
            continue
        seen.add(entry["file"])
        out.append(entry)
    return out, skipped


def check(entry) -> tuple[str, int, str]:
    argv, skip_next = [], False
    for token in shlex.split(entry["command"]):
        if skip_next:
            skip_next = False
            continue
        if token == "-o":
            skip_next = True
            continue
        argv.append(token)
    argv += ["-Werror", "-o", "/dev/null"]
    proc = subprocess.run(argv, cwd=entry["directory"], capture_output=True, text=True)
    return entry["file"], proc.returncode, proc.stderr


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("filter", nargs="?", help="only check files whose path contains this substring")
    parser.add_argument("-j", "--jobs", type=int, default=10, help="parallel compiles (default 10)")
    parser.add_argument("--build-dir", default="build", help="build directory holding compile_commands.json")
    args = parser.parse_args()

    db_path = pathlib.Path(args.build_dir) / "compile_commands.json"
    if not db_path.exists():
        print(f"error: {db_path} not found — run ./build.sh first", file=sys.stderr)
        return 2

    entries, skipped = compile_entries(db_path, args.filter)
    if not entries:
        print(f"no compile entries match {args.filter!r}", file=sys.stderr)
        return 2

    print(f"replaying {len(entries)} compiles with -Werror "
          f"({skipped} skipped: not built with PJ_WARNING_FLAGS) ...")
    failures = []
    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        for path, code, err in pool.map(check, entries):
            if code:
                failures.append((path, err))

    root = str(pathlib.Path.cwd()) + "/"
    for path, err in failures:
        print("=" * 90)
        print(path.replace(root, ""))
        for line in err.splitlines():
            if re.search(r"\berror:", line):
                print("   " + line.replace(root, ""))

    print(f"\n{len(entries) - len(failures)}/{len(entries)} clean under -Werror")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
