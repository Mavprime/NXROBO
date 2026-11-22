#!/usr/bin/env python3
"""Extract the file blocks embedded in project_bundle.txt to .tmp/bundle/."""
import os
import re

BUNDLE = r"D:\NXROBO\auto\docs\handoff\project_bundle.txt"
OUT = r"D:\NXROBO\.tmp\bundle"

FILE_RE = re.compile(r"^FILE \d+/\d+: (.+?)\s*$")
SIZE_RE = re.compile(r"^SIZE:\s+(\d+) lines, (\d+) bytes, sha256=([0-9a-f]{64})\s*$")
SEP_RE = re.compile(r"^={100,}$")
SECTION_RE = re.compile(r"^SECTION \d+")


def main():
    with open(BUNDLE, "rb") as fh:
        raw = fh.read()
    lines = raw.decode("utf-8").replace("\r\n", "\n").replace("\r", "\n").split("\n")

    count = 0
    i = 0
    while i < len(lines):
        m = FILE_RE.match(lines[i])
        if not m:
            i += 1
            continue
        path = m.group(1)
        j = i + 1
        size = None
        while j < len(lines) and j < i + 5:
            sm = SIZE_RE.match(lines[j])
            if sm:
                size = (int(sm.group(1)), int(sm.group(2)), sm.group(3))
                break
            j += 1
        if size is None:
            i += 1
            continue
        k = j
        while k < len(lines) and not SEP_RE.match(lines[k]):
            k += 1
        start = k + 1
        if start < len(lines) and lines[start] == "":
            start += 1
        end = start
        while end < len(lines) and not FILE_RE.match(lines[end]) and not SECTION_RE.match(lines[end]):
            end += 1
        content = lines[start:end]
        while content and content[-1] == "":
            content.pop()
        if content and SEP_RE.match(content[-1]):
            content.pop()
        while content and content[-1] == "":
            content.pop()
        text = "\n".join(content) + "\n"
        dest = os.path.join(OUT, path.replace("/", os.sep))
        os.makedirs(os.path.dirname(dest), exist_ok=True)
        with open(dest, "w", encoding="utf-8", newline="\n") as fh:
            fh.write(text)
        count += 1
        i = end
    print("extracted %d files to %s" % (count, OUT))


if __name__ == "__main__":
    main()
