#!/usr/bin/env python3
"""Compare working-tree files with the versions embedded in project_bundle.txt."""
import hashlib
import os
import re
import sys

BUNDLE = r"D:\NXROBO\auto\docs\handoff\project_bundle.txt"
ROOT = r"D:\NXROBO"

FILE_RE = re.compile(r"^FILE \d+/\d+: (.+?)\s*$")
SIZE_RE = re.compile(r"^SIZE:\s+(\d+) lines, (\d+) bytes, sha256=([0-9a-f]{64})\s*$")
SEP_RE = re.compile(r"^={100,}$")


def main():
    with open(BUNDLE, "rb") as fh:
        raw_bundle = fh.read()
    lines = raw_bundle.decode("utf-8").replace("\r\n", "\n").replace("\r", "\n").split("\n")

    entries = []
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
        while end < len(lines) and not FILE_RE.match(lines[end]):
            end += 1
        content_lines = lines[start:end]
        while content_lines and content_lines[-1] == "":
            content_lines.pop()
        if content_lines and SEP_RE.match(content_lines[-1]):
            content_lines.pop()
        while content_lines and content_lines[-1] == "":
            content_lines.pop()
        text = "\n".join(content_lines) + "\n"
        entries.append((path, size, text))
        i = end

    print("parsed %d file blocks from the bundle" % len(entries))
    mismatched = 0
    missing = 0
    for path, (nlines, nbytes, sha), text in entries:
        disk = os.path.join(ROOT, path.replace("/", os.sep))
        if not os.path.exists(disk):
            print("MISSING  %s" % path)
            missing += 1
            continue
        with open(disk, "rb") as fh:
            raw = fh.read()
        norm = raw.replace(b"\r\n", b"\n").replace(b"\r", b"\n")
        disk_sha = hashlib.sha256(norm).hexdigest()
        bundle_sha = hashlib.sha256(text.encode("utf-8")).hexdigest()
        if disk_sha == bundle_sha:
            continue
        mismatched += 1
        print(
            "DIFF    %s\n"
            "         bundle: %d lines, %d bytes, %s  (stated %d/%d/%s)\n"
            "         disk  : %d lines, %d bytes, %s"
            % (
                path,
                text.count("\n"),
                len(text.encode("utf-8")),
                bundle_sha[:12],
                nlines,
                nbytes,
                sha[:12],
                norm.count(b"\n"),
                len(norm),
                disk_sha[:12],
            )
        )
    print("\n%d identical, %d differ, %d missing" % (len(entries) - mismatched - missing, mismatched, missing))
    return 0


if __name__ == "__main__":
    sys.exit(main())
