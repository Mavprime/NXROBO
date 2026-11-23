#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Create redacted copies of CONTEXT_TRANSFER.md and PLAN.md in handoff/."""

from pathlib import Path

ROOT = Path(r"D:\NXROBO")
HANDOFF = ROOT / "auto" / "docs" / "handoff"
HANDOFF.mkdir(parents=True, exist_ok=True)

REPLACEMENTS = {
    "simple2026": "<REDACTED-ROBOT-PASSWORD>",
    "123456": "<REDACTED-ROOT-PASSWORD>",
}


def redact(text):
    for secret, replacement in REPLACEMENTS.items():
        text = text.replace(secret, replacement)
    return text


for source_name in ("CONTEXT_TRANSFER.md", "PLAN.md"):
    source = ROOT / "auto" / "docs" / source_name
    target = HANDOFF / source_name
    target.write_text(redact(source.read_text(encoding="utf-8")), encoding="utf-8")
    print("WROTE:", target, target.stat().st_size, "bytes")

for name in ("CONTEXT_TRANSFER.md", "PLAN.md"):
    text = (HANDOFF / name).read_text(encoding="utf-8")
    leftovers = [s for s in REPLACEMENTS if s in text]
    print("SECRETS LEFT IN %s: %s" % (name, leftovers or "none"))
