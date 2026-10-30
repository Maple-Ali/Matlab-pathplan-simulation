# -*- coding: utf-8 -*-
"""Verify citation numbers increase with first appearance in the body."""
from pathlib import Path
import re

tex = Path(r"D:\Claude-File\shiyan1\paper\paper_CN_full\paper_CN.tex").read_text(encoding="utf-8")
aux = Path(r"D:\Claude-File\shiyan1\paper\paper_CN_full\paper_CN_new.aux").read_text(encoding="utf-8")

# key -> label number from aux
labels = dict(re.findall(r"\\bibcite\{([^}]+)\}\{([^}]+)\}", aux))
print("bibcite count:", len(labels))

body = tex.split("\\begin{thebibliography}")[0]
print("\nFirst-appearance sequence (key -> number):")
seen = set()
seq = []
for m in re.finditer(r"\\cite\{([^}]+)\}", body):
    for k in [x.strip() for x in m.group(1).split(",") if x.strip()]:
        if k not in seen:
            seen.add(k)
            num = labels.get(k, "?")
            seq.append((k, int(num) if str(num).isdigit() else -1))
            print(f"  {k:8s} -> [{num}]")

nums = [n for _, n in seq]
print("\nnumbers:", nums)
print("strictly increasing 1..N:", nums == list(range(1, len(nums) + 1)))

# Also show first 3 multi-cites as they would display
print("\nSample multi-cites (sorted labels as IEEEtran would show):")
for m in re.finditer(r"\\cite\{([^}]+)\}", body):
    keys = [x.strip() for x in m.group(1).split(",") if x.strip()]
    if len(keys) >= 2:
        labs = sorted(int(labels[k]) for k in keys if k in labels)
        s = max(0, m.start() - 30)
        ctx = body[s : m.end()].replace("\n", " ")[-70:]
        print(f"  {labs}  ...{ctx}")
        if m.start() > 2500:
            break
