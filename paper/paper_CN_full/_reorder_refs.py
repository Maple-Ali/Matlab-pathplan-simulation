# -*- coding: utf-8 -*-
"""Reorder thebibliography so numeric labels follow first-citation order in the body.

Keys stay the same; only \bibitem order changes (IEEEtran numbers by bibitem order).
"""
from pathlib import Path
import re

path = Path(r"D:\Claude-File\shiyan1\paper\paper_CN_full\paper_CN.tex")
text = path.read_text(encoding="utf-8")

# Split body / bibliography
bib_m = re.search(r"(\\begin\{thebibliography\}\{[^}]*\})(.*?)(\\end\{thebibliography\})", text, re.S)
if not bib_m:
    raise SystemExit("no bibliography")

body = text[: bib_m.start()]
bib_body = bib_m.group(2)
bib_header = bib_m.group(1)
bib_footer = bib_m.group(3)

# Parse existing bibitems: key -> full block (without the \bibitem{key} line content after)
items = {}
order_in_bib = []
for m in re.finditer(r"(\\bibitem\{([^}]+)\})(.*?)(?=\\bibitem\{|\Z)", bib_body, re.S):
    full = m.group(1) + m.group(3)
    key = m.group(2)
    items[key] = full
    order_in_bib.append(key)

print("bib keys in file order:", order_in_bib)

# First-appearance order of keys in body (\cite{a,b,c})
first_order = []
seen = set()
for m in re.finditer(r"\\cite\{([^}]+)\}", body):
    for k in m.group(1).split(","):
        k = k.strip()
        if not k:
            continue
        if k not in seen:
            seen.add(k)
            first_order.append(k)

print("first-appearance order:", first_order)

missing_in_bib = [k for k in first_order if k not in items]
unused = [k for k in order_in_bib if k not in seen]
print("cited but no bibitem:", missing_in_bib)
print("bibitem never cited:", unused)

if missing_in_bib:
    raise SystemExit("cannot reorder: missing bibitems")

# Build new bib body
new_parts = []
for k in first_order:
    block = items[k]
    # normalize: ensure \bibitem{key} starts the block and ends with blank line
    if not block.endswith("\n"):
        block += "\n"
    if not block.endswith("\n\n"):
        block += "\n"
    new_parts.append(block)

# Keep any uncited bibitems at the end (shouldn't happen)
for k in order_in_bib:
    if k not in seen:
        block = items[k]
        if not block.endswith("\n"):
            block += "\n"
        if not block.endswith("\n\n"):
            block += "\n"
        new_parts.append(block)

new_bib_body = "\n".join(new_parts)
# Update thebibliography width argument to count
n = len(first_order) + len(unused)
bib_header_new = f"\\begin{{thebibliography}}{{{n}}}"

new_text = text[: bib_m.start()] + bib_header_new + "\n" + new_bib_body + bib_footer + text[bib_m.end() :]

# Print mapping: old position -> new number
print("\nNew numbering (first appearance):")
for i, k in enumerate(first_order, 1):
    print(f"  [{i:2d}] {k}")

path.write_text(new_text, encoding="utf-8")
print("\nWROTE", path)
print("entries:", n)
