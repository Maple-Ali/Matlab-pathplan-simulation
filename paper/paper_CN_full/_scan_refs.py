# -*- coding: utf-8 -*-
import re
from pathlib import Path

p = Path(r"D:\Claude-File\shiyan1\paper\paper_CN_full\paper_CN.tex")
text = p.read_text(encoding="utf-8")

print("=== TITLE / ABSTRACT / KEYWORDS ===")
start = text.find("\\title{")
end = text.find("\\section{引言}")
print(text[start:end])

print("\n=== SECTIONS ===")
for m in re.finditer(r"\\(?:sub)*section\{([^}]+)\}", text):
    print(m.group(0)[:100])

print("\n=== CITE KEYS USED ===")
keys = re.findall(r"\\(?:cite|ref)\{([^}]+)\}", text)
all_keys = []
for k in keys:
    for part in k.split(","):
        all_keys.append(part.strip())
print(sorted(set(all_keys)))
print("unique keys:", len(set(all_keys)))

print("\n=== BIBLIOGRAPHY ===")
idx = text.find("\\begin{thebibliography}")
print(text[idx:idx + 15000] if idx >= 0 else "NO BIB")
