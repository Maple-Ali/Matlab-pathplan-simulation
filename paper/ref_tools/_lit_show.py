# -*- coding: utf-8 -*-
import json
from pathlib import Path

p = Path(r"D:\Claude-File\shiyan1\paper\paper_CN_full\_lit_search_results.json")
data = json.loads(p.read_text(encoding="utf-8"))

print("=" * 80)
print("ARXIV RESULTS")
print("=" * 80)
for k, items in data["arxiv"].items():
    print(f"\n### {k} ###")
    if isinstance(items, dict):
        print(items)
        continue
    for it in items:
        print(f"- [{it['published'][:10]}] {it['title']}")
        print(f"  {', '.join(it['authors'][:5])}")
        print(f"  {it['id']}")
        print(f"  {it['summary'][:180]}")

print("\n" + "=" * 80)
print("SEMANTIC SCHOLAR RESULTS")
print("=" * 80)
for k, items in data["semantic_scholar"].items():
    print(f"\n### {k} ###")
    if isinstance(items, dict):
        print(items)
        continue
    for it in items:
        print(f"- [{it['year']}] {it['title']}")
        print(f"  {it['authors'][:80]} | cit={it['citations']} | {it['venue']}")
        print(f"  DOI={it['doi']} arXiv={it['arxiv']}")
        print(f"  {(it['abstract'] or '')[:160]}")
