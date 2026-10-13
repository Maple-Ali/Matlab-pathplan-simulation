# -*- coding: utf-8 -*-
import json
from pathlib import Path

p = Path(r"D:\Claude-File\shiyan1\paper\paper_CN_full\_openalex_results.json")
data = json.loads(p.read_text(encoding="utf-8"))
for k, items in data.items():
    print(f"\n### {k} ###")
    if isinstance(items, dict):
        print(items)
        continue
    for it in items:
        print(f"- [{it['year']}] {it['title']}")
        print(f"  {it['authors']} | cit={it['cited_by']} | {it['venue']}")
        print(f"  DOI={it['doi']} vol={it.get('volume')} pp={it.get('first_page')}-{it.get('last_page')}")
