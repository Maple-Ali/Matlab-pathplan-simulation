# -*- coding: utf-8 -*-
"""Smaller OpenAlex batch."""
import json
import ssl
import time
import urllib.parse
import urllib.request
from pathlib import Path

OUT = Path(r"D:\Claude-File\shiyan1\paper\paper_CN_full\_openalex_results.json")
CTX = ssl._create_unverified_context()
UA = {"User-Agent": "research-lit/1.0"}

def search(q, per_page=10, year="2020-01-01", sort="relevance_score:desc"):
    params = {
        "search": q,
        "per-page": per_page,
        "sort": sort,
        "filter": f"from_publication_date:{year}",
    }
    url = "https://api.openalex.org/works?" + urllib.parse.urlencode(params)
    req = urllib.request.Request(url, headers=UA)
    with urllib.request.urlopen(req, timeout=60, context=CTX) as r:
        data = json.loads(r.read().decode("utf-8"))
    out = []
    for w in data.get("results", []):
        authors = ", ".join(a["author"]["display_name"] for a in (w.get("authorships") or [])[:5])
        bib = w.get("biblio") or {}
        src = ((w.get("primary_location") or {}) or {}).get("source") or {}
        out.append({
            "title": w.get("title"),
            "year": w.get("publication_year"),
            "date": w.get("publication_date"),
            "authors": authors,
            "venue": src.get("display_name"),
            "doi": w.get("doi"),
            "cited_by": w.get("cited_by_count"),
            "volume": bib.get("volume"),
            "issue": bib.get("issue"),
            "first_page": bib.get("first_page"),
            "last_page": bib.get("last_page"),
        })
    return out

queries = [
    ("path_survey", "survey path planning mobile robot"),
    ("astar_review", "A* path planning algorithm review improved"),
    ("multi_goal", "multi-goal path planning robot traveling salesman"),
    ("open_tsp", "open traveling salesman problem"),
    ("tsp_obs", "traveling salesman obstacles robot path"),
    ("tsp_survey", "traveling salesman problem survey metaheuristic"),
    ("aco_hybrid", "ant colony optimization hybrid local search traveling salesman"),
    ("jps", "jump point search pathfinding grid"),
    ("smooth", "path smoothing robot spline"),
    ("nav_review", "mobile robot navigation survey"),
]

results = {}
for name, q in queries:
    print("OA", name, flush=True)
    try:
        results[name] = search(q)
    except Exception as e:
        results[name] = {"error": str(e)}
        print("ERR", name, e, flush=True)
    time.sleep(0.3)

OUT.write_text(json.dumps(results, ensure_ascii=False, indent=2), encoding="utf-8")
print("done", sum(1 for v in results.values() if isinstance(v, list)), "ok")
