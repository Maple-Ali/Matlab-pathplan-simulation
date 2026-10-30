# -*- coding: utf-8 -*-
"""Targeted OpenAlex lookups for OTSP / CETSP / TSPN / multi-goal classics."""
import json
import ssl
import time
import urllib.parse
import urllib.request
from pathlib import Path

OUT = Path(r"D:\Claude-File\shiyan1\paper\paper_CN_full\_openalex_extra.json")
CTX = ssl._create_unverified_context()
UA = {"User-Agent": "research-lit/1.0"}

def search(q, per_page=8, year=None, sort="relevance_score:desc"):
    params = {"search": q, "per-page": per_page, "sort": sort}
    if year:
        params["filter"] = f"from_publication_date:{year}-01-01"
    url = "https://api.openalex.org/works?" + urllib.parse.urlencode(params)
    req = urllib.request.Request(url, headers=UA)
    with urllib.request.urlopen(req, timeout=50, context=CTX) as r:
        data = json.loads(r.read().decode("utf-8"))
    out = []
    for w in data.get("results", []):
        authors = ", ".join(a["author"]["display_name"] for a in (w.get("authorships") or [])[:6])
        bib = w.get("biblio") or {}
        src = ((w.get("primary_location") or {}) or {}).get("source") or {}
        out.append({
            "title": w.get("title"),
            "year": w.get("publication_year"),
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

queries = {
    "open_tsp_classic": "open traveling salesman problem Fischetti Toth Vigo",
    "cetsp": "close-enough traveling salesman problem neighborhoods",
    "tspn": "traveling salesman problem with neighborhoods robot",
    "tsp_obstacles_classic": "traveling salesman problem with obstacles visibility",
    "multi_goal_smug": "SMUG planner safe multi-goal planner mobile robots",
    "astar_weighted": "weighted A* anytime repairing path planning",
    "aco_mmas": "MAX-MIN ant system traveling salesman",
    "vnd_tsp": "variable neighborhood descent ant colony traveling salesman",
    "logistics_multi_goal": "multi-goal path planning warehouse logistics robot",
    "inspection_tsp": "inspection path planning robot traveling salesman",
    "tsp_survey_recent": "traveling salesman problem review 2024 2025",
    "path_simplification": "path simplification robot visibility line of sight",
}

results = {}
for name, q in queries.items():
    print("X", name, flush=True)
    try:
        results[name] = search(q)
    except Exception as e:
        results[name] = {"error": str(e)}
        print("ERR", e, flush=True)
    time.sleep(0.35)

OUT.write_text(json.dumps(results, ensure_ascii=False, indent=2), encoding="utf-8")
print("done")
