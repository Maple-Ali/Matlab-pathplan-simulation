# -*- coding: utf-8 -*-
"""Resolve authors for incomplete bib entries by title search."""
import json
import ssl
import time
import urllib.parse
import urllib.request
from pathlib import Path

CTX = ssl._create_unverified_context()
UA = {"User-Agent": "research-lit/1.0"}

titles = {
    "ref4": "A survey on approximability of traveling salesman problems using the TSP-T3CO definition scheme",
    "ref8": "A comprehensive review of improved A* path planning algorithms and their hybrid integrations",
    "ref13": "Improved JPS and DWA path planning algorithm for mobile robots",
    "ref14": "Research on mobile robot path planning based on an improved bidirectional jump point search algorithm",
    "ref15": "Adaptive weight optimization based jump point Theta* algorithm in mobile robot path planning for intricate environments",
    "ref16": "UGV path optimization in UAV-assisted environments using visibility-aware path simplification",
    "ref17": "Enhancing the safety and smoothness of path planning through an integration of Dijkstra's algorithm and piecewise cubic Bezier optimization",
    "ref18": "Adaptive A*/NSGA-II framework for multi-goal navigation of mobile robots",
    "ref26": "An improvement to the 2-opt heuristic algorithm for approximation of optimal TSP tour",
    "ref27": "Matrix-based ant colony system for traveling salesman problem",
    "ref28": "nLKH-ACS: A niching Lin-Kernighan-Helsgaun-based ant colony system for multisolution traveling salesman problems",
    "ref29": "CAACS: A carbon aware ant colony system",
    "ref30": "Ant colony optimization for multiple traveling salesmen problem with revisitable cities",
}

def search_title(q):
    params = {"search": q, "per-page": 3}
    url = "https://api.openalex.org/works?" + urllib.parse.urlencode(params)
    req = urllib.request.Request(url, headers=UA)
    with urllib.request.urlopen(req, timeout=40, context=CTX) as r:
        data = json.loads(r.read().decode("utf-8"))
    out = []
    for w in data.get("results", [])[:2]:
        authors = [a["author"]["display_name"] for a in (w.get("authorships") or [])]
        bib = w.get("biblio") or {}
        src = ((w.get("primary_location") or {}) or {}).get("source") or {}
        out.append({
            "title": w.get("title"),
            "year": w.get("publication_year"),
            "authors": authors,
            "venue": src.get("display_name"),
            "doi": w.get("doi"),
            "volume": bib.get("volume"),
            "issue": bib.get("issue"),
            "first_page": bib.get("first_page"),
            "last_page": bib.get("last_page"),
        })
    return out

results = {}
for k, t in titles.items():
    print("Q", k, flush=True)
    try:
        results[k] = search_title(t)
    except Exception as e:
        results[k] = {"error": str(e)}
    time.sleep(0.35)

Path(r"D:\Claude-File\shiyan1\paper\paper_CN_full\_ref_authors_fix.json").write_text(
    json.dumps(results, ensure_ascii=False, indent=2), encoding="utf-8"
)
for k, v in results.items():
    print("\n", k)
    if isinstance(v, dict):
        print(v)
    else:
        for it in v:
            print(" ", it["year"], it["title"])
            print("  ", "; ".join(it["authors"]))
            print("  ", it["venue"], it.get("volume"), it.get("issue"), it.get("first_page"), it.get("last_page"), it.get("doi"))
