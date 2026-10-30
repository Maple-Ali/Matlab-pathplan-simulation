# -*- coding: utf-8 -*-
"""Retry literature search with SSL workaround and slower S2 pacing."""
import json
import ssl
import time
import urllib.parse
import urllib.request
import xml.etree.ElementTree as ET
from pathlib import Path

OUT = Path(r"D:\Claude-File\shiyan1\paper\paper_CN_full\_lit_search_results.json")
CTX = ssl._create_unverified_context()
UA = {"User-Agent": "Mozilla/5.0 (Windows NT 10.0; Win64; x64) research-lit/1.0"}

ARXIV = "https://export.arxiv.org/api/query"
SS = "https://api.semanticscholar.org/graph/v1/paper/search"
SS_SEARCH2 = "https://api.semanticscholar.org/graph/v1/paper/search/bulk"


def http_get_json(url, timeout=40):
    req = urllib.request.Request(url, headers=UA)
    with urllib.request.urlopen(req, timeout=timeout, context=CTX) as r:
        return json.loads(r.read().decode("utf-8"))


def http_get_text(url, timeout=40):
    req = urllib.request.Request(url, headers=UA)
    with urllib.request.urlopen(req, timeout=timeout, context=CTX) as r:
        return r.read().decode("utf-8")


def arxiv_search(q, max_results=10, sort="submittedDate"):
    params = {
        "search_query": q,
        "start": 0,
        "max_results": max_results,
        "sortBy": sort,
        "sortOrder": "descending",
    }
    url = ARXIV + "?" + urllib.parse.urlencode(params)
    raw = http_get_text(url)
    ns = {"a": "http://www.w3.org/2005/Atom"}
    root = ET.fromstring(raw)
    items = []
    for e in root.findall("a:entry", ns):
        items.append({
            "title": (e.findtext("a:title", default="", namespaces=ns) or "").strip().replace("\n", " "),
            "summary": (e.findtext("a:summary", default="", namespaces=ns) or "").strip().replace("\n", " ")[:500],
            "published": e.findtext("a:published", default="", namespaces=ns),
            "id": e.findtext("a:id", default="", namespaces=ns),
            "authors": [a.findtext("a:name", default="", namespaces=ns) for a in e.findall("a:author", ns)],
        })
    return items


def ss_search(q, limit=10, year="2022-"):
    params = {
        "query": q,
        "limit": limit,
        "fields": "title,year,authors,abstract,citationCount,externalIds,venue,publicationVenue",
    }
    if year:
        params["year"] = year
    url = SS + "?" + urllib.parse.urlencode(params)
    data = http_get_json(url)
    out = []
    for p in data.get("data", []) or []:
        authors = ", ".join(a.get("name", "") for a in (p.get("authors") or [])[:8])
        out.append({
            "title": p.get("title"),
            "year": p.get("year"),
            "authors": authors,
            "abstract": (p.get("abstract") or "")[:400],
            "citations": p.get("citationCount"),
            "venue": (p.get("publicationVenue") or {}).get("name") or p.get("venue"),
            "doi": (p.get("externalIds") or {}).get("DOI"),
            "arxiv": (p.get("externalIds") or {}).get("ArXiv"),
        })
    return out


results = {"arxiv": {}, "semantic_scholar": {}}

arxiv_queries = {
    "multi_goal_nav": 'all:"multi-goal" AND all:"path planning" AND all:robot',
    "multi_goal_tsp": 'all:"multi-goal" AND all:"traveling salesman"',
    "path_planning_survey": 'all:"path planning" AND all:"mobile robot" AND (all:survey OR all:review)',
    "astar_review": 'abs:"path planning" AND abs:"A*" AND (abs:review OR abs:survey)',
    "jps": 'all:"jump point search"',
    "open_tsp": 'all:"open traveling salesman"',
    "tsp_obstacles": 'all:"traveling salesman" AND all:obstacle',
    "aco_tsp": 'all:"ant colony" AND all:"traveling salesman"',
    "path_smoothing": 'all:"path smoothing" AND all:robot',
    "robot_navigation_survey": 'all:"mobile robot navigation" AND (all:survey OR all:review)',
}

for name, q in arxiv_queries.items():
    print("ARXIV", name, flush=True)
    try:
        results["arxiv"][name] = arxiv_search(q, sort="submittedDate")
    except Exception as e:
        try:
            results["arxiv"][name] = arxiv_search(q, sort="relevance")
        except Exception as e2:
            results["arxiv"][name] = {"error": f"{e} | {e2}"}
    time.sleep(3.2)

ss_queries = {
    "survey_path_planning": "survey path planning algorithms mobile robots",
    "survey_multi_goal": "multi-goal path planning mobile robot",
    "survey_tsp_meta": "traveling salesman problem survey metaheuristic",
    "open_tsp": "open traveling salesman problem",
    "tsp_obstacles_robot": "traveling salesman problem with obstacles robot",
    "jps_astar_grid": "jump point search grid pathfinding",
    "aco_tsp_recent": "ant colony optimization traveling salesman",
    "multi_goal_nav": "multi-goal navigation mobile robot TSP",
    "path_smoothing_robot": "path smoothing mobile robot",
    "autonomous_nav_review": "autonomous mobile robot navigation review",
    "tsp_path_planning": "traveling salesman path planning robot goals",
}

for name, q in ss_queries.items():
    print("SS", name, flush=True)
    try:
        results["semantic_scholar"][name] = ss_search(q, year="2021-")
    except Exception as e:
        results["semantic_scholar"][name] = {"error": str(e)}
    time.sleep(2.5)

OUT.write_text(json.dumps(results, ensure_ascii=False, indent=2), encoding="utf-8")
print("WROTE", OUT)
