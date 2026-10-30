# -*- coding: utf-8 -*-
"""Search recent related literature for the multi-goal traversal paper."""
import json
import time
import urllib.parse
import urllib.request
from pathlib import Path

OUT = Path(r"D:\Claude-File\shiyan1\paper\paper_CN_full\_lit_search_results.json")

ARXIV = "https://export.arxiv.org/api/query"
SS = "https://api.semanticscholar.org/graph/v1/paper/search"

queries = {
    "multi_goal_nav": 'all:"multi-goal" AND all:"path planning" AND all:robot',
    "multi_goal_survey": 'all:"multi-goal" AND (all:survey OR all:review) AND all:robot',
    "path_planning_survey": 'all:"path planning" AND all:"mobile robot" AND (all:survey OR all:review)',
    "astar_review": 'all:"A*" AND all:"path planning" AND (all:review OR all:survey)',
    "jps": 'all:"jump point search" AND all:path',
    "open_tsp": 'all:"open traveling salesman" OR all:"open travelling salesman"',
    "tsp_obstacles": 'all:"traveling salesman" AND all:obstacle AND all:path',
    "aco_tsp": 'all:"ant colony" AND all:"traveling salesman"',
    "tsp_metaheuristic_survey": 'all:"traveling salesman" AND (all:survey OR all:review) AND all:metaheuristic',
    "path_smoothing": 'all:"path smoothing" AND all:robot AND all:grid',
    "multi_goal_tsp_robot": 'all:"traveling salesman" AND all:robot AND all:"goal"',
}


def get_json(url, headers=None):
    req = urllib.request.Request(url, headers=headers or {"User-Agent": "Mozilla/5.0 research"})
    with urllib.request.urlopen(req, timeout=30) as r:
        return json.loads(r.read().decode("utf-8"))


def arxiv_search(q, max_results=8):
    params = {
        "search_query": q,
        "start": 0,
        "max_results": max_results,
        "sortBy": "submittedDate",
        "sortOrder": "descending",
    }
    url = ARXIV + "?" + urllib.parse.urlencode(params)
    req = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0 research"})
    with urllib.request.urlopen(req, timeout=30) as r:
        raw = r.read().decode("utf-8")
    # minimal atom parse
    import xml.etree.ElementTree as ET
    ns = {"a": "http://www.w3.org/2005/Atom"}
    root = ET.fromstring(raw)
    items = []
    for e in root.findall("a:entry", ns):
        items.append({
            "title": (e.findtext("a:title", default="", namespaces=ns) or "").strip().replace("\n", " "),
            "summary": (e.findtext("a:summary", default="", namespaces=ns) or "").strip().replace("\n", " ")[:400],
            "published": e.findtext("a:published", default="", namespaces=ns),
            "id": e.findtext("a:id", default="", namespaces=ns),
            "authors": [a.findtext("a:name", default="", namespaces=ns) for a in e.findall("a:author", ns)],
        })
    return items


def ss_search(q, limit=8):
    params = {
        "query": q,
        "limit": limit,
        "fields": "title,year,authors,abstract,citationCount,externalIds,venue,publicationVenue,year",
        "year": "2022-",
    }
    # year filter as optional - if fails retry without
    url = SS + "?" + urllib.parse.urlencode(params)
    try:
        data = get_json(url)
    except Exception as ex:
        params.pop("year", None)
        url = SS + "?" + urllib.parse.urlencode(params)
        data = get_json(url)
    out = []
    for p in data.get("data", []) or []:
        authors = ", ".join(a.get("name", "") for a in (p.get("authors") or [])[:6])
        out.append({
            "title": p.get("title"),
            "year": p.get("year"),
            "authors": authors,
            "abstract": (p.get("abstract") or "")[:350],
            "citations": p.get("citationCount"),
            "venue": (p.get("publicationVenue") or {}).get("name") or p.get("venue"),
            "doi": (p.get("externalIds") or {}).get("DOI"),
            "arxiv": (p.get("externalIds") or {}).get("ArXiv"),
        })
    return out


results = {"arxiv": {}, "semantic_scholar": {}}

for name, q in queries.items():
    print("ARXIV", name, flush=True)
    try:
        results["arxiv"][name] = arxiv_search(q)
    except Exception as e:
        results["arxiv"][name] = {"error": str(e)}
    time.sleep(3.5)

ss_queries = {
    "survey_path_planning": "survey path planning mobile robots",
    "survey_multi_goal": "multi-goal path planning mobile robot",
    "survey_tsp": "traveling salesman problem survey metaheuristic",
    "open_tsp": "open traveling salesman problem",
    "tsp_obstacles_robot": "traveling salesman problem obstacles robot path planning",
    "jps_astar": "jump point search A* grid path planning",
    "aco_recent": "ant colony optimization traveling salesman 2024",
    "multi_goal_nav": "multi-goal navigation mobile robot 2024",
    "path_smoothing_robot": "path smoothing mobile robot grid",
    "autonomous_navigation_review": "autonomous mobile robot navigation review 2024",
}

for name, q in ss_queries.items():
    print("SS", name, flush=True)
    try:
        results["semantic_scholar"][name] = ss_search(q)
    except Exception as e:
        results["semantic_scholar"][name] = {"error": str(e)}
    time.sleep(1.2)

OUT.write_text(json.dumps(results, ensure_ascii=False, indent=2), encoding="utf-8")
print("WROTE", OUT)
