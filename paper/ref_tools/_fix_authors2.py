# -*- coding: utf-8 -*-
import json, ssl, time, urllib.parse, urllib.request
from pathlib import Path

CTX = ssl._create_unverified_context()
UA = {"User-Agent": "research-lit/1.0"}

queries = {
    "ref8": "comprehensive review improved A* path planning algorithms hybrid integrations Automation 2025",
    "ref15": "jump point Theta* adaptive weight optimization mobile robot path planning",
    "ref17": "Dijkstra piecewise cubic Bezier optimization path planning safety smoothness",
    "ref18": "Adaptive A* NSGA-II multi-goal navigation mobile robots",
    "ref8b": "improved A* path planning hybrid integrations review",
}

def search(q, n=5):
    params = {"search": q, "per-page": n}
    url = "https://api.openalex.org/works?" + urllib.parse.urlencode(params)
    req = urllib.request.Request(url, headers=UA)
    with urllib.request.urlopen(req, timeout=40, context=CTX) as r:
        data = json.loads(r.read().decode("utf-8"))
    out = []
    for w in data.get("results", [])[:n]:
        authors = [a["author"]["display_name"] for a in (w.get("authorships") or [])]
        bib = w.get("biblio") or {}
        src = ((w.get("primary_location") or {}) or {}).get("source") or {}
        out.append({
            "title": w.get("title"), "year": w.get("publication_year"),
            "authors": authors, "venue": src.get("display_name"),
            "doi": w.get("doi"), "volume": bib.get("volume"), "issue": bib.get("issue"),
            "first_page": bib.get("first_page"), "last_page": bib.get("last_page"),
        })
    return out

# Also Crossref
def crossref(q, n=5):
    params = {"query.bibliographic": q, "rows": n, "select": "title,author,issued,container-title,volume,issue,page,DOI"}
    url = "https://api.crossref.org/works?" + urllib.parse.urlencode(params)
    req = urllib.request.Request(url, headers={"User-Agent": "research-lit/1.0 (mailto:research@example.com)"})
    with urllib.request.urlopen(req, timeout=40, context=CTX) as r:
        data = json.loads(r.read().decode("utf-8"))
    out = []
    for it in data.get("message", {}).get("items", [])[:n]:
        authors = []
        for a in it.get("author") or []:
            given = a.get("given", "")
            family = a.get("family", "")
            authors.append(f"{given} {family}".strip())
        year = ((it.get("issued") or {}).get("date-parts") or [[None]])[0][0]
        out.append({
            "title": (it.get("title") or [""])[0],
            "year": year,
            "authors": authors,
            "venue": (it.get("container-title") or [""])[0],
            "doi": it.get("DOI"),
            "volume": it.get("volume"),
            "issue": it.get("issue"),
            "first_page": it.get("page"),
        })
    return out

results = {}
for k, q in queries.items():
    print("Q", k, flush=True)
    try:
        results[k] = {"openalex": search(q)}
    except Exception as e:
        results[k] = {"openalex_error": str(e)}
    try:
        results[k]["crossref"] = crossref(q)
    except Exception as e:
        results[k]["crossref_error"] = str(e)
    time.sleep(0.4)

Path(r"D:\Claude-File\shiyan1\paper\paper_CN_full\_ref_authors_fix2.json").write_text(
    json.dumps(results, ensure_ascii=False, indent=2), encoding="utf-8"
)
print(json.dumps(results, ensure_ascii=False, indent=2)[:8000])
