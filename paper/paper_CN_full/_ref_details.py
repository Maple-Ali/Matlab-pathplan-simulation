# -*- coding: utf-8 -*-
"""Get complete author lists for top recommended papers via OpenAlex by DOI/title."""
import json
import ssl
import urllib.parse
import urllib.request

CTX = ssl._create_unverified_context()
UA = {"User-Agent": "research-lit/1.0"}

dois = [
    "10.3390/vehicles3030027",
    "10.3390/robotics12040093",
    "10.3390/s24113573",
    "10.48084/etasr.13885",
    "10.1007/s10514-022-10039-8",
    "10.1016/j.ejor.2023.07.022",
    "10.3390/a19040278",
    "10.3390/mca31050180",
    "10.1016/j.robot.2024.104655",
    "10.1016/j.asoc.2021.107439",
    "10.1016/j.engappai.2022.105139",
    "10.1016/j.swevo.2022.101056",
    "10.1613/jair.1.12255",
    "10.1609/socs.v3i1.18254",
    "10.1609/aaai.v37i10.26458",
    "10.1109/lra.2023.3311207",
    "10.1109/lra.2024.3524912",
    "10.3390/s25020446",
    "10.1109/TSMC.2020.3048984",
    "10.1109/lra.2021.3091695",
    "10.1016/j.ins.2011.05.019",
    "10.1016/j.eswa.2024.125185",
    "10.1109/iros.2014.6942974",
    "10.1016/j.cor.2018.07.023",
    "10.1287/ijoc.2013.0574",
    "10.1016/j.ejor.2023.04.010",
    "10.1007/s11370-025-00654-6",
    "10.1016/j.autcon.2017.04.013",
    "10.1109/access.2021.3058190",
    "10.1007/s10462-025-11267-x",
    "10.1007/s11704-024-40490-y",
    "10.1016/j.procs.2018.07.018",
    "10.1155/2016/7426913",
    "10.3390/math11214439",
    "10.1016/j.matcom.2023.12.003",
]

def get_work(doi):
    url = "https://api.openalex.org/works/https://doi.org/" + urllib.parse.quote(doi, safe="")
    req = urllib.request.Request(url, headers=UA)
    with urllib.request.urlopen(req, timeout=40, context=CTX) as r:
        return json.loads(r.read().decode("utf-8"))

out = []
for doi in dois:
    try:
        w = get_work(doi)
        authors = [a["author"]["display_name"] for a in (w.get("authorships") or [])]
        bib = w.get("biblio") or {}
        src = ((w.get("primary_location") or {}) or {}).get("source") or {}
        out.append({
            "doi": doi,
            "title": w.get("title"),
            "year": w.get("publication_year"),
            "authors": authors,
            "venue": src.get("display_name"),
            "volume": bib.get("volume"),
            "issue": bib.get("issue"),
            "first_page": bib.get("first_page"),
            "last_page": bib.get("last_page"),
            "cited": w.get("cited_by_count"),
        })
        print("OK", doi, w.get("title")[:60] if w.get("title") else "")
    except Exception as e:
        print("ERR", doi, e)
        out.append({"doi": doi, "error": str(e)})

path = r"D:\Claude-File\shiyan1\paper\paper_CN_full\_ref_details.json"
with open(path, "w", encoding="utf-8") as f:
    json.dump(out, f, ensure_ascii=False, indent=2)
print("WROTE", path, "n=", len(out))
