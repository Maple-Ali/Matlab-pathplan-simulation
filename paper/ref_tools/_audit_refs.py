# -*- coding: utf-8 -*-
"""Audit references.bib against Crossref and collect citation contexts from the .tex."""
import io
import json
import re
import time
import urllib.parse
import urllib.request
import difflib

BIB = r"D:\Claude-File\shiyan1\paper\paper_CN_simple\references.bib"
TEX = r"D:\Claude-File\shiyan1\paper\paper_CN_simple\paper_CN.tex"
OUT = r"D:\Claude-File\shiyan1\paper\ref_tools\_ref_audit.json"

ENTRY_RE = re.compile(r"@(\w+)\{([^,]+),(.*?)\n\}", re.S)
FIELD_RE = re.compile(r"(\w+)\s*=\s*\{(.*?)\}\s*,?\s*(?=\n\s*\w+\s*=|$)", re.S)


def parse_bib(text):
    entries = []
    for m in ENTRY_RE.finditer(text):
        kind, key, body = m.group(1), m.group(2).strip(), m.group(3)
        fields = {}
        for fm in re.finditer(r"(\w+)\s*=\s*\{(.*?)\}\s*,?\s*$", body, re.M):
            fields[fm.group(1).lower()] = re.sub(r"\s+", " ", fm.group(2)).strip()
        entries.append({"key": key, "type": kind, **fields})
    return entries


def norm(s):
    return re.sub(r"[^a-z0-9]", "", (s or "").lower())


def crossref(doi):
    url = "https://api.crossref.org/works/" + urllib.parse.quote(doi)
    req = urllib.request.Request(url, headers={"User-Agent": "ref-audit/1.0 (mailto:audit@example.com)"})
    with urllib.request.urlopen(req, timeout=25) as r:
        return json.load(r)["message"]


def compare(e, c):
    issues = []
    ctitle = (c.get("title") or [""])[0]
    if norm(ctitle) and difflib.SequenceMatcher(None, norm(e.get("title", "")), norm(ctitle)).ratio() < 0.90:
        issues.append("TITLE: bib=%r crossref=%r" % (e.get("title"), ctitle))
    cvol = str(c.get("volume") or "")
    if e.get("volume") and cvol and e["volume"] != cvol:
        issues.append("VOLUME: bib=%s crossref=%s" % (e["volume"], cvol))
    ciss = str(c.get("issue") or "")
    if e.get("number") and ciss and e["number"] != ciss:
        issues.append("ISSUE: bib=%s crossref=%s" % (e["number"], ciss))
    cpage = (c.get("page") or "").replace("-", "")
    bpage = re.sub(r"[^0-9]", "", e.get("pages", ""))
    if cpage and bpage and cpage != bpage:
        issues.append("PAGES: bib=%s crossref=%s" % (e.get("pages"), c.get("page")))
    cyear = None
    for fld in ("published-print", "published-online", "issued"):
        parts = ((c.get(fld) or {}).get("date-parts") or [[None]])[0]
        if parts and parts[0]:
            cyear = str(parts[0])
            break
    if e.get("year") and cyear and e["year"] != cyear:
        issues.append("YEAR: bib=%s crossref=%s" % (e["year"], cyear))
    cauth = (c.get("author") or [{}])[0].get("family", "")
    bauth = e.get("author", "").split(",")[0].strip()
    if cauth and bauth and norm(cauth) != norm(bauth) and norm(bauth) not in norm(cauth):
        if difflib.SequenceMatcher(None, norm(cauth), norm(bauth)).ratio() < 0.85:
            issues.append("AUTHOR1: bib=%s crossref=%s" % (bauth, cauth))
    ccont = (c.get("container-title") or [""])[0]
    return issues, ctitle, ccont


def contexts(tex):
    """key -> list of surrounding sentence fragments."""
    out = {}
    for m in re.finditer(r"\\cite\{([^}]*)\}", tex):
        start, end = m.start(), m.end()
        left = tex.rfind("。", 0, start)
        left2 = tex.rfind("\n", 0, start)
        l = max(left, left2) + 1
        right = tex.find("。", end)
        r = right + 1 if right != -1 else min(len(tex), end + 120)
        frag = re.sub(r"\s+", " ", tex[l:r]).strip()
        for key in [k.strip() for k in m.group(1).split(",")]:
            out.setdefault(key, []).append({"pos": start, "text": frag[:260]})
    return out


def main():
    bib = io.open(BIB, encoding="utf-8", newline="").read()
    tex = io.open(TEX, encoding="utf-8", newline="").read()
    entries = parse_bib(bib)
    ctx = contexts(tex)

    results = []
    for e in entries:
        rec = {"key": e["key"], "type": e["type"], "year": e.get("year", ""),
               "title": e.get("title", ""), "journal": e.get("journal") or e.get("booktitle", ""),
               "doi": e.get("doi", ""), "authors": e.get("author", ""),
               "volume": e.get("volume", ""), "number": e.get("number", ""), "pages": e.get("pages", ""),
               "cited": len(ctx.get(e["key"], [])), "contexts": ctx.get(e["key"], [])}
        if e.get("doi"):
            try:
                c = crossref(e["doi"])
                issues, ctitle, ccont = compare(e, c)
                rec["crossref_title"] = ctitle
                rec["crossref_container"] = ccont
                rec["issues"] = issues
            except Exception as exc:
                rec["issues"] = ["CROSSREF_FAIL: %s" % exc]
            time.sleep(0.3)
        else:
            rec["issues"] = ["NO_DOI"]
        results.append(rec)

    io.open(OUT, "w", encoding="utf-8").write(json.dumps(results, ensure_ascii=False, indent=1))
    bad = [r for r in results if r["issues"] and r["issues"] != ["NO_DOI"]]
    print("entries=%d  with-issues=%d  no-doi=%d" % (len(results), len(bad), sum(1 for r in results if r["issues"] == ["NO_DOI"])))
    for r in results:
        print(r["key"], r["year"], "|", ",".join(r["issues"])[:110])


if __name__ == "__main__":
    main()
