import re, io

p = r'D:\Claude-File\shiyan1\paper\paper_CN_simple\paper_CN.tex'
s = io.open(p, encoding='utf-8').read()
body = s[s.index(r'\begin{document}'):]
body = body[:body.index(r'\begin{thebibliography}')]

pat = re.compile(r'\\(section|subsection|subsubsection)\*?\{([^}]*)\}')
marks = [(m.start(), m.group(1), m.group(2)) for m in pat.finditer(body)]

def cjk(t):
    return len(re.findall(r'[\u4e00-\u9fff]', t))

rows = []
for i, (pos, kind, title) in enumerate(marks):
    end = marks[i+1][0] if i+1 < len(marks) else len(body)
    seg = body[pos:end]
    rows.append((cjk(seg), len(re.findall(r'\\begin\{figure\*?\}', seg)), len(re.findall(r'\\begin\{table\}', seg)),
                 len(re.findall(r'\\begin\{algorithm\}', seg)), kind, title))

tc = sum(r[0] for r in rows)
for r in rows:
    print(f"{r[0]:5d} {r[0]/tc*100:5.1f}%  fig={r[1]} tab={r[2]} alg={r[3]}  {r[4][:12]:<12} {r[5]}")
print('TOTAL', tc)
