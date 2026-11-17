import re, io, sys

p = r'D:\Claude-File\shiyan1\paper\paper_CN_simple\paper_CN.tex'
s = io.open(p, encoding='utf-8').read()

# split body
body = s[s.index(r'\begin{document}'):]
# find section starts
pat = re.compile(r'\\(section|subsection)\*?\{([^}]*)\}')
marks = [(m.start(), m.group(1), m.group(2)) for m in pat.finditer(body)]
marks = [(0, 'preamble', 'ABSTRACT+INTRO')] + marks
marks = [m for m in marks if 'thebibliography' not in body[m[0]:m[0]+50]]

def cjk(t):
    return len(re.findall(r'[\u4e00-\u9fff]', t))

def total(t):
    # strip commands roughly
    t = re.sub(r'\\[a-zA-Z@]+\*?(\[[^\]]*\])?', ' ', t)
    t = re.sub(r'[{}$\\_^&~%]', ' ', t)
    return len(re.findall(r'[A-Za-z0-9]+|[\u4e00-\u9fff]', t))

print(f"{'sec':<4} {'cjk':>6} {'words':>6} {'fig':>4} {'tab':>4} {'eq':>4} {'alg':>4}  title")
rows = []
for i, (pos, kind, title) in enumerate(marks):
    end = marks[i+1][0] if i+1 < len(marks) else len(body)
    seg = body[pos:end]
    if i == 0:
        seg = body[:marks[1][0]] if len(marks) > 1 else body
    rows.append((cjk(seg), total(seg), len(re.findall(r'\\begin\{figure', seg)),
                 len(re.findall(r'\\begin\{table', seg)),
                 len(re.findall(r'\\begin\{(equation|align)\}', seg)),
                 len(re.findall(r'\\begin\{algorithm\}', seg)),
                 kind, title))
tc = sum(r[0] for r in rows)
for r in rows:
    print(f"{r[0]/tc*100:5.1f}% {r[0]:6d} {r[1]:6d} {r[2]:4d} {r[3]:4d} {r[4]:4d} {r[5]:4d}  {r[6]}: {r[7][:40]}")
print('total cjk', tc)
print('figs', len(re.findall(r'\\begin\{figure', body)), 'tabs', len(re.findall(r'\\begin\{table', body)),
      'eqs', len(re.findall(r'\\begin\{(equation|align)\}', body)), 'algs', len(re.findall(r'\\begin\{algorithm\}', body)))
print('refs', len(re.findall(r'\\bibitem', body)))
