import re, io
p = r'D:\Claude-File\shiyan1\paper\paper_CN_simple\paper_CN.tex'
s = io.open(p, encoding='utf-8').read()
body = s[s.index(r'\begin{document}'):]
body = body[:body.index(r'\begin{thebibliography}')]
order = []
for m in re.finditer(r'\\cite\{([^}]*)\}', body):
    for k in m.group(1).split(','):
        k = k.strip()
        if k and k not in order:
            order.append(k)
bib = re.findall(r'\\bibitem\{([^}]*)\}', s)
num = {k: i+1 for i, k in enumerate(bib)}
seq = [num[k] for k in order if k in num]
print('bib count', len(bib), 'cited', len(order))
bad = [(a, b) for a, b in zip(seq, seq[1:]) if b < a]
print('non-monotonic citation jumps:', len(bad))
print('sequence:', seq)
print('uncited:', [k for k in bib if k not in order])
