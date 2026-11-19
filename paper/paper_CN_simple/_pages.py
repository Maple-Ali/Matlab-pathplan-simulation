import subprocess, sys
pdf = r'D:\Claude-File\shiyan1\paper\paper_CN_simple\paper_CN.pdf'
out = subprocess.run([r'D:\texlive\2026\bin\windows\pdftotext.exe', '-layout', pdf, '-'], capture_output=True)
txt = out.stdout.decode('utf-8', 'ignore')
pages = txt.split('\f')
for i, p in enumerate(pages, 1):
    for key in ('代价矩阵构建开销', '实验环境示意图', '参数敏感性分析'):
        if key in p:
            print(f'page {i}: {key}')
print('total pages(text):', len(pages))
