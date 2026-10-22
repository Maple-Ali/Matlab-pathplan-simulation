# -*- coding: utf-8 -*-
import re
from pathlib import Path
text = Path(r"D:\Claude-File\shiyan1\paper\paper_CN_full\paper_CN.tex").read_text(encoding="utf-8")
start = text.find("\\section{引言}")
end = text.find("\\section{问题形式化}")
print(text[start:end])
print("\n===== BIB KEYS =====")
print(re.findall(r"\\bibitem\{([^}]+)\}", text))
