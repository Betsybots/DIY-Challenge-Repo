#!/usr/bin/env python3
"""
generate_nav_design_light.py
Generates docs/Navigation_Design_Guide_Light.html + .pdf  (bright theme)
All content is identical to the dark version — only CSS differs.
"""

import sys
import types
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent

# ── Load the dark generator as a module without executing its write block ──
import importlib.util, re

src = (REPO_ROOT / "docs" / "generate_nav_design.py").read_text()

# Patch output paths so importing doesn't overwrite dark files
src = src.replace(
    'OUT_HTML  = REPO_ROOT / "docs" / "Navigation_Design_Guide.html"',
    'OUT_HTML  = REPO_ROOT / "docs" / "Navigation_Design_Guide_Light.html"',
).replace(
    'OUT_PDF   = REPO_ROOT / "docs" / "Navigation_Design_Guide.pdf"',
    'OUT_PDF   = REPO_ROOT / "docs" / "Navigation_Design_Guide_Light.pdf"',
)

# Stop before the write block — replace it with a pass
src = re.sub(
    r'# ─+\n# Write HTML.*',
    '# (write block removed — handled below)',
    src,
    flags=re.DOTALL,
)

exec(compile(src, "generate_nav_design.py", "exec"), globals())

# ── Override CSS with light theme ──
CSS = CSS.replace(
    """  --dark:    #0f172a;
  --card:    #1e293b;
  --border:  #334155;
  --muted:   #64748b;
  --text:    #e2e8f0;
  --subtext: #94a3b8;""",
    """  --dark:    #f8fafc;
  --card:    #ffffff;
  --border:  #cbd5e1;
  --muted:   #64748b;
  --text:    #0f172a;
  --subtext: #334155;"""
).replace(
    "background: var(--dark); color: var(--text);",
    "background: var(--dark); color: var(--text);"
).replace(
    "background: linear-gradient(160deg, #0f172a 0%, #0c1a2e 50%, #130f1a 100%);",
    "background: linear-gradient(160deg, #e0f2fe 0%, #f0fdf4 50%, #faf5ff 100%);"
).replace(
    "background: rgba(255,255,255,0.04);",
    "background: rgba(0,0,0,0.03);"
).replace(
    "background: rgba(255,255,255,0.02);",
    "background: rgba(0,0,0,0.02);"
).replace(
    "background: rgba(255,255,255,0.06);",
    "background: rgba(0,0,0,0.05);"
).replace(
    ".cover h1 { font-size: 34px; font-weight: 700; color: #fff;",
    ".cover h1 { font-size: 34px; font-weight: 700; color: #0f172a;"
).replace(
    ".toc h2 { font-size: 20px; font-weight: 700; color: #fff;",
    ".toc h2 { font-size: 20px; font-weight: 700; color: #0f172a;"
).replace(
    "background: #0a0e1a; border: 1px solid var(--border); border-left: 3px solid var(--cyan); border-radius: 6px; padding: 12px 16px; font-family: 'JetBrains Mono', monospace; font-size: 9.5px; color: #a0c4ff;",
    "background: #f1f5f9; border: 1px solid var(--border); border-left: 3px solid var(--cyan); border-radius: 6px; padding: 12px 16px; font-family: 'JetBrains Mono', monospace; font-size: 9.5px; color: #1e40af;"
).replace(
    "td { padding: 7px 10px; color: var(--subtext); border-bottom: 1px solid rgba(255,255,255,0.04);",
    "td { padding: 7px 10px; color: var(--subtext); border-bottom: 1px solid rgba(0,0,0,0.06);"
)

# Rebuild HTML with patched CSS
HTML = HTML.replace(
    # find the <style> block and replace it
    HTML[HTML.find("<style>"):HTML.find("</style>")+8],
    f"<style>\n{CSS}\n</style>"
)

# ── Write light HTML ──
OUT_HTML.write_text(HTML, encoding="utf-8")
print(f"[gen] HTML written → {OUT_HTML}")

# ── Write light PDF ──
try:
    import weasyprint
    wp = weasyprint.HTML(filename=str(OUT_HTML))
    wp.write_pdf(str(OUT_PDF))
    print(f"[gen] PDF written  → {OUT_PDF}")
except Exception as e:
    print(f"[gen] PDF generation failed: {e}")
    print(f"[gen] HTML is still available at {OUT_HTML}")
