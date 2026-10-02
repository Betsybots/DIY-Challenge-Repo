import re, sys, markdown, weasyprint
CSS = """
@page { size: A4; margin: 16mm 14mm 16mm 14mm;
        @bottom-center { content: counter(page) " / " counter(pages); font-size: 8pt; color: #888; } }
body { font-family: 'DejaVu Sans', sans-serif; font-size: 9.6pt; line-height: 1.38; color: #1d1d1d; }
h1 { font-size: 19pt; color: #12355b; border-bottom: 2.5px solid #12355b; padding-bottom: 4px; margin-top: 0; }
h2 { font-size: 13.5pt; color: #12355b; margin-top: 18px; border-bottom: 1px solid #c9d3df; padding-bottom: 2px; page-break-after: avoid; }
h3 { font-size: 11pt; color: #23527c; margin-top: 12px; page-break-after: avoid; }
code { font-family: 'DejaVu Sans Mono', monospace; font-size: 8.4pt; background: #f1f3f6; padding: 0 2px; border-radius: 2px; }
pre { background: #f5f7fa; border: 1px solid #d8dee6; border-left: 3px solid #23527c; padding: 6px 8px; font-size: 7.6pt;
      line-height: 1.3; white-space: pre-wrap; word-wrap: break-word; page-break-inside: avoid; }
pre code { background: none; padding: 0; font-size: 7.6pt; }
table { border-collapse: collapse; width: 100%; table-layout: auto; margin: 6px 0 10px 0; font-size: 8.3pt; page-break-inside: auto; }
th { background: #12355b; color: white; text-align: left; padding: 4px 5px; }
td { border-bottom: 1px solid #dde3ea; padding: 3px 5px; vertical-align: top; overflow-wrap: anywhere; word-break: break-word; }
td code, th code { word-break: break-all; white-space: normal; }
tr:nth-child(even) td { background: #f7f9fb; }
tr { page-break-inside: avoid; }
hr { border: none; border-top: 1px solid #c9d3df; margin: 10px 0; }
.subtitle { color: #555; font-size: 10.5pt; margin-top: -6px; margin-bottom: 10px; }
ul, ol { margin-top: 3px; } li { margin-bottom: 2px; }
img { max-width: 100%; margin: 6px 0; } em { color: #555; }
blockquote { border-left: 3px solid #e0a800; background: #fff8e1; margin: 6px 0; padding: 4px 10px; }
"""
for src in sys.argv[1:]:
    text = open(src).read()
    m = re.match(r'^---\n(.*?)\n---\n', text, re.S)
    sub = ''
    if m:
        meta = dict(re.findall(r'^(\w+):\s*"(.*)"\s*$', m.group(1), re.M))
        text = text[m.end():]
        sub = meta.get('subtitle', '')
    html = markdown.markdown(text, extensions=['tables', 'fenced_code', 'sane_lists'])
    if sub:
        html = html.replace('</h1>', f'</h1><div class="subtitle">{sub}</div>', 1)
    out = src.rsplit('.', 1)[0] + '.pdf'
    weasyprint.HTML(base_url=__import__('os').path.dirname(__import__('os').path.abspath(src)), string=f'<html><head><meta charset="utf-8"><style>{CSS}</style></head><body>{html}</body></html>').write_pdf(out)
    print('wrote', out)
