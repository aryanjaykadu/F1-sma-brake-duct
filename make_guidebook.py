"""Build report/Project_Guidebook.pdf from docs/GUIDEBOOK.md (headings, paragraphs, lists, tables, figures)."""
import os
import re
import tempfile

from PIL import Image as PImage
from reportlab.lib import colors
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.units import mm
from reportlab.platypus import (HRFlowable, Image, KeepTogether, ListFlowable, ListItem, PageBreak, Paragraph,
                                SimpleDocTemplate, Spacer, Table, TableStyle)

ROOT = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(ROOT, "docs", "GUIDEBOOK.md")
OUT = os.path.join(ROOT, "report", "Project_Guidebook.pdf")
TMP = tempfile.mkdtemp()

from reportlab.pdfbase import pdfmetrics  # noqa: E402
from reportlab.pdfbase.ttfonts import TTFont  # noqa: E402

FONTS = os.path.join(os.environ.get("WINDIR", r"C:\Windows"), "Fonts")
for name, file in (("Body", "arial.ttf"), ("Body-Bold", "arialbd.ttf"), ("Body-Italic", "ariali.ttf"),
                   ("Mono", "consola.ttf")):
    pdfmetrics.registerFont(TTFont(name, os.path.join(FONTS, file)))   # full Unicode (ṁ, ⁴, ω, ξ)
pdfmetrics.registerFontFamily("Body", normal="Body", bold="Body-Bold", italic="Body-Italic", boldItalic="Body-Bold")
INK, ACCENT, GREY, RULE = (colors.HexColor(c) for c in ("#1d1d1f", "#c2410c", "#6b6b70", "#d6d6db"))
FONT, BOLD, MONO = "Body", "Body-Bold", "Mono"
ST = {
    "title": ParagraphStyle("title", fontName=BOLD, fontSize=26, leading=31, textColor=INK, spaceAfter=6),
    "h2": ParagraphStyle("h2", fontName=BOLD, fontSize=15, leading=19, textColor=ACCENT, spaceBefore=10, spaceAfter=5),
    "h3": ParagraphStyle("h3", fontName=BOLD, fontSize=11.5, leading=14.5, textColor=INK, spaceBefore=8, spaceAfter=3),
    "body": ParagraphStyle("body", fontName=FONT, fontSize=9.6, leading=13.4, textColor=INK, spaceAfter=5),
    "cell": ParagraphStyle("cell", fontName=FONT, fontSize=8.4, leading=10.8, textColor=INK),
    "cellb": ParagraphStyle("cellb", fontName=BOLD, fontSize=8.4, leading=10.8, textColor=INK),
    "cap": ParagraphStyle("cap", fontName=FONT, fontSize=8, leading=10, textColor=GREY, alignment=1, spaceAfter=8),
    "eq": ParagraphStyle("eq", fontName=MONO, fontSize=9, leading=12.5, textColor=INK, leftIndent=8 * mm,
                         spaceBefore=2, spaceAfter=6, backColor=colors.HexColor("#f4f4f6"), borderPadding=4),
}


def inline(text):
    text = text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
    text = re.sub(r"\*\*(.+?)\*\*", r"<b>\1</b>", text)
    text = re.sub(r"(?<!\*)\*(?!\s)(.+?)(?<!\s)\*(?!\*)", r"<i>\1</i>", text)
    text = re.sub(r"`(.+?)`", r'<font face="Mono" size="8.8">\1</font>', text)
    return text


def figure(rel, caption, width=150 * mm, cell=False):
    path = os.path.normpath(os.path.join(ROOT, "docs", rel))
    im = PImage.open(path).convert("RGB")
    box = im.convert("L").point(lambda v: 255 if v < 200 else 0).getbbox() or (0, 0, *im.size)
    pad = 14
    crop = im.crop((max(0, box[0] - pad), max(0, box[1] - pad), min(im.width, box[2] + pad), min(im.height, box[3] + pad)))
    out = os.path.join(TMP, f"fig_{abs(hash(rel))}.png")
    crop.save(out)
    h = width * crop.height / crop.width
    if h > 105 * mm:                       # keep tall figures from eating a page
        width, h = width * 105 * mm / h, 105 * mm
    parts = [Image(out, width=width, height=h), Paragraph(inline(caption), ST["cap"])]
    if cell:                               # inside a table cell: plain flowables only
        return parts
    return KeepTogether([Spacer(1, 2 * mm)] + parts)


def table(lines):
    rows = [[c.strip() for c in ln.strip().strip("|").split("|")] for ln in lines]
    rows = [r for r in rows if not all(re.fullmatch(r":?-{2,}:?", c) for c in r)]
    ncol = len(rows[0])
    data = [[Paragraph(inline(c), ST["cellb"] if (i == 0 or j == 0) else ST["cell"]) for j, c in enumerate(r)]
            for i, r in enumerate(rows)]
    total = 182 * mm
    widths = [total * 0.30] + [total * 0.70 / (ncol - 1)] * (ncol - 1) if ncol > 1 else [total]
    if ncol == 2:
        widths = [total * 0.32, total * 0.68]
    t = Table(data, colWidths=widths, repeatRows=1)
    t.setStyle(TableStyle([("VALIGN", (0, 0), (-1, -1), "TOP"),
                           ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#f1ece8")),
                           ("LINEBELOW", (0, 0), (-1, -1), 0.3, RULE),
                           ("TOPPADDING", (0, 0), (-1, -1), 3), ("BOTTOMPADDING", (0, 0), (-1, -1), 3),
                           ("LEFTPADDING", (0, 0), (-1, -1), 4), ("RIGHTPADDING", (0, 0), (-1, -1), 4)]))
    return [t, Spacer(1, 4 * mm)]


BULLET = re.compile(r"^(\s*)(\* |- |\d+\. )")


def parse_list(lines, i, indent, out):
    """Parse a (possibly nested) list starting at lines[i] with the given indent; append a ListFlowable."""
    items = []
    ordered = bool(re.match(r"^\s*\d+\. ", lines[i]))
    while i < len(lines):
        m = BULLET.match(lines[i])
        if not m or len(m.group(1)) < indent:
            break
        if len(m.group(1)) > indent:                       # nested list belongs to the previous item
            sub = []
            i = parse_list(lines, i, len(m.group(1)), sub)
            if items:
                items[-1][1].extend(sub)
            continue
        segs = [lines[i][m.end():].strip()]                  # raw text, formatted once so **bold** can wrap lines
        i += 1
        while i < len(lines) and lines[i].strip() and not BULLET.match(lines[i]) and \
                lines[i].startswith(" " * (indent + 2)):
            nxt = lines[i].strip()
            if nxt.startswith("`") and nxt.endswith("`"):
                segs.append("\n" + nxt)                      # equation on its own line inside the item
            elif segs[-1].startswith("\n"):                # text after an equation starts a new line
                segs.append(nxt)
            else:
                segs[-1] += " " + nxt
            i += 1
        text = "<br/>".join(inline(s.strip()) for s in segs)
        items.append((text, []))
    flow = []
    for text, sub in items:
        flow.append(ListItem([Paragraph(text, ST["body"])] + sub, leftIndent=12))
    out.append(ListFlowable(flow, bulletType="1" if ordered else "bullet", start=1 if ordered else None,
                            bulletFormat="%s." if ordered else None, bulletFontName=FONT,
                            bulletFontSize=9 if ordered else 7, leftIndent=12,
                            bulletColor=ACCENT))
    return i


def build():
    lines = open(SRC, encoding="utf-8").read().splitlines()
    story, para, i = [], [], 0
    cover_done = False

    def flush():
        if para:
            text = " ".join(s.strip() for s in para)
            story.append(Paragraph(inline(text), ST["body"]))
            para.clear()

    while i < len(lines):
        ln = lines[i]
        s = ln.strip()
        if s.startswith("# "):
            flush()
            story.append(Spacer(1, 30 * mm))
            story.append(Paragraph(inline(s[2:]), ST["title"]))
            story.append(HRFlowable(width="100%", thickness=2, color=ACCENT, spaceBefore=2, spaceAfter=10))
        elif s.startswith("## "):
            flush()
            if not cover_done:
                story.append(PageBreak())
                cover_done = True
            story.append(Paragraph(inline(s[3:]), ST["h2"]))
        elif s.startswith("### "):
            flush()
            story.append(Paragraph(inline(s[4:]), ST["h3"]))
        elif s == "---":
            flush()
        elif s.startswith("!["):
            flush()
            figs = re.findall(r"!\[(.*?)\]\((.*?)\)", s)
            if len(figs) == 1:
                story.append(figure(figs[0][1], figs[0][0], 165 * mm if not cover_done else 150 * mm))
            else:                              # side-by-side figures
                w = 178 * mm / len(figs)
                story.append(Table([[figure(p, c, w - 6 * mm, cell=True) for c, p in figs]], colWidths=[w] * len(figs)))
        elif s.startswith("|"):
            flush()
            block = []
            while i < len(lines) and lines[i].strip().startswith("|"):
                block.append(lines[i])
                i += 1
            story += table(block)
            continue
        elif re.match(r"^(\* |- |\d+\. )", s) and not ln.startswith(" "):
            flush()
            i = parse_list(lines, i, 0, story)
            continue
        elif s.startswith("`") and s.endswith("`") and len(s) > 2:
            flush()
            story.append(Paragraph(s.strip("`").replace("&", "&amp;").replace("<", "&lt;"), ST["eq"]))
        elif not s:
            flush()
        else:
            para.append(s)
        i += 1
    flush()

    def page(canvas, doc):
        canvas.saveState()
        if doc.page > 1:
            canvas.setFont(FONT, 7.5)
            canvas.setFillColor(GREY)
            canvas.drawString(14 * mm, 9 * mm, "Passive SMA Brake-Duct Actuator - Project Guidebook")
            canvas.drawRightString(A4[0] - 14 * mm, 9 * mm, f"{doc.page}")
            canvas.setStrokeColor(RULE)
            canvas.line(14 * mm, 12 * mm, A4[0] - 14 * mm, 12 * mm)
        canvas.restoreState()

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    SimpleDocTemplate(OUT, pagesize=A4, leftMargin=14 * mm, rightMargin=14 * mm, topMargin=14 * mm,
                      bottomMargin=17 * mm, title="Passive SMA Brake-Duct Actuator - Project Guidebook",
                      author="", subject="Project guidebook").build(story, onFirstPage=page, onLaterPages=page)
    print("WROTE", OUT)


if __name__ == "__main__":
    build()
