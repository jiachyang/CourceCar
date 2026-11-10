import copy, re
from pptx.util import Inches, Pt, Emu
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR
from pptx.enum.shapes import MSO_SHAPE, MSO_CONNECTOR
from pptx.enum.dml import MSO_LINE
from pptx.oxml.ns import qn
from lxml import etree

# ---------------- palette ----------------
NAVY   = RGBColor(0x1F,0x38,0x64)
BLUE   = RGBColor(0x25,0x55,0x7A)
MID    = RGBColor(0x2E,0x75,0xB6)
LIGHT  = RGBColor(0xD6,0xE4,0xF0)
PALE   = RGBColor(0xF3,0xF7,0xFB)
WHITE  = RGBColor(0xFF,0xFF,0xFF)
GRAY   = RGBColor(0x4F,0x60,0x73)
LGRAY  = RGBColor(0x8C,0x9A,0xAA)
XLGRAY = RGBColor(0xBF,0xC9,0xD4)
ORANGE = RGBColor(0xD9,0x7A,0x1F)
RED    = RGBColor(0xB8,0x3A,0x2E)
GREEN  = RGBColor(0x2E,0x7D,0x55)
TEAL   = RGBColor(0x1B,0x6F,0x7A)
DARK   = RGBColor(0x10,0x20,0x2F)
CODEBG = RGBColor(0x0F,0x1F,0x2E)
PATHBLUE = RGBColor(0x1E,0x88,0xE5)
FONT = '微软雅黑'
MONO = 'Consolas'

# ---------------- primitive helpers ----------------
def _margins(tf, m=0):
    tf.margin_left = Inches(m); tf.margin_right = Inches(m)
    tf.margin_top = Inches(m); tf.margin_bottom = Inches(m)

def add_text(s, x, y, w, h, text, size=11, color=GRAY, bold=False, font=FONT,
             align=PP_ALIGN.LEFT, anchor=MSO_ANCHOR.TOP, line=1.0, after=0,
             margins=0, italic=False):
    tb = s.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h))
    tf = tb.text_frame; tf.clear(); tf.word_wrap = True
    _margins(tf, margins); tf.vertical_anchor = anchor
    lines = str(text).split('\n')
    for i, tx in enumerate(lines):
        p = tf.paragraphs[0] if i == 0 else tf.add_paragraph()
        p.alignment = align; p.line_spacing = line
        p.space_after = Pt(after); p.space_before = Pt(0)
        r = p.add_run(); r.text = tx
        r.font.size = Pt(size); r.font.bold = bold; r.font.name = font
        r.font.color.rgb = color; r.font.italic = italic
    return tb

def add_paras(s, x, y, w, h, paras, anchor=MSO_ANCHOR.TOP, margins=0):
    tb = s.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h))
    tf = tb.text_frame; tf.clear(); tf.word_wrap = True
    _margins(tf, margins); tf.vertical_anchor = anchor
    for i, para in enumerate(paras):
        p = tf.paragraphs[0] if i == 0 else tf.add_paragraph()
        p.alignment = para.get('align', PP_ALIGN.LEFT)
        p.line_spacing = para.get('line', 1.0)
        p.space_before = Pt(para.get('before', 0))
        p.space_after = Pt(para.get('after', 0))
        if para.get('bullet'):
            _bullet(p, para.get('bullet_char', '•'), para.get('bullet_color', MID),
                    para.get('indent', 0.18))
        for rd in para['runs']:
            r = p.add_run(); r.text = rd['text']
            r.font.size = Pt(rd.get('size', 11))
            r.font.bold = rd.get('bold', False)
            r.font.italic = rd.get('italic', False)
            r.font.name = rd.get('font', FONT)
            r.font.color.rgb = rd.get('color', GRAY)
    return tb

def _bullet(p, char='•', color=MID, indent=0.18):
    pPr = p._p.get_or_add_pPr()
    pPr.set('marL', str(int(Inches(indent))))
    pPr.set('indent', str(-int(Inches(indent))))
    for tag in ('a:buClr', 'a:buFont', 'a:buChar', 'a:buNone'):
        for e in pPr.findall(qn(tag)):
            pPr.remove(e)
    buClr = etree.SubElement(pPr, qn('a:buClr'))
    srgb = etree.SubElement(buClr, qn('a:srgbClr')); srgb.set('val', str(color))
    buFont = etree.SubElement(pPr, qn('a:buFont')); buFont.set('typeface', 'Arial')
    buChar = etree.SubElement(pPr, qn('a:buChar')); buChar.set('char', char)

def add_rect(s, x, y, w, h, fill=WHITE, line_color=None, line_w=0.75,
             shape=MSO_SHAPE.RECTANGLE, radius=None, dash=None):
    shp = s.shapes.add_shape(shape, Inches(x), Inches(y), Inches(w), Inches(h))
    shp.shadow.inherit = False
    if fill is None:
        shp.fill.background()
    else:
        shp.fill.solid(); shp.fill.fore_color.rgb = fill
    if line_color is None:
        shp.line.fill.background()
    else:
        shp.line.color.rgb = line_color; shp.line.width = Pt(line_w)
        if dash is not None:
            shp.line.dash_style = dash
    if shape == MSO_SHAPE.ROUNDED_RECTANGLE and radius is not None:
        try: shp.adjustments[0] = radius
        except Exception: pass
    return shp

def add_line(s, x1, y1, x2, y2, color=MID, w=1.0, dash=None, arrow=False,
             head=False, cap='rnd'):
    c = s.shapes.add_connector(MSO_CONNECTOR.STRAIGHT, Inches(x1), Inches(y1),
                               Inches(x2), Inches(y2))
    c.line.color.rgb = color; c.line.width = Pt(w)
    if dash is not None:
        c.line.dash_style = dash
    ln = c.line._get_or_add_ln(); ln.set('cap', cap)
    if head:
        hd = etree.SubElement(ln, qn('a:headEnd'))
        hd.set('type', 'triangle'); hd.set('w', 'med'); hd.set('len', 'med')
    if arrow:
        tail = etree.SubElement(ln, qn('a:tailEnd'))
        tail.set('type', 'triangle'); tail.set('w', 'med'); tail.set('len', 'med')
    return c

def add_arrow_polyline(s, pts, color=MID, w=1.2, dash=None, arrow=True):
    for i in range(len(pts) - 1):
        add_line(s, *pts[i], *pts[i+1], color=color, w=w, dash=dash,
                 arrow=(arrow and i == len(pts) - 2))

def add_circle(s, cx, cy, d, fill=NAVY, line_color=None, text=None, size=12,
               color=WHITE, bold=True, font=FONT):
    shp = s.shapes.add_shape(MSO_SHAPE.OVAL, Inches(cx-d/2), Inches(cy-d/2), Inches(d), Inches(d))
    shp.shadow.inherit = False
    if fill is None: shp.fill.background()
    else: shp.fill.solid(); shp.fill.fore_color.rgb = fill
    if line_color is None: shp.line.fill.background()
    else: shp.line.color.rgb = line_color; shp.line.width = Pt(1)
    if text is not None:
        tf = shp.text_frame; tf.clear(); tf.word_wrap = False
        _margins(tf, 0); tf.vertical_anchor = MSO_ANCHOR.MIDDLE
        p = tf.paragraphs[0]; p.alignment = PP_ALIGN.CENTER
        r = p.add_run(); r.text = str(text)
        r.font.size = Pt(size); r.font.bold = bold; r.font.name = font
        r.font.color.rgb = color
    return shp

def add_chip(s, x, y, w, h, label, value, fill=PALE, line_color=LIGHT,
             value_color=NAVY, label_color=LGRAY):
    add_rect(s, x, y, w, h, fill=fill, line_color=line_color, line_w=0.75,
             shape=MSO_SHAPE.ROUNDED_RECTANGLE, radius=0.10)
    add_text(s, x+0.12, y+0.09, w-0.24, 0.20, label, size=8, color=label_color, bold=True)
    add_text(s, x+0.12, y+0.27, w-0.24, h-0.32, value, size=15, color=value_color, bold=True)

def add_status(s, text, color=GREEN, x=8.75, y=0.56, w=2.35):
    add_text(s, x, y, w, 0.28, '● ' + text, size=9, color=color,
             bold=True, align=PP_ALIGN.RIGHT)

def add_rule(s, y=1.36):
    add_rect(s, 0.55, y, 12.2, 0.008, fill=RGBColor(0xDD, 0xE6, 0xF0))

def add_footer(s, page, total=20):
    add_rect(s, 0.55, 7.10, 12.2, 0.007, fill=RGBColor(0xE5, 0xEB, 0xF2))
    add_text(s, 0.55, 7.15, 5.0, 0.2, '程序部分 · VEX V5 迷宫机器人', size=7.5,
             color=XLGRAY)
    add_text(s, 11.85, 7.15, 0.9, 0.2, f'{page:02d} / {total}', size=7.5,
             color=XLGRAY, align=PP_ALIGN.RIGHT)

def add_formula(s, x, y, w, h, lines, size=12, color=NAVY, fill=PALE, accent=MID):
    add_rect(s, x, y, w, h, fill=fill, line_color=LIGHT, line_w=0.75,
             shape=MSO_SHAPE.ROUNDED_RECTANGLE, radius=0.06)
    add_rect(s, x, y, 0.06, h, fill=accent)
    add_text(s, x+0.18, y+0.10, w-0.30, h-0.18, lines, size=size, color=color,
             font=MONO, line=1.18, anchor=MSO_ANCHOR.MIDDLE)

def add_callout(s, x, y, w, h, text, color=ORANGE, fill=RGBColor(0xFD,0xF4,0xE8),
                size=10, bold=False, icon='!'):
    add_rect(s, x, y, w, h, fill=fill, line_color=None,
             shape=MSO_SHAPE.ROUNDED_RECTANGLE, radius=0.08)
    add_rect(s, x, y, 0.06, h, fill=color)
    add_text(s, x+0.18, y+0.08, w-0.28, h-0.16, text, size=size, color=GRAY,
             bold=bold, line=1.12, anchor=MSO_ANCHOR.MIDDLE)

# ---------------- tables ----------------
def status_color(val):
    if '已实现' in val or '通过' in val or '完成' in val or '一致' in val: return GREEN
    if any(x in val for x in ('需确认', '待对齐', '需核对')): return ORANGE
    if '机械先行' in val: return MID
    if '部分' in val or '进行中' in val: return ORANGE
    if '待' in val: return LGRAY
    return GRAY

def manual_table(s, x, y, w, headers, rows, col_fracs, row_h=0.42, header_h=0.44,
                 fs=10, header_fs=9.2, status_col=None, first_bold=True):
    add_rect(s, x, y, w, header_h, fill=BLUE)
    cx = x
    for h, f in zip(headers, col_fracs):
        add_text(s, cx+0.10, y, w*f-0.14, header_h, h, size=header_fs, color=WHITE,
                 bold=True, anchor=MSO_ANCHOR.MIDDLE)
        cx += w*f
    yy = y + header_h
    for i, row in enumerate(rows):
        fill = PALE if i % 2 == 0 else WHITE
        add_rect(s, x, yy, w, row_h, fill=fill)
        add_line(s, x, yy+row_h, x+w, yy+row_h, color=RGBColor(0xE1,0xE8,0xF1), w=0.7)
        cx = x
        for j, (val, f) in enumerate(zip(row, col_fracs)):
            col = status_color(val) if j == status_col else (NAVY if (j == 0 and first_bold) else GRAY)
            bold = (j == 0 and first_bold) or (j == status_col)
            add_text(s, cx+0.10, yy, w*f-0.14, row_h, val, size=fs, color=col,
                     bold=bold, anchor=MSO_ANCHOR.MIDDLE, line=0.95)
            cx += w*f
        yy += row_h
    add_rect(s, x, y, w, header_h + row_h*len(rows), fill=None, line_color=RGBColor(0xD5,0xDF,0xEA), line_w=0.8)

# ---------------- code block ----------------
_KEY = {'double','if','else','const','return','for','while','bool','void','static','auto',
        'int','uint32_t','std','enableHeadingHold','enableSlewLimit','true','false'}
_TOK = re.compile(r'(\b(?:double|if|else|const|return|for|while|bool|void|static|auto|int|uint32_t|true|false)\b|\b\d+(?:\.\d+)?\b|"[^"]*"|[(),;{}\[\]]|->|\.)')

def _code_tokens(p, text, fs):
    pos = 0
    for m in _TOK.finditer(text):
        if m.start() > pos:
            r = p.add_run(); r.text = text[pos:m.start()]
            r.font.size = Pt(fs); r.font.name = MONO; r.font.color.rgb = RGBColor(0xDC,0xE6,0xF0)
        tok = m.group(0); pos = m.end()
        r = p.add_run(); r.text = tok
        r.font.size = Pt(fs); r.font.name = MONO
        if tok in _KEY: r.font.color.rgb = RGBColor(0x7F,0xB3,0xE8)
        elif re.fullmatch(r'\d+(?:\.\d+)?', tok): r.font.color.rgb = RGBColor(0xE8,0xB3,0x6A)
        elif tok.startswith('"'): r.font.color.rgb = RGBColor(0x9C,0xD6,0x8C)
        elif tok in '(),;{}[]': r.font.color.rgb = RGBColor(0xA9,0xB8,0xC7)
        else: r.font.color.rgb = RGBColor(0xDC,0xE6,0xF0)

def code_box(s, x, y, w, h, code, fs=8.6, title=None, line=1.12):
    add_rect(s, x, y, w, h, fill=CODEBG, line_color=RGBColor(0x1E,0x3A,0x52), line_w=0.8,
             shape=MSO_SHAPE.ROUNDED_RECTANGLE, radius=0.035)
    if title:
        add_text(s, x+w-2.2, y+0.08, 2.05, 0.2, title, size=7, color=RGBColor(0x7F,0x93,0xA8),
                 font=MONO, align=PP_ALIGN.RIGHT)
    tb = s.shapes.add_textbox(Inches(x+0.16), Inches(y+0.14), Inches(w-0.30), Inches(h-0.24))
    tf = tb.text_frame; tf.clear(); tf.word_wrap = False
    _margins(tf, 0)
    for i, ln in enumerate(code.split('\n')):
        p = tf.paragraphs[0] if i == 0 else tf.add_paragraph()
        p.line_spacing = line; p.space_after = Pt(0)
        if not ln:
            r = p.add_run(); r.text = ' '; r.font.size = Pt(fs); r.font.name = MONO
            continue
        if '//' in ln:
            code_part, comment = ln.split('//', 1)
            _code_tokens(p, code_part, fs)
            r = p.add_run(); r.text = '//' + comment
            r.font.size = Pt(fs); r.font.name = MONO
            r.font.color.rgb = RGBColor(0x7F,0xA8,0x8A)
        else:
            _code_tokens(p, ln, fs)
    return tb

# ---------------- maze ----------------
def _grid_pt(x0, y0, cell, n, c, r):
    return x0 + c*cell, y0 + (n-r)*cell

def draw_maze(s, x0, y0, cell=0.50, n=8, walls=None, partitions=None, path=None,
              wall_w=2.2, part_w=3.0):
    W = cell*n
    # floor & grid
    add_rect(s, x0, y0, W, W, fill=RGBColor(0xFA,0xFC,0xFE), line_color=None)
    for i in range(n+1):
        add_line(s, x0+i*cell, y0, x0+i*cell, y0+W, color=RGBColor(0xDF,0xE8,0xF2), w=0.55)
        add_line(s, x0, y0+i*cell, x0+W, y0+i*cell, color=RGBColor(0xDF,0xE8,0xF2), w=0.55)
    # outer wall
    add_rect(s, x0, y0, W, W, fill=None, line_color=NAVY, line_w=2.4)
    # interior fixed walls
    for (a, b) in walls or []:
        p1 = _grid_pt(x0,y0,cell,n,*a); p2 = _grid_pt(x0,y0,cell,n,*b)
        add_line(s, *p1, *p2, color=BLUE, w=wall_w, cap='flat')
    # path first: white halo then bright line
    if path:
        pts = [_grid_pt(x0,y0,cell,n,c,r) for c,r in path]
        for i in range(len(pts)-1):
            add_line(s, *pts[i], *pts[i+1], color=WHITE, w=4.6, cap='round')
            add_line(s, *pts[i], *pts[i+1], color=PATHBLUE, w=2.5,
                     arrow=(i == len(pts)-2), cap='round')
    # movable partitions on top so their labels stay readable
    for (a,b,label) in partitions or []:
        p1 = _grid_pt(x0,y0,cell,n,*a); p2 = _grid_pt(x0,y0,cell,n,*b)
        add_line(s, *p1, *p2, color=ORANGE, w=part_w, dash=MSO_LINE.DASH, cap='flat')
        mx, my = (p1[0]+p2[0])/2, (p1[1]+p2[1])/2
        add_rect(s, mx-0.20, my-0.10, 0.40, 0.20, fill=WHITE, line_color=ORANGE, line_w=0.6,
                 shape=MSO_SHAPE.ROUNDED_RECTANGLE, radius=0.25)
        add_text(s, mx-0.20, my-0.105, 0.40, 0.20, label, size=7, color=ORANGE,
                 bold=True, align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.MIDDLE)
    return W

def marker(s, x0, y0, cell, n, c, r, label, fill, textcolor=WHITE, d=0.34):
    px, py = _grid_pt(x0,y0,cell,n,c,r)
    add_circle(s, px, py, d, fill=fill, text=label, size=9.5, color=textcolor, bold=True)
