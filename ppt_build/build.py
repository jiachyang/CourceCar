import copy, sys
from pptx import Presentation
from pptx.util import Inches, Pt
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR
from pptx.enum.shapes import MSO_SHAPE
from pptx.enum.dml import MSO_LINE
from pptx.oxml.ns import qn
from helpers import *

TEMPLATE = '/home/jc/下载/华科蓝.pptx'
OUT = '/home/jc/下载/VEX_V5_迷宫机器人_程序实现.pptx'

prs = Presentation(TEMPLATE)
original = list(prs.slides)
cover, toc, sec1, seed = original[0], original[1], original[2], original[3]
sec2, sec3, sec4 = original[5], original[8], original[11]

# ---------- extract header / corner from a content slide ----------
HEADER = copy.deepcopy(original[3].shapes[0]._element)   # P logo + title/subtitle
CORNER = copy.deepcopy(original[3].shapes[8]._element)   # HUST corner mark
for el in HEADER.iter(qn('a:t')):
    if el.text in ('请输入标题', 'PLEASE ENTER THE TOPIC'):
        el.text = ''

# Reuse the 8 existing content slides as blank canvases; add 6 new ones later.
content_pool = [original[i] for i in (3,4,6,7,9,10,12,13)]
thanks_slide = original[14]

# ---------- cover ----------
def set_group_text(group, predicate, text, size, color, bold=False, font=FONT, italic=False):
    for sh in group.shapes:
        if sh.has_text_frame and predicate(sh.text_frame.text):
            tf = sh.text_frame; tf.clear(); tf.word_wrap = True
            p = tf.paragraphs[0]
            r = p.add_run(); r.text = text
            r.font.size = Pt(size); r.font.bold = bold; r.font.name = font
            r.font.color.rgb = color; r.font.italic = italic
            return True
    return False

cover_group = None
for sh in cover.shapes:
    if sh.name == '组合 10':
        cover_group = sh
set_group_text(cover_group, lambda t: '华中大' in t, '走迷宫任务小车', 38, NAVY, True)
set_group_text(cover_group, lambda t: '学术报告' in t,
               '机械结构  ×  四轮全向底盘  ×  感知与规划', 17, BLUE, True)
set_group_text(cover_group, lambda t: '汇报人' in t,
               '方案中期汇报   |   第八组   |   2026 年 10 月', 13, GRAY)
add_text(cover, 2.18, 1.55, 8.9, 0.30,
         'VEX V5   ·   PROS   ·   机械 + 程序   ·   中期检查',
         size=9, color=LGRAY, font='Arial', bold=True)

# ---------- toc ----------
toc_items = [
    ('任务与总体方案', 'TASK & CONCEPT'),
    ('机械方案与进展', 'MECHANICAL PROGRESS'),
    ('接口对比与融合', 'INTERFACE & INTEGRATION'),
    ('中期进度与下一步', 'STATUS & NEXT'),
]
found = []
def collect_toc(shapes):
    for sh in shapes:
        if sh.shape_type == 6:
            collect_toc(sh.shapes)
        elif sh.has_text_frame and sh.text_frame.text.strip() == '请输入标题':
            found.append(sh)
collect_toc(toc.shapes)
for sh, (zh, en) in zip(found, toc_items):
    tf = sh.text_frame; tf.clear(); tf.word_wrap = True
    tf.margin_left = 0; tf.margin_right = 0
    p = tf.paragraphs[0]
    r = p.add_run(); r.text = zh
    r.font.size = Pt(20); r.font.bold = True; r.font.name = FONT; r.font.color.rgb = NAVY
    p2 = tf.add_paragraph(); p2.space_before = Pt(2)
    r2 = p2.add_run(); r2.text = en
    r2.font.size = Pt(8.5); r2.font.name = 'Arial'; r2.font.color.rgb = LGRAY

# ---------- section dividers ----------
def setup_section(slide, title, en):
    def walk(shapes):
        for sh in shapes:
            if sh.shape_type == 6:
                walk(sh.shapes)
            elif sh.has_text_frame:
                t = sh.text_frame.text.strip()
                if t == '请输入标题':
                    tf = sh.text_frame; tf.clear(); tf.word_wrap = True
                    p = tf.paragraphs[0]
                    r = p.add_run(); r.text = title
                    r.font.size = Pt(40); r.font.bold = True; r.font.name = FONT
                    r.font.color.rgb = RGBColor(0,0,0)
                elif 'PLEASE ENTER THE TOPIC' in t:
                    tf = sh.text_frame; tf.clear()
                    p = tf.paragraphs[0]
                    r = p.add_run(); r.text = en
                    r.font.size = Pt(9); r.font.name = 'Arial'; r.font.color.rgb = LGRAY
    walk(slide.shapes)

setup_section(sec1, '任务与总体方案', 'TASK  &  OVERALL  CONCEPT')
setup_section(sec2, '底盘程序实现', 'X-DRIVE  ·  IMU  HEADING  ·  REAL-CAR  DEBUG')
setup_section(sec3, '四向测距融合定位', 'RANGE + GYRO  ·  EKF')
setup_section(sec4, '规划、集成与验证', 'PLANNING  ·  INTEGRATION  ·  VALIDATION')

# ---------- content slide chrome ----------
content_slides = []
def new_content(title, sub, status='', status_color=GREEN, page=4):
    if content_pool:
        s = content_pool.pop(0)
    else:
        s = prs.slides.add_slide(prs.slide_layouts[6])
    for sh in list(s.shapes):
        sh._element.getparent().remove(sh._element)
    s.shapes._spTree.insert_element_before(copy.deepcopy(HEADER), 'p:extLst')
    s.shapes._spTree.insert_element_before(copy.deepcopy(CORNER), 'p:extLst')
    add_text(s, 1.55, 0.40, 7.15, 0.62, title, size=26, color=NAVY, bold=True)
    add_text(s, 1.57, 0.99, 8.0, 0.26, sub, size=8.5, color=LGRAY, font='Arial', bold=True)
    add_rule(s)
    if status:
        add_status(s, status, status_color)
    if page is not None:
        add_footer(s, page)
    content_slides.append(s)
    return s

# =====================================================================
# 04 任务定义与核心挑战
# =====================================================================
s = new_content('任务定义与核心挑战',
                'TASK DEFINITION  ·  8 × 8 GRID  ·  45 CM CELL  ·  180 S LIMIT',
                '规则确认', GREEN, 4)
# exact geometry from maze_interactive_review.html
# fixed walls: vertical (x, y_start, y_end) and horizontal (y, x_start, x_end)
V = [(3,0,1),(1,1,2),(2,2,3),(4,0,4),(5,1,2),(6,0,1),(6,2,4),
     (7,1,5),(3,3,5),(2,5,7),(5,5,6),(4,6,7),(1,6,8),(3,7,8),
     (5,7,8),(6,5,7)]
H = [(1,1,3),(2,2,4),(2,5,7),(3,0,1),(3,4,5),(4,1,3),(4,5,6),
     (5,3,5),(6,2,5),(6,7,8),(7,6,7)]
walls = [((x, 8-a), (x, 8-b)) for x,a,b in V]
walls += [((a, 8-y), (b, 8-y)) for y,a,b in H]
parts = [((1,5),(1,4),'M1'), ((5,4),(5,3),'M2'), ((6,3),(7,3),'M3')]
# example order S -> B -> C -> A -> F, drawn with M1-M3 assumed uninstalled
path_html = [(4,7),(4,6),(5,6),(5,5),(5,4),(4,4),(3,4),(3,3),(3,2),
             (2,2),(2,3),(1,3),(0,3),(1,3),(1,2),(1,1),(2,1),(3,1),(3,0)]
path = [(x+0.5, 7.5-y) for x,y in path_html]
x0, y0, cell = 0.65, 1.62, 0.50
draw_maze(s, x0, y0, cell, 8, walls, parts, path)
marker(s, x0,y0,cell,8, 4.5,0.5, 'S', TEAL)
marker(s, x0,y0,cell,8, 3.5,7.5, 'F', NAVY)
marker(s, x0,y0,cell,8, 0.5,4.5, 'A', RED)
marker(s, x0,y0,cell,8, 5.5,3.5, 'B', ORANGE)
marker(s, x0,y0,cell,8, 3.5,5.5, 'C', GREEN)
# column / row indices (0..7), faint
for i in range(8):
    add_text(s, x0+(i+0.5)*cell-0.10, y0-0.20, 0.20, 0.16, str(i), size=6.2, color=XLGRAY, align=PP_ALIGN.CENTER)
    add_text(s, 0.45, y0+(i+0.5)*cell-0.08, 0.16, 0.16, str(i), size=6.2, color=XLGRAY, align=PP_ALIGN.RIGHT)
# legend (two rows, all inside the 4.0 in map column)
ly = 5.68
add_line(s, 0.65, ly+0.10, 1.05, ly+0.10, color=BLUE, w=3.0, cap='flat')
add_text(s, 1.12, ly, 1.0, 0.22, '已知墙体', size=8, color=GRAY)
add_line(s, 2.15, ly+0.10, 2.55, ly+0.10, color=ORANGE, w=3.0, dash=MSO_LINE.DASH, cap='flat')
add_text(s, 2.62, ly, 1.85, 0.22, '随机隔板 M1–M3', size=8, color=GRAY)
add_line(s, 0.65, ly+0.23, 1.05, ly+0.23, color=PATHBLUE, w=2.5, arrow=True)
add_text(s, 1.12, ly+0.13, 1.0, 0.22, '初始路径', size=8, color=GRAY)
add_text(s, 0.65, 6.14, 4.00, 0.20,
         '初始路径 S→B→C→A→F；按隔板未安装，检出 M1/M2 后重规划。', size=6.9, color=LGRAY)
add_text(s, 0.65, 6.35, 4.00, 0.20,
         '坐标：左上原点，x→右，y→下。', size=6.9, color=LGRAY)
add_text(s, 0.65, 6.56, 4.00, 0.20,
         'A/B/C = 红/黄/绿物块；几何见交互校对图。', size=6.6, color=XLGRAY)

# metrics
mx, my, mw, mh, gap = 5.00, 1.62, 1.85, 0.76, 0.10
metrics = [('迷宫网格','8 × 8'), ('单元格','45 cm'), ('任务限时','180 s'), ('物块 / 隔板','3 + 3')]
for i,(lab,val) in enumerate(metrics):
    add_chip(s, mx+i*(mw+gap), my, mw, mh, lab, val, fill=PALE, line_color=LIGHT)

# task flow
add_text(s, 5.00, 2.60, 7.4, 0.26, '任务流程', size=10, color=BLUE, bold=True)
steps = [
    ('01', '从 S 出发，访问 A / B / C，识别并抓取红、黄、绿三块物块；顺序由规划器决定。'),
    ('02', '搬运途中检测 M1–M3 随机隔板的实际状态，更新可通行图。'),
    ('03', '动态调整路线，把三块物块全部送至 F，总时间不超过 180 s。'),
]
for i,(num,txt) in enumerate(steps):
    yy = 2.90 + i*0.56
    add_circle(s, 5.18, yy+0.16, 0.30, fill=BLUE, text=num, size=7.5)
    add_text(s, 5.45, yy, 7.0, 0.36, txt, size=9.5, color=GRAY, line=1.02)

# challenges
add_text(s, 5.00, 4.66, 7.4, 0.26, '核心挑战', size=10, color=BLUE, bold=True)
challenges = [
    ('通道狭窄', '全向底盘需在 45 cm 单元格内完成贴边抓取与转向。'),
    ('轮子打滑', '仅靠编码器积分会累积误差，必须有外部观测。'),
    ('地图未知', '3 块随机隔板改变拓扑，最短路线不能预先写死。'),
    ('多目标顺序', 'A/B/C 的访问顺序影响总时间，需要联合优化。'),
    ('嵌入式约束', '所有算法必须在 V5 Brain 上以 20 ms 周期稳定运行。'),
]
for i,(head,txt) in enumerate(challenges):
    yy = 4.96 + i*0.35
    add_circle(s, 5.10, yy+0.09, 0.11, fill=ORANGE)
    add_text(s, 5.25, yy-0.02, 1.05, 0.22, head, size=9, color=NAVY, bold=True)
    add_text(s, 6.28, yy-0.02, 6.4, 0.24, txt, size=9, color=GRAY)

# =====================================================================
# 05 设计目标与技术指标
# =====================================================================
s = new_content('设计目标与技术指标',
                'DESIGN TARGETS  ·  SUBSYSTEM  ·  IMPLEMENTATION  ·  CURRENT STATUS',
                '目标 vs 现状', ORANGE, 5)
rows = [
    ['移动平台','前后、横移、原地旋转','X-Drive 四全向轮 + 4×3 逆运动学','● 已实现'],
    ['航向控制','抑制平移时的非预期偏航','IMU + PID + 方向相关静摩擦前馈','● 已实现'],
    ['位置估计','连续输出 x / y / θ 与协方差','编码器 + IMU + 四向测距 → EKF','● 待实现'],
    ['地图管理','已知墙体 + 随机隔板','8×8 栅格 + 墙段模型 + 置信度','● 待实现'],
    ['路径规划','生成低时间成本路线','Dijkstra + 3! 任务顺序 DP','● 待实现'],
    ['障碍感知','判断 M1–M3 是否安装','测距值与地图几何匹配','● 部分实现'],
    ['任务执行','抓取、搬运、交付','任务状态机 + 抓取机构','● 待实现'],
    ['软件平台','20 ms 实时、模块协作','VEX V5 + PROS + C++17','● 已实现'],
]
manual_table(s, 0.55, 1.62, 12.2, ['子系统','设计目标','实现方式','当前状态'], rows,
             [0.13,0.26,0.47,0.14], row_h=0.43, header_h=0.44, fs=9.8, status_col=3)
add_callout(s, 0.55, 5.72, 12.2, 0.82,
            '设计原则：以已知地图减少探索成本，以多传感器融合减少定位漂移，以动态规划减少运输时间。',
            color=MID, fill=RGBColor(0xEC,0xF4,0xFB), size=10.5, bold=True)
add_text(s, 0.55, 6.60, 12.2, 0.25,
         '注：状态按当前代码仓库标注；定位、地图与规划为下一阶段目标方案，不表示已完成实车验证。',
         size=7.8, color=LGRAY)

# =====================================================================
# 06 系统总体方案
# =====================================================================
s = new_content('程序方案：架构与中期状态',
                'SOFTWARE ARCHITECTURE  ·  MODULE LAYERS  ·  MID-TERM STATUS',
                '程序部分', MID, 6)
cols = [
    ('01','任务输入层', '8×8 地图\n目标 A / B / C / F\n3 个随机隔板\n180 s 规则', RGBColor(0xEE,0xF3,0xF8), NAVY, LIGHT, None, GRAY),
    ('02','决策规划层', '全局路径\n任务顺序\n隔板后重规划\n风险代价', RGBColor(0xE8,0xF0,0xF8), NAVY, MID, MSO_LINE.DASH, GRAY),
    ('03','导航控制层', '路径跟踪\n航向闭环\n全向运动学\n速度限幅', RGBColor(0xDC,0xE9,0xF5), NAVY, MID, None, GRAY),
    ('04','执行机构层', '四轮全向底盘（已制造）\n单自由度机械臂（3D 完成）\n抓取 / 收纳\n20 ms 周期', RGBColor(0xDC,0xE9,0xF5), NAVY, MID, None, GRAY),
    ('05','传感与估计', 'IMU + 4× ToF\n4× 编码器\n位姿估计\n状态反馈', RGBColor(0xE6,0xF2,0xF2), NAVY, TEAL, None, GRAY),
]
bw, gap, by, bh = 2.20, 0.28, 1.60, 2.35
for i,(num,title,body,fill,tcol,lcol,dash,txtcol) in enumerate(cols):
    x = 0.59 + i*(bw+gap)
    add_rect(s, x, by, bw, bh, fill=fill, line_color=lcol, line_w=1.1,
             shape=MSO_SHAPE.ROUNDED_RECTANGLE, radius=0.055, dash=dash)
    add_text(s, x+0.16, by+0.13, 0.5, 0.2, num, size=8, color=tcol, bold=True, font='Arial')
    add_text(s, x+0.16, by+0.38, bw-0.28, 0.30, title, size=12, color=tcol, bold=True)
    add_text(s, x+0.16, by+0.82, bw-0.28, bh-0.90, body, size=8.8, color=txtcol, line=1.28)
    if i < 4:
        add_line(s, x+bw+0.03, by+bh/2, x+bw+gap-0.03, by+bh/2, color=MID, w=1.4, arrow=True)
# feedback
add_arrow_polyline(s, [(12.05,3.95),(12.05,4.55),(7.24,4.55),(7.24,3.95)],
                   color=TEAL, w=1.3, dash=MSO_LINE.DASH, arrow=True)
add_text(s, 8.75, 4.66, 3.0, 0.24, '状态反馈', size=8, color=TEAL, bold=True)

# bottom principles: mechanical baseline / software architecture / mid-term focus
calls = [
    ('机械基线', '四轮全向底盘已制造；单自由度机械臂三维设计完成，等待装配。'),
    ('程序架构', '感知 → 定位 → 规划 → 控制；模块之间只传命令与状态。'),
    ('中期重点', '先打通底盘 / 测距 / 机械臂，再做定位与整机联调；不追求一次到位。'),
]
for i,(h,t) in enumerate(calls):
    x = 0.59 + i*4.02
    add_rect(s, x, 5.02, 3.82, 1.05, fill=WHITE, line_color=LIGHT, line_w=0.9,
             shape=MSO_SHAPE.ROUNDED_RECTANGLE, radius=0.06)
    add_rect(s, x, 5.02, 0.055, 1.05, fill=MID)
    add_text(s, x+0.18, 5.12, 3.5, 0.24, h, size=9.5, color=NAVY, bold=True)
    add_text(s, x+0.18, 5.40, 3.48, 0.60, t, size=8.3, color=GRAY, line=1.16)
add_text(s, 0.59, 6.38, 12.1, 0.26,
         '中期状态：底盘制造完成；驱动 / IMU / 四路测距读取已落地；定位、规划、夹取联调待推进。',
         size=8.3, color=LGRAY)

# =====================================================================
# 07 代码地图与模块边界
# =====================================================================
s = new_content('代码地图与模块边界',
                'REPOSITORY  ·  SINGLE OWNER PER PERIPHERAL  ·  PURE MATH TESTABLE ON HOST',
                '代码现状', GREEN, 7)
add_rect(s, 0.55, 1.60, 6.45, 4.80, fill=RGBColor(0xF7,0xFA,0xFD), line_color=LIGHT, line_w=0.9,
         shape=MSO_SHAPE.ROUNDED_RECTANGLE, radius=0.03)
add_text(s, 0.75, 1.72, 4.5, 0.26, 'Car/ 代码地图（当前仓库）', size=10, color=BLUE, bold=True)
tree = [
    ('Car/', '', 'root'),
    ('├─ src/main.cpp', '调度 / 初始化 / 生命周期', 'file'),
    ('├─ modules/chassis.cpp', '航向 → IK → 驱动', 'hot'),
    ('├─ chassis/kinematics.cpp', '4×3 逆运动学（纯数学）', 'file'),
    ('├─ chassis/chassis_drive.cpp', '饱和 / slew / 电压下发', 'hot'),
    ('├─ chassis/heading_controller.cpp', 'PID + 静摩擦前馈', 'hot'),
    ('├─ modules/imu.cpp', '校准 / 读数 / 单位换算', 'hot'),
    ('├─ modules/distance_array.cpp', '四路 ToF 读取 / 遥测', 'file'),
    ('├─ modules/radio.cpp', '遥控 / 失联保护', 'file'),
    ('└─ tests/native/', 'IK / 航向 / 映射单测', 'file'),
]
for i,(path,desc,kind) in enumerate(tree):
    yy = 2.12 + i*0.41
    if kind == 'hot':
        add_rect(s, 0.68, yy-0.03, 6.18, 0.36, fill=RGBColor(0xE8,0xF1,0xFA), line_color=None,
                 shape=MSO_SHAPE.ROUNDED_RECTANGLE, radius=0.10)
    col = NAVY if kind in ('root','hot') else BLUE
    add_text(s, 0.78, yy, 3.45, 0.30, path, size=8.7, color=col, font=MONO,
             bold=(kind in ('root','hot')))
    add_text(s, 4.28, yy, 2.55, 0.30, desc, size=8.6, color=GRAY)
add_text(s, 0.75, 6.02, 6.0, 0.25,
         '热路径：20 ms 内完成 航向 → IK → 驱动；测距只读取，不阻塞控制。',
         size=7.8, color=LGRAY)

# right cards
cards = [
    ('01','唯一硬件管理者', '电机 → ChassisDrive；IMU → Imu；ToF → DistanceArray；遥控 → Radio。'),
    ('02','纯数学可单测', 'kinematics / heading_controller 不依赖 PROS，主机端直接跑单测。'),
    ('03','配置真值分层', '硬件与控制参数放 *_config.hpp；main 只持有整车级开关。'),
    ('04','扩展点清晰', '新增 Localization 只产出 Pose；不改 IK、电机驱动与安全链路。'),
]
for i,(num,h,t) in enumerate(cards):
    yy = 1.60 + i*0.98
    add_rect(s, 7.22, yy, 5.52, 0.86, fill=WHITE, line_color=LIGHT, line_w=0.9,
             shape=MSO_SHAPE.ROUNDED_RECTANGLE, radius=0.07)
    add_circle(s, 7.50, yy+0.30, 0.30, fill=BLUE, text=num, size=7.2)
    add_text(s, 7.76, yy+0.12, 2.2, 0.24, h, size=9.6, color=NAVY, bold=True)
    add_text(s, 7.76, yy+0.38, 4.80, 0.44, t, size=8.3, color=GRAY, line=1.12)
code_box(s, 7.22, 5.58, 5.52, 0.86,
         'chassis.setCommand({vx, vy, omega});\nchassis.setImuState(imu.getState());\nchassis.update(dt);      // 20 ms',
         fs=8.3, title='main.cpp')
add_text(s, 7.22, 6.52, 5.5, 0.24, '新增定位模块时，只改 main 的调度与数据流。',
         size=8, color=LGRAY)


# =====================================================================
# 09 X-Drive 构型与逆运动学
# =====================================================================
s = new_content('X-Drive 构型与逆运动学',
                '4 OMNI WHEELS  ·  DIAGONAL HEADING  ·  4 × 3 INVERSE KINEMATICS',
                '已实现', GREEN, 9)
# left panel
add_rect(s, 0.55, 1.55, 6.05, 5.02, fill=RGBColor(0xF8,0xFB,0xFE), line_color=LIGHT, line_w=0.8,
         shape=MSO_SHAPE.ROUNDED_RECTANGLE, radius=0.03)
add_text(s, 0.75, 1.65, 4.0, 0.24, '机械构型（俯视）', size=9.5, color=BLUE, bold=True)
cx, cy, R = 3.55, 3.95, 1.55
# frame diamond: square rotated 45 deg
side = 2*R/(2**0.5)
add_rect(s, cx-side/2, cy-side/2, side, side, fill=RGBColor(0xE8,0xF1,0xFA), line_color=NAVY,
         line_w=2.2, shape=MSO_SHAPE.RECTANGLE).rotation = 45
# wheels
wheels = [('FL', 2.775, 3.175, -45, '1'), ('FR', 4.325, 3.175, 45, '3'),
          ('RL', 2.775, 4.725, 45, '2'), ('RR', 4.325, 4.725, -45, '4')]
for name, wx, wy, rot, port in wheels:
    shp = add_rect(s, wx-0.34, wy-0.09, 0.68, 0.18, fill=RGBColor(0x33,0x47,0x5C), line_color=None,
                   shape=MSO_SHAPE.ROUNDED_RECTANGLE, radius=0.35)
    shp.rotation = rot
    # center line along rolling direction
    dx, dy = (0.23, -0.23) if rot == -45 else (0.23, 0.23)
    add_line(s, wx-dx, wy-dy, wx+dx, wy+dy, color=RGBColor(0xB9,0xC9,0xD8), w=0.8, cap='rnd')
# labels + ports
lab = [('FL', 2.02, 2.78, '1'), ('FR', 4.62, 2.78, '3'), ('RL', 2.02, 5.12, '2'), ('RR', 4.62, 5.12, '4')]
for name, lx, ly, port in lab:
    add_text(s, lx, ly, 0.55, 0.22, name, size=9, color=NAVY, bold=True)
    add_circle(s, lx+0.42, ly+0.10, 0.20, fill=BLUE, text=port, size=6.8)
# axes
add_line(s, cx, cy, cx, cy-1.02, color=NAVY, w=1.8, arrow=True)
add_line(s, cx, cy, cx-1.02, cy, color=NAVY, w=1.8, arrow=True)
add_text(s, cx+0.10, cy-1.10, 0.95, 0.24, '+X 车头', size=8.5, color=NAVY, bold=True)
add_text(s, cx-1.82, cy+0.08, 0.80, 0.24, '+Y 左侧', size=8.5, color=NAVY, bold=True)
# R line
add_line(s, cx, cy, 2.775, 3.175, color=GRAY, w=0.9, dash=MSO_LINE.DASH)
add_text(s, 2.62, 3.53, 1.55, 0.22, 'R = 0.1475 m', size=8, color=GRAY, bold=True)
add_circle(s, cx, cy, 0.10, fill=NAVY)
add_text(s, 0.75, 5.96, 5.6, 0.22,
         '25.5 cm 正方形框架 · 轮心外伸 2 cm · 轮径 10 cm · 直驱 1:1',
         size=7.8, color=GRAY)
add_text(s, 0.75, 6.22, 5.6, 0.22,
         '车头沿对角线：+X 与框架边成 45°；a = R/√2 = 0.1043 m。',
         size=7.8, color=LGRAY)

# formulas
add_formula(s, 6.78, 1.55, 5.97, 1.56,
            's_FL = (1/√2)(−vx + vy) + R·ω\n'
            's_FR = (1/√2)( vx + vy) + R·ω\n'
            's_RL = (1/√2)(−vx − vy) + R·ω\n'
            's_RR = (1/√2)( vx − vy) + R·ω\n'
            'RPM_i = s_i · 60/(2π·r) · gear',
            size=10.1)
add_formula(s, 6.78, 3.24, 5.97, 1.56,
            'M = [ −1/√2   +1/√2   R ]\n'
            '    [ +1/√2   +1/√2   R ]\n'
            '    [ −1/√2   −1/√2   R ]\n'
            '    [ +1/√2   −1/√2   R ]\n'
            'RPM = 190.99 · M · [vx  vy  ω]ᵀ',
            size=9.4, fill=RGBColor(0xEC,0xF4,0xFB))
add_text(s, 6.78, 4.88, 5.9, 0.22, '三个正交情形自检', size=9.2, color=BLUE, bold=True)
rows = [
    ['vx=1, vy=0, ω=0', '−0.707 / +0.707 / −0.707 / +0.707', '前后反向 → 前进'],
    ['vy=1, vx=0, ω=0', '+0.707 / +0.707 / −0.707 / −0.707', '前同后反 → 左移'],
    ['ω=1, vx=vy=0', '+R / +R / +R / +R', '四轮同向 → 自转'],
]
manual_table(s, 6.78, 5.14, 5.97, ['输入', '四轮轮缘速度 s (FL/FR/RL/RR)', '物理含义'], rows,
             [0.26,0.47,0.27], row_h=0.34, header_h=0.32, fs=7.8, header_fs=7.3)
add_text(s, 6.78, 6.58, 5.9, 0.22,
         '饱和：max|RPM| > 200 时四轮等比缩放；方向符号只在驱动层处理。',
         size=7.5, color=LGRAY)

# =====================================================================
# 10 单周期控制链
# =====================================================================
s = new_content('单周期控制链：从目标速度到电机电压',
                '20 MS LOOP  ·  HEADING → SLEW → IK → SATURATE → VOLTAGE',
                '已实现', GREEN, 10)
stages = [
    ('运动目标', 'vx, vy, ω', PALE, NAVY, 'main'),
    ('航向修正', 'PID + 前馈', RGBColor(0xE8,0xF1,0xFA), NAVY, 'heading'),
    ('ω 平滑', '20 rad/s²', RGBColor(0xE8,0xF1,0xFA), NAVY, 'chassis'),
    ('逆运动学', '4×3 矩阵', RGBColor(0xDC,0xE9,0xF5), NAVY, 'kinematics'),
    ('饱和缩放', '200 rpm 等比', RGBColor(0xDC,0xE9,0xF5), NAVY, 'drive'),
    ('斜率限制', '40 rpm / 20 ms', RGBColor(0xDC,0xE9,0xF5), NAVY, 'drive'),
    ('电压下发', '60 mV/rpm', BLUE, WHITE, 'drive'),
]
bw, gap, by, bh = 1.55, 0.16, 1.55, 0.95
for i,(name,detail,fill,tcol,src) in enumerate(stages):
    x = 0.76 + i*(bw+gap)
    add_text(s, x, 1.37, bw, 0.16, f'0{i+1}', size=7, color=LGRAY, bold=True, font='Arial')
    add_rect(s, x, by, bw, bh, fill=fill, line_color=(None if fill==BLUE else LIGHT), line_w=0.8,
             shape=MSO_SHAPE.ROUNDED_RECTANGLE, radius=0.08)
    add_text(s, x+0.10, by+0.16, bw-0.18, 0.26, name, size=9.2, color=tcol, bold=True)
    add_text(s, x+0.10, by+0.48, bw-0.18, 0.34, detail, size=7.6, color=(RGBColor(0xDC,0xE9,0xF5) if fill==BLUE else GRAY), line=1.0)
    add_text(s, x, by+bh+0.06, bw, 0.16, src, size=6.8, color=LGRAY, font=MONO, align=PP_ALIGN.CENTER)
    if i < len(stages)-1:
        add_line(s, x+bw+0.02, by+bh/2, x+bw+gap-0.02, by+bh/2, color=MID, w=1.2, arrow=True)

code_box(s, 0.55, 2.86, 7.52, 2.06,
'''// modules/chassis.cpp   (dt = 20 ms)
double w = target_.omega;
if (enableHeadingHold && imuState_.valid)
    w = heading_.update(imuState_.rotationRad,
                        target_.omega,
                        imuState_.angularVelocityRadPerSec, dt);
w = slew(w, lastOmega_, 20.0 * dt);
bodyVelocityToWheelRpm(config_, target_.vx,
                       target_.vy, w, rpm);
drive_->applyWheelRpm(rpm, enableSlewLimit, applied);''',
         fs=8.4, title='chassis.cpp')
add_text(s, 8.38, 2.86, 4.3, 0.22, '关键常数', size=9, color=BLUE, bold=True)
metrics = [('控制周期','20 ms'), ('满速上限','200 rpm'), ('电压换算','60 mV/rpm'), ('速度分辨率','0.017 rpm')]
for i,(lab,val) in enumerate(metrics):
    xx = 8.38 + (i%2)*2.10
    yy = 3.14 + (i//2)*0.72
    add_rect(s, xx, yy, 2.00, 0.62, fill=PALE, line_color=LIGHT, line_w=0.8,
             shape=MSO_SHAPE.ROUNDED_RECTANGLE, radius=0.10)
    add_text(s, xx+0.10, yy+0.06, 1.80, 0.18, lab, size=7.2, color=LGRAY, bold=True)
    add_text(s, xx+0.10, yy+0.24, 1.80, 0.30, val, size=12.5, color=NAVY, bold=True)
add_callout(s, 8.38, 4.62, 4.30, 0.30,
            '1 rpm 量化约 2.03 °/s；0.37 rpm 会被 lround 抹成 0。',
            color=RED, fill=RGBColor(0xFB,0xEE,0xEC), size=7.9)

code_box(s, 0.55, 5.10, 6.10, 1.38,
'''cmd[i] = rpm[i] * commandSign[i];
if (maxAbs > 200) cmd[i] *= 200 / maxAbs;
cmd[i] = slew(cmd[i], lastCmd[i], 40);   // per cycle
move_voltage(lround(cmd[i] * 60));       // mV''',
         fs=8.2, title='chassis_drive.cpp')
add_text(s, 6.92, 5.08, 5.8, 0.22, '为什么从 move_velocity 换成 move_voltage', size=9.2, color=NAVY, bold=True)
add_paras(s, 6.92, 5.38, 5.82, 1.10, [
    {'runs':[{'text':'move_velocity 只接受整数 rpm','bold':True,'color':GRAY,'size':8.6},{'text':'：最小 1 rpm，形成 2.03 °/s 的航向死区。','color':GRAY,'size':8.6}], 'bullet':True, 'bullet_color':RED, 'after':3},
    {'runs':[{'text':'move_voltage 支持小数','bold':True,'color':GRAY,'size':8.6},{'text':'：60 mV/rpm，分辨率约 0.017 rpm。','color':GRAY,'size':8.6}], 'bullet':True, 'bullet_color':GREEN, 'after':3},
    {'runs':[{'text':'代价','bold':True,'color':GRAY,'size':8.6},{'text':'：开环、低速仍有电压死区；下一步加软件轮速 PI。','color':GRAY,'size':8.6}], 'bullet':True, 'bullet_color':ORANGE},
])

# =====================================================================
# 11 IMU 航向保持
# =====================================================================
s = new_content('IMU 航向保持：保持与主动旋转的切换',
                'HEADING LATCH  ·  PID + FRICTION FEEDFORWARD  ·  ACTIVE-ROTATION BYPASS',
                '已实现', GREEN, 11)
add_rect(s, 0.55, 1.55, 5.95, 5.06, fill=RGBColor(0xF8,0xFB,0xFE), line_color=LIGHT, line_w=0.8,
         shape=MSO_SHAPE.ROUNDED_RECTANGLE, radius=0.03)
add_text(s, 0.75, 1.64, 4.0, 0.22, '控制状态机', size=9.4, color=BLUE, bold=True)

def state_box(x,y,w,h,title,detail,fill=PALE,line_color=LIGHT,tcol=NAVY):
    add_rect(s,x,y,w,h,fill=fill,line_color=line_color,line_w=0.9,shape=MSO_SHAPE.ROUNDED_RECTANGLE,radius=0.08)
    add_text(s,x+0.14,y+0.08,w-0.24,0.22,title,size=8.8,color=tcol,bold=True)
    add_text(s,x+0.14,y+0.31,w-0.24,h-0.36,detail,size=7.7,color=GRAY,line=1.10)

state_box(0.88,1.95,4.90,0.58,'① 上电 / 校准（非阻塞）','端口 6；reset(false) → is_calibrating() 轮询；3 s 超时')
add_line(s,3.33,2.53,3.33,2.75,color=MID,w=1.2,arrow=True)
state_box(0.88,2.75,4.90,0.58,'② 等待 5 s → 锁存目标 θ*','上电 5 s 后读取 rotationRad，作为唯一保持目标')
add_line(s,3.33,3.33,3.33,3.55,color=MID,w=1.2,arrow=True)
# decision
add_rect(s,1.55,3.55,3.55,0.55,fill=RGBColor(0xE8,0xF1,0xFA),line_color=MID,line_w=1.0,
         shape=MSO_SHAPE.DIAMOND)
add_text(s,1.55,3.68,3.55,0.28,'|ω_ext| 接近 0 ?',size=9.2,color=NAVY,bold=True,align=PP_ALIGN.CENTER)
# branches
state_box(0.88,4.38,2.42,1.08,'是：航向保持','PID + 静摩擦前馈\nω_cmd = ω_ext + corr + ff',fill=RGBColor(0xE8,0xF1,0xFA),line_color=MID)
state_box(3.52,4.38,2.42,1.08,'否：主动旋转','旁路 PID；目标跟随当前朝向\n松手即重锁，不转回旧目标',fill=RGBColor(0xFF,0xF4,0xE6),line_color=ORANGE)
add_line(s,2.60,4.10,2.10,4.38,color=MID,w=1.2,arrow=True)
add_line(s,4.05,4.10,4.55,4.38,color=ORANGE,w=1.2,arrow=True)
state_box(0.88,5.65,4.90,0.58,'IMU 无效 → 自动旁路','不阻塞底盘；恢复后重新锁存目标',fill=RGBColor(0xFB,0xEE,0xEC),line_color=RED,tcol=RED)
add_line(s,3.33,5.48,3.33,5.65,color=RED,w=1.1,dash=MSO_LINE.DASH,arrow=True)
add_text(s,0.88,6.33,4.9,0.20,'每个 20 ms 周期重新判断；状态切换不产生 ω 跳变。',size=7.4,color=LGRAY)

add_formula(s, 6.72, 1.55, 6.03, 1.10,
            'e = wrapToPi(θ* − θ)\n'
            'ω_cmd = ω_ext + clamp(Kp·e + Ki∫e + Kd·ė, ±2.0) + ff',
            size=9.7)
rows = [
    ['Kp','5.5','比例：主修正'],
    ['Ki','0.0','积分：暂不启用'],
    ['Kd','1.0','微分：抑制超调'],
    ['限幅','2.0 rad/s','单项 10 / 5 / 5'],
]
manual_table(s, 6.72, 2.80, 6.03, ['参数','当前值','作用'], rows,
             [0.20,0.24,0.56], row_h=0.31, header_h=0.32, fs=8.0, header_fs=7.6)
add_rect(s, 6.72, 4.42, 6.03, 0.86, fill=RGBColor(0xFF,0xF7,0xEC), line_color=None,
         shape=MSO_SHAPE.ROUNDED_RECTANGLE, radius=0.06)
add_rect(s, 6.72, 4.42, 0.055, 0.86, fill=ORANGE)
add_text(s, 6.90, 4.50, 2.4, 0.20, '方向相关静摩擦前馈 ff', size=8.4, color=ORANGE, bold=True)
add_text(s, 6.90, 4.72, 5.7, 0.48,
         '+0.361 rad/s（逆时针）  |  −0.221 rad/s（顺时针）\n条件：|gz| ≤ 0.05 rad/s 且 |e| > 0.0087 rad（约0.5°）',
         size=7.4, color=GRAY, line=1.10)
add_text(s, 6.72, 5.42, 5.9, 0.20, '三个设计点', size=8.8, color=BLUE, bold=True)
add_paras(s, 6.72, 5.66, 6.00, 0.95, [
    {'runs':[{'text':'主动旋转旁路','bold':True,'color':NAVY,'size':8.4},{'text':'：目标跟随当前朝向，松手不回转。','color':GRAY,'size':8.4}], 'bullet':True, 'after':2},
    {'runs':[{'text':'每项独立限幅','bold':True,'color':NAVY,'size':8.4},{'text':'：P/I/D 与总输出四级 clamp。','color':GRAY,'size':8.4}], 'bullet':True, 'after':2},
    {'runs':[{'text':'失效自动旁路','bold':True,'color':NAVY,'size':8.4},{'text':'：IMU 无效时只失去保持，不失底盘。','color':GRAY,'size':8.4}], 'bullet':True},
])
add_text(s, 6.72, 6.38, 6.0, 0.20, '安装偏航 +π/4：车头 = IMU +x 向 +y 偏 45°，gyro.z 不受影响。', size=7.2, color=GRAY)
add_text(s, 6.72, 6.62, 6.0, 0.20, '注：增益为当前配置值，仍需实车整定。', size=7.2, color=LGRAY)

# =====================================================================
# 12 实车调试复盘
# =====================================================================
s = new_content('实车调试复盘：三个“看不见”的坑',
                'IMU READINESS  ·  DOUBLE DRIFT COMPENSATION  ·  RPM QUANTIZATION',
                '已修复', GREEN, 12)
cards = [
    ('01','IMU 就绪判定：状态字不可信',
     '校准完成后 get_status() 仍返回 18，代码误判“未就绪”。',
     'PROS 4.2.2 的状态字与文档枚举不一致：ready=0 / calibrating=19 并不成立。',
     '改用 is_calibrating() 判定；加 3 s 校准超时，失败则 valid=false。', RED),
    ('02','漂移二次补偿：越补越漂',
     '静置 43 s 后 rot 缓慢漂移；开启角度补偿后出现“无输入自转”。',
     'get_rotation() 已做过零偏积分补偿，再扣一次是“二次补偿”，凭空制造漂移。',
     '关闭角度补偿（kApplyRotationDriftComp=false）；零偏仅用于诊断与角速度上报。', ORANGE),
    ('03','轮速量化：0.37 rpm 被抹成 0',
     '航向误差 1.26°，PID 输出 −13 mrad/s，但四轮命令全是 0。',
     'move_velocity 只接受整数 rpm，0.37 rpm 被 lround 舍入为 0。',
     '改 move_voltage，60 mV/rpm；分辨率 0.017 rpm，消掉量化死区。', GREEN),
]
for i,(num,title,sym,cause,fix,color) in enumerate(cards):
    yy = 1.55 + i*1.47
    add_rect(s, 0.55, yy, 12.20, 1.30, fill=WHITE, line_color=LIGHT, line_w=0.8,
             shape=MSO_SHAPE.ROUNDED_RECTANGLE, radius=0.05)
    add_rect(s, 0.55, yy, 0.06, 1.30, fill=color)
    add_circle(s, 0.92, yy+0.34, 0.34, fill=color, text=num, size=7.8)
    add_text(s, 1.22, yy+0.14, 2.55, 0.46, title, size=9.6, color=NAVY, bold=True, line=1.05)
    add_text(s, 1.22, yy+0.72, 2.45, 0.42, '现象 → 根因 → 修复', size=7.6, color=LGRAY)
    for j,(lab,txt,lc) in enumerate([('现象',sym,RED),('根因',cause,ORANGE),('修复',fix,GREEN)]):
        xx = 3.95 + j*2.95
        add_text(s, xx, yy+0.13, 2.6, 0.20, lab, size=7.2, color=lc, bold=True)
        add_text(s, xx, yy+0.38, 2.75, 0.82, txt, size=8.0, color=GRAY, line=1.12)
        if j < 2:
            add_line(s, xx+2.83, yy+0.14, xx+2.83, yy+1.16, color=RGBColor(0xE1,0xE8,0xF1), w=0.8)
add_callout(s, 0.55, 5.98, 12.20, 0.42,
            '调试方法：把 err / P / I / D / corr / ω / 轮命令 / 轮实测全部回传——先定量，再调参。',
            color=MID, fill=RGBColor(0xEC,0xF4,0xFB), size=8.8, bold=True)
add_text(s, 0.55, 6.78, 12.2, 0.18, '记录：docs/debug_log/02_IMU就绪判定 · 04_静置漂移补偿 · 06_轮速量化死区', size=6.8, color=LGRAY)
facts = ['0.013 rad/s → 0.366 rpm → lround = 0','1 rpm → 2.03 °/s','编码器 0.2° / tick']
for i,t in enumerate(facts):
    add_text(s, 0.55 + i*4.10, 6.55, 3.95, 0.24, '● ' + t, size=7.6, color=GRAY, align=PP_ALIGN.CENTER)


# =====================================================================
# 14 定位问题：为什么需要外部观测
# =====================================================================
s = new_content('定位问题：里程计为什么会漂',
                'DEAD RECKONING  ·  SLIP  ·  CALIBRATION  ·  GYRO BIAS',
                '方案设计', ORANGE, 14)
# chart panel
add_rect(s, 0.55, 1.55, 5.95, 4.92, fill=RGBColor(0xF8,0xFB,0xFE), line_color=LIGHT, line_w=0.8,
         shape=MSO_SHAPE.ROUNDED_RECTANGLE, radius=0.03)
add_text(s, 0.75, 1.65, 4.8, 0.24, '位置误差随时间增长（仿真示意）', size=9.4, color=BLUE, bold=True)
px0, px1, py0, py1 = 1.28, 5.98, 2.12, 5.42
for t in range(0,61,10):
    xx = px0 + t/60*(px1-px0)
    add_line(s, xx, py0, xx, py1, color=RGBColor(0xE4,0xEC,0xF4), w=0.6)
    add_text(s, xx-0.18, py1+0.05, 0.36, 0.20, str(t), size=7.4, color=GRAY, align=PP_ALIGN.CENTER)
for e in range(0,26,5):
    yy = py1 - e/25*(py1-py0)
    add_line(s, px0, yy, px1, yy, color=RGBColor(0xE4,0xEC,0xF4), w=0.6)
    add_text(s, 0.96, yy-0.08, 0.30, 0.18, str(e), size=6.8, color=LGRAY, align=PP_ALIGN.RIGHT)
add_line(s, px0, py1, px1, py1, color=GRAY, w=1.0)
add_line(s, px0, py0, px0, py1, color=GRAY, w=1.0)
add_text(s, 5.24, py1+0.26, 0.80, 0.20, 't / s', size=7.2, color=GRAY)
add_text(s, px0, 1.88, 1.6, 0.20, '位置误差 / cm', size=7.2, color=GRAY)
series = [
    ('纯里程计', [0,1.5,4.0,7.5,12.0,17.0,23.0], RED),
    ('编码器 + IMU', [0,1.0,2.2,3.8,5.6,7.5,9.5], ORANGE),
    ('EKF + 四向测距', [0,0.5,0.8,1.0,1.2,1.35,1.5], GREEN),
]
for name, vals, col in series:
    pts = [(px0 + (i*10)/60*(px1-px0), py1 - v/25*(py1-py0)) for i,v in enumerate(vals)]
    for i in range(len(pts)-1):
        add_line(s, *pts[i], *pts[i+1], color=col, w=2.0)
    for (xx,yy) in pts:
        add_circle(s, xx, yy, 0.07, fill=col)
    ex, ey = pts[-1]
    add_text(s, ex+0.06, ey-0.10, 0.65, 0.18, f'{vals[-1]:g}', size=6.8, color=col, bold=True)
# legend
lx = px0
for name, vals, col in series:
    add_line(s, lx, 5.78, lx+0.22, 5.78, color=col, w=2.3)
    add_text(s, lx+0.28, 5.69, 1.25, 0.20, name, size=6.9, color=GRAY)
    lx += 1.58
add_text(s, px0, 6.12, 4.8, 0.20, '仅用于说明误差趋势；数值为方案仿真，不是实车数据。', size=6.9, color=LGRAY)

add_text(s, 6.78, 1.58, 4.5, 0.24, '误差从哪来', size=9.6, color=BLUE, bold=True)
errs = [
    ('01','侧向滑移','全向轮被动滚子摩擦低，横向速度与航向都可能失真。'),
    ('02','轮径 / 轮距标定','比例误差随时间累积，直线也会慢慢走偏。'),
    ('03','IMU 零偏 / 漂移','角速度积分让 θ 缓慢偏，距离越远横向误差越大。'),
    ('04','碰撞 / 抓取扰动','位姿跳变；滤波器必须识别异常，而不是照单全收。'),
]
for i,(num,h,t) in enumerate(errs):
    yy = 1.92 + i*0.83
    add_rect(s, 6.78, yy, 5.92, 0.70, fill=WHITE, line_color=LIGHT, line_w=0.8,
             shape=MSO_SHAPE.ROUNDED_RECTANGLE, radius=0.07)
    add_circle(s, 7.04, yy+0.24, 0.26, fill=(RED if i==0 else ORANGE if i<=2 else TEAL), text=num, size=6.5)
    add_text(s, 7.26, yy+0.08, 1.8, 0.22, h, size=8.8, color=NAVY, bold=True)
    add_text(s, 7.26, yy+0.32, 5.20, 0.34, t, size=8.0, color=GRAY, line=1.05)
add_formula(s, 6.78, 5.30, 5.92, 0.55,
            'u = M⁺·s,   M⁺ = (MᵀM)⁻¹Mᵀ\n4 轮冗余 → 最小二乘正运动学',
            size=7.9, fill=PALE)
add_callout(s, 6.78, 6.02, 5.92, 0.48,
            '结论：里程计负责短时平滑；绝对位置必须由外部观测约束。',
            color=GREEN, fill=RGBColor(0xEC,0xF6,0xF0), size=8.4, bold=True)

# =====================================================================
# 15 四向测距观测模型
# =====================================================================
s = new_content('四向测距观测模型：把墙变成约束',
                'RANGE  =  f(POSE, WALL)  ·  INCIDENCE  ·  VALIDITY  ·  DATA ASSOCIATION',
                '方案设计', ORANGE, 15)
add_rect(s, 0.55, 1.55, 5.92, 4.95, fill=RGBColor(0xF8,0xFB,0xFE), line_color=LIGHT, line_w=0.8,
         shape=MSO_SHAPE.ROUNDED_RECTANGLE, radius=0.03)
add_text(s, 0.75, 1.65, 4.5, 0.22, '单元格内的四向观测', size=9.4, color=BLUE, bold=True)
cx, cy = 2.95, 3.72
cell = 3.55
add_rect(s, cx-cell/2, cy-cell/2, cell, cell, fill=WHITE, line_color=NAVY, line_w=2.2)
# rays to walls at 15 deg heading
import math
th = math.radians(15)
dirs = {'dF':(math.sin(th),-math.cos(th)), 'dB':(-math.sin(th),math.cos(th)),
        'dL':(-math.cos(th),-math.sin(th)), 'dR':(math.cos(th),math.sin(th))}
ends = {'dF':(3.42,cy-cell/2), 'dB':(2.48,cy+cell/2), 'dL':(cx-cell/2,3.25), 'dR':(cx+cell/2,4.19)}
for k,(dx,dy) in dirs.items():
    ex,ey = ends[k]
    add_line(s, cx, cy, ex, ey, color=MID, w=1.4, dash=MSO_LINE.DASH, arrow=True)
# robot
robot = add_rect(s, cx-0.40, cy-0.40, 0.80, 0.80, fill=RGBColor(0xDC,0xE9,0xF5), line_color=NAVY, line_w=1.2,
                 shape=MSO_SHAPE.ROUNDED_RECTANGLE, radius=0.08)
robot.rotation = 15
add_line(s, cx, cy, cx+0.15, cy-0.62, color=NAVY, w=1.6, arrow=True)
add_line(s, cx, cy, cx, 3.18, color=GRAY, w=0.8, dash=MSO_LINE.DASH)
add_text(s, cx+0.05, 3.31, 0.3, 0.2, 'θ', size=8, color=GRAY, bold=True, italic=True)
# labels
add_text(s, 3.48, 2.68, 0.55, 0.22, 'dF', size=8.2, color=MID, bold=True)
add_text(s, 2.10, 4.62, 0.55, 0.22, 'dB', size=8.2, color=MID, bold=True)
add_text(s, 1.72, 3.32, 0.55, 0.22, 'dL', size=8.2, color=MID, bold=True)
add_text(s, 4.06, 3.98, 0.55, 0.22, 'dR', size=8.2, color=MID, bold=True)
for txt,xx,yy in [('前 7',3.62,1.72),('后 8',2.12,5.58),('左 9',1.00,2.98),('右 10',4.35,4.20)]:
    add_text(s, xx, yy, 0.65, 0.18, txt, size=6.8, color=LGRAY, bold=True)
# dimension
add_line(s, cx-cell/2, cy+cell/2+0.24, cx+cell/2, cy+cell/2+0.24, color=GRAY, w=0.9, arrow=True, head=True)
add_text(s, cx-0.55, cy+cell/2+0.30, 1.1, 0.20, '45 cm', size=7.2, color=GRAY, align=PP_ALIGN.CENTER)
add_text(s, 0.75, 6.12, 5.5, 0.22, '四路距离同时约束 x / y / θ；墙角与隔板打破对称性。', size=7.6, color=LGRAY)

add_formula(s, 6.78, 1.55, 5.92, 1.22,
            'p_i = p + R(θ)·r_i\n'
            'n_i = R(θ)·u_i\n'
            'ẑ_i = (c − n·p_i) / (n·n_i)',
            size=9.2)
add_text(s, 6.78, 2.94, 5.9, 0.22, '观测质量', size=9.2, color=BLUE, bold=True)
add_paras(s, 6.78, 3.20, 5.95, 1.10, [
    {'runs':[{'text':'有效范围','bold':True,'color':NAVY,'size':8.2},{'text':'：20–2000 mm；无效 / 无目标不参与更新。','color':GRAY,'size':8.2}], 'bullet':True, 'after':2},
    {'runs':[{'text':'入射角','bold':True,'color':NAVY,'size':8.2},{'text':'：α 越大噪声越大，σ ~ 1/cosα。','color':GRAY,'size':8.2}], 'bullet':True, 'after':2},
    {'runs':[{'text':'置信度','bold':True,'color':NAVY,'size':8.2},{'text':'：0–63 与物体大小 0–400 作为权重。','color':GRAY,'size':8.2}], 'bullet':True, 'after':2},
    {'runs':[{'text':'几何有效','bold':True,'color':NAVY,'size':8.2},{'text':'：射线必须落在墙段内，否则丢弃。','color':GRAY,'size':8.2}], 'bullet':True},
])
add_text(s, 6.78, 4.48, 5.9, 0.22, '数据关联', size=9.2, color=BLUE, bold=True)
assoc = [
    ('1','静态地图：找射线命中的最近墙段。'),
    ('2','预测距离 vs 实测：偏差大 → 候选隔板。'),
    ('3','连续 N 帧一致才改地图；单帧异常不更新。'),
]
for i,(n,t) in enumerate(assoc):
    yy = 4.76 + i*0.31
    add_circle(s, 6.94, yy+0.08, 0.18, fill=BLUE, text=n, size=6.2)
    add_text(s, 7.10, yy-0.02, 5.5, 0.24, t, size=8.0, color=GRAY)
# sensor chips
for i,(lab,port) in enumerate([('前',7),('后',8),('左',9),('右',10)]):
    xx = 6.78 + i*1.48
    add_rect(s, xx, 5.78, 1.38, 0.38, fill=PALE, line_color=LIGHT, line_w=0.7,
             shape=MSO_SHAPE.ROUNDED_RECTANGLE, radius=0.12)
    add_text(s, xx+0.08, 5.82, 0.45, 0.20, lab, size=7.0, color=GRAY, bold=True)
    add_text(s, xx+0.50, 5.81, 0.80, 0.24, str(port), size=9.0, color=NAVY, bold=True)
add_callout(s, 6.78, 6.28, 5.92, 0.38,
            '当前：DistanceArray 只读取与遥测；观测模型与融合待实现。',
            color=ORANGE, fill=RGBColor(0xFD,0xF4,0xE8), size=7.8, bold=True)

# =====================================================================
# 16 EKF 多传感器融合
# =====================================================================
s = new_content('EKF 多传感器融合：预测 + 观测',
                'STATE [x, y, θ]  ·  PREDICT WITH ODOMETRY  ·  UPDATE WITH 4-WAY RANGE',
                '待实现', ORANGE, 16)
# block diagram
blocks = [
    ('运动输入','4× 编码器 + IMU',0.55,1.62,1.90),
    ('预测','f(x,u), F, Q',2.72,1.62,1.55),
    ('先验','x⁻, P⁻',4.52,1.62,1.10),
    ('更新','K, x̂, P',5.88,1.62,1.55),
    ('后验','x̂, P',7.68,1.62,1.10),
    ('输出','Pose → 导航 / 规划',9.03,1.62,2.10),
]
for name,detail,x,y,w in blocks:
    add_rect(s,x,y,w,0.92,fill=(BLUE if name=='输出' else PALE),line_color=(None if name=='输出' else LIGHT),line_w=0.8,
             shape=MSO_SHAPE.ROUNDED_RECTANGLE,radius=0.08)
    add_text(s,x+0.12,y+0.14,w-0.22,0.24,name,size=8.8,color=(WHITE if name=='输出' else NAVY),bold=True)
    add_text(s,x+0.12,y+0.44,w-0.22,0.36,detail,size=7.2,color=(RGBColor(0xDC,0xE9,0xF5) if name=='输出' else GRAY),line=1.0)
    if name != '输出':
        add_line(s,x+w+0.02,y+0.46,x+w+0.14,y+0.46,color=MID,w=1.2,arrow=True)
add_rect(s,5.88,2.92,1.55,0.68,fill=RGBColor(0xE6,0xF2,0xF2),line_color=TEAL,line_w=0.9,
         shape=MSO_SHAPE.ROUNDED_RECTANGLE,radius=0.08)
add_text(s,6.00,3.02,1.35,0.22,'观测',size=8.8,color=TEAL,bold=True)
add_text(s,6.00,3.25,1.35,0.24,'4× ToF + 地图',size=7.2,color=GRAY)
add_line(s,6.65,2.92,6.65,2.54,color=TEAL,w=1.3,arrow=True)
add_text(s,6.72,2.64,0.8,0.18,'z, H, R',size=6.8,color=TEAL,font=MONO)
add_text(s, 0.55, 3.57, 1.2, 0.18, '预测（20 ms）', size=8.0, color=BLUE, bold=True)
add_text(s, 4.63, 3.57, 1.7, 0.18, '观测（ToF 20 Hz）', size=8.0, color=BLUE, bold=True)
add_text(s, 8.71, 3.57, 1.2, 0.18, '更新（EKF）', size=8.0, color=BLUE, bold=True)
# equations
add_formula(s, 0.55, 3.78, 3.95, 1.72,
'''x⁻ = x + (vx cosθ − vy sinθ)Δt
y⁻ = y + (vx sinθ + vy cosθ)Δt
θ⁻ = θ + ωΔt
P⁻ = F P Fᵀ + Q''', size=8.6, fill=PALE)
add_formula(s, 4.63, 3.78, 3.95, 1.72,
'''z_i = h_i(x) + v_i
h_i = (c − n·p_i)/(n·n_i)
p_i = p + R(θ)·r_i
H_i = ∂h_i/∂x''', size=8.6, fill=PALE)
add_formula(s, 8.71, 3.78, 3.95, 1.72,
'''S = H P⁻ Hᵀ + R
K = P⁻ Hᵀ S⁻¹
x̂ = x⁻ + K(z − h(x⁻))
P = (I − K H) P⁻''', size=8.6, fill=PALE)
# bottom cards
cards = [
    ('3 状态 EKF', 'x = [x, y, θ]ᵀ；陀螺零偏启动时标定，残余进 Q；4 轮速度用 M⁺ 最小二乘。'),
    ('为什么用 EKF', '距离观测非线性、噪声随入射角变化；门控抑制异常值；协方差可交给规划。'),
    ('实现边界', '新增 localization/ekf.cpp；输出 Pose{x,y,θ,σ}；不进入 Chassis，只进导航层。'),
]
for i,(h,t) in enumerate(cards):
    x = 0.55 + i*4.08
    add_rect(s,x,5.66,3.90,1.02,fill=WHITE,line_color=LIGHT,line_w=0.8,shape=MSO_SHAPE.ROUNDED_RECTANGLE,radius=0.06)
    add_rect(s,x,5.66,0.05,1.02,fill=(MID if i<2 else TEAL))
    add_text(s,x+0.18,5.76,3.55,0.22,h,size=8.8,color=NAVY,bold=True)
    add_text(s,x+0.18,6.02,3.58,0.56,t,size=7.5,color=GRAY,line=1.12)

# =====================================================================
# 17 初始化、地图匹配与失效降级
# =====================================================================
s = new_content('初始化、地图匹配与失效降级',
                'SYMMETRY  ·  GRID SEARCH  ·  INNOVATION GATING  ·  SAFE DEGRADATION',
                '方案设计', ORANGE, 17)
add_rect(s, 0.55, 1.55, 5.85, 2.95, fill=RGBColor(0xF8,0xFB,0xFE), line_color=LIGHT, line_w=0.8,
         shape=MSO_SHAPE.ROUNDED_RECTANGLE, radius=0.03)
add_text(s, 0.75, 1.63, 4.5, 0.22, '对称性：为什么不能只看四路距离', size=9.2, color=BLUE, bold=True)
# symmetry cell
sx, sy, sc = 2.85, 3.02, 2.45
add_rect(s, sx-sc/2, sy-sc/2, sc, sc, fill=WHITE, line_color=NAVY, line_w=2.0)
for ex,ey in [(sx,sy-sc/2),(sx,sy+sc/2),(sx-sc/2,sy),(sx+sc/2,sy)]:
    add_line(s, sx, sy, ex, ey, color=MID, w=0.9, dash=MSO_LINE.DASH, arrow=True)
for lx,ly in [(sx-0.15,sy-0.70),(sx-0.15,sy+0.62),(sx-0.62,sy-0.10),(sx+0.44,sy-0.10)]:
    add_text(s, lx, ly, 0.30, 0.18, 'd', size=7.2, color=MID, bold=True)
add_rect(s, sx-0.27, sy-0.27, 0.54, 0.54, fill=RGBColor(0xDC,0xE9,0xF5), line_color=NAVY, line_w=1.0,
         shape=MSO_SHAPE.ROUNDED_RECTANGLE, radius=0.08).rotation = 12
add_text(s, sx-0.45, sy+0.30, 0.90, 0.20, '4 重对称', size=7.2, color=ORANGE, bold=True, align=PP_ALIGN.CENTER)
add_text(s, 0.75, 4.32, 5.5, 0.20, '空单元格中旋转 90° 后四向距离完全相同 → θ 存在 4 重歧义。', size=7.4, color=GRAY)
# init flow
add_text(s, 0.55, 4.62, 4.0, 0.22, '初始化流程', size=9.2, color=BLUE, bold=True)
init = [
    ('1','已知起点 S：x0 = S 中心，θ0 = 场地朝向，P0 由墙距给定。'),
    ('2','未知起点：网格搜索 (x,y,θ)，最小化 Σρ(||z − h||)。'),
    ('3','取 top-k 假设，按残差初始化协方差。'),
    ('4','首次运动后用里程计 + 测距消除镜像 / 旋转歧义。'),
]
for i,(n,t) in enumerate(init):
    yy = 4.92 + i*0.38
    add_circle(s, 0.72, yy+0.09, 0.20, fill=BLUE, text=n, size=6.4)
    add_text(s, 0.90, yy-0.02, 5.45, 0.28, t, size=7.8, color=GRAY)
# robustness table
rows = [
    ['单侧无效 / 无目标','跳过该路，R 膨胀','其余三路继续'],
    ['低置信度 / 大入射角','σ = σ0 / cosα，降权','不污染位姿'],
    ['隔板状态变化','连续 N 帧创新超门限 → 改地图','触发重规划'],
    ['打滑 / 碰撞','协方差膨胀 + 重新捕获','避免滤波发散'],
    ['长时间失效','停车扫墙 / 回到已知点','安全降级'],
]
manual_table(s, 6.65, 1.60, 6.10, ['场景','策略','结果'], rows,
             [0.28,0.42,0.30], row_h=0.40, header_h=0.38, fs=7.7, header_fs=7.2)
code_box(s, 6.65, 4.20, 6.10, 1.42,
'''nu = z - h(x_pred);
S  = H * P * H.transpose() + R;
d2 = nu.transpose() * S.inverse() * nu;
if (d2 < chi2_gate) ekf.update(z, R);
else if (++outlier > N) map.mark(edge, BLOCKED);''',
         fs=8.0, title='gating + map update')
add_text(s, 6.65, 5.78, 6.10, 0.22, '单帧异常不更新；连续 N 帧一致才改地图。', size=7.6, color=GRAY, bold=True)
add_text(s, 6.65, 6.06, 6.10, 0.44, '失效降级：停车扫墙 → 重定位 → 恢复任务；定位协方差过大时不盲目继续。', size=7.4, color=LGRAY, line=1.10)


# =====================================================================
# 19 从定位到任务：地图更新与动态规划
# =====================================================================
s = new_content('从定位到任务：地图更新与动态规划',
                'DIJKSTRA  ·  3! TASK ORDER DP  ·  REPLAN ON PARTITION CHANGE',
                '待实现', ORANGE, 19)
add_text(s, 0.55, 1.58, 4.5, 0.24, '地图 → 加权图', size=9.4, color=BLUE, bold=True)
add_text(s, 0.55, 1.88, 5.95, 0.44,
         '64 个单元格节点；相邻格有边。边权由距离、转弯、抓取与未知风险共同决定。',
         size=8.2, color=GRAY, line=1.12)
add_formula(s, 0.55, 2.38, 5.95, 0.96,
            'cost(u→v) = d/vmax + w_turn·|Δθ|\n'
            '          + w_grab·[v∈{A,B,C}]\n'
            '          + w_risk·P(blocked)',
            size=8.3)
add_text(s, 0.55, 3.48, 5.9, 0.22, 'Dijkstra：任意两点最短路；3 个目标 → 3! = 6 种访问顺序。', size=8.0, color=GRAY)
add_formula(s, 0.55, 3.78, 5.95, 1.18,
            'dp[1<<j][j] = d(S, j)\n'
            'dp[mask][j] = min_i (dp[mask−{j}][i] + d(i,j))\n'
            'answer     = min_i (dp[111][i] + d(i,F))',
            size=8.2, fill=RGBColor(0xEC,0xF4,0xFB))
add_text(s, 0.55, 5.08, 5.9, 0.20, 'mask 为 {A,B,C} 的子集；8 个状态 × 3 个终点，远小于全排列搜索。', size=7.6, color=LGRAY)
add_text(s, 0.55, 5.40, 5.9, 0.22, '重规划触发', size=8.8, color=BLUE, bold=True)
triggers = ['隔板变化','定位协方差大','任务超时','抓取失败']
for i,t in enumerate(triggers):
    xx = 0.55 + i*1.50
    add_rect(s, xx, 5.68, 1.38, 0.34, fill=RGBColor(0xFD,0xF4,0xE8), line_color=None,
             shape=MSO_SHAPE.ROUNDED_RECTANGLE, radius=0.15)
    add_text(s, xx, 5.72, 1.38, 0.26, t, size=7.2, color=ORANGE, bold=True, align=PP_ALIGN.CENTER)
add_callout(s, 0.55, 6.18, 5.95, 0.42,
            '规划层只输出目标点 / 路径；Chassis 只接收 MotionCommand。',
            color=MID, fill=RGBColor(0xEC,0xF4,0xFB), size=7.8, bold=True)

add_rect(s, 6.75, 1.55, 5.95, 5.02, fill=RGBColor(0xF8,0xFB,0xFE), line_color=LIGHT, line_w=0.8,
         shape=MSO_SHAPE.ROUNDED_RECTANGLE, radius=0.03)
add_text(s, 6.95, 1.63, 3.5, 0.22, '任务状态机', size=9.4, color=BLUE, bold=True)
# state boxes
def stbox(x,y,label,fill=PALE,tcol=NAVY,line=LIGHT):
    add_rect(s,x,y,1.20,0.55,fill=fill,line_color=line,line_w=0.8,shape=MSO_SHAPE.ROUNDED_RECTANGLE,radius=0.10)
    add_text(s,x,y+0.15,1.20,0.24,label,size=7.4,color=tcol,bold=True,align=PP_ALIGN.CENTER)
top = [('LOCALIZE',7.00),('PLAN',8.40),('NAVIGATE',9.80),('ALIGN',11.20)]
for lab,x in top:
    stbox(x,2.02,lab,fill=(RGBColor(0xDC,0xE9,0xF5) if lab=='PLAN' else PALE))
    if lab!='ALIGN': add_line(s,x+1.22,2.29,x+1.38,2.29,color=MID,w=1.0,arrow=True)
bot = [('GRAB',11.20),('DELIVER',9.80),('NEXT',8.40),('DONE',7.00)]
for lab,x in bot:
    stbox(x,3.22,lab,fill=(ORANGE if lab=='DONE' else PALE),tcol=(WHITE if lab=='DONE' else NAVY))
add_line(s,11.80,2.57,11.80,3.22,color=MID,w=1.0,arrow=True)
add_line(s,11.18,3.49,11.02,3.49,color=MID,w=1.0,arrow=True)
add_line(s,9.78,3.49,9.62,3.49,color=MID,w=1.0,arrow=True)
# loop next -> plan
add_line(s,9.00,3.22,9.00,2.57,color=TEAL,w=1.0,dash=MSO_LINE.DASH,arrow=True)
add_text(s,9.06,2.78,0.85,0.16,'下一目标',size=6.4,color=TEAL)
# next -> done
add_line(s,8.38,3.49,8.22,3.49,color=ORANGE,w=1.0,dash=MSO_LINE.DASH,arrow=True)
add_text(s,7.30,3.80,1.05,0.16,'全部完成',size=6.4,color=ORANGE,align=PP_ALIGN.CENTER)
# failure branches
add_text(s, 6.95, 4.10, 3.0, 0.20, '失败分支', size=8.4, color=BLUE, bold=True)
fails = [('定位丢失','停车重定位'),('抓取失败','重试 / 跳过'),('超时','放弃当前目标')]
for i,(a,b) in enumerate(fails):
    xx = 6.95 + i*1.92
    add_rect(s, xx, 4.38, 1.78, 0.62, fill=WHITE, line_color=LIGHT, line_w=0.8,
             shape=MSO_SHAPE.ROUNDED_RECTANGLE, radius=0.08)
    add_text(s, xx+0.10, 4.44, 1.58, 0.18, a, size=7.2, color=RED, bold=True)
    add_text(s, xx+0.10, 4.64, 1.58, 0.20, b, size=7.0, color=GRAY)
add_text(s, 6.95, 5.18, 5.5, 0.22, '每个状态都有超时；定位协方差过大时不盲目继续。', size=7.4, color=GRAY)
add_text(s, 6.95, 5.48, 5.55, 0.90,
         '规划结果缓存：地图变化时只重算受影响的边；任务顺序 DP 只有 8×3 个状态，可在 20 ms 空闲时完成。',
         size=7.4, color=LGRAY, line=1.16)

# =====================================================================
# 20 验证计划与下一步
# =====================================================================
s = new_content('融合进度与下一步',
                'STATUS  ·  ACCEPTANCE TARGETS  ·  ROADMAP',
                '验证中', ORANGE, 20)
rows = [
    ['驱动 / 航向 / 输入单测','主机单测','● 通过'],
    ['程序编译 / 下载','编译与下载','● 通过'],
    ['四路 ToF 读取','20 Hz 遥测','● 通过'],
    ['静态航向保持','实车 60 s 记录','● 进行中'],
    ['静态定位重复性','20 点 × 30 s','● 待做'],
    ['动态定位','方形 / 8 字轨迹','● 待做'],
    ['迷宫全程','3 物块，≤ 180 s','● 待做'],
]
manual_table(s, 0.55, 1.60, 7.10, ['验证项','方法','状态'], rows,
             [0.28,0.44,0.28], row_h=0.40, header_h=0.38, fs=7.6, status_col=2)
add_callout(s, 0.55, 4.86, 7.10, 0.64,
            '当前边界：底盘 + IMU 航向 + 四向测距读取已落地；EKF、地图、规划、抓取为下一阶段。',
            color=ORANGE, fill=RGBColor(0xFD,0xF4,0xE8), size=8.0, bold=True)
add_text(s, 0.55, 5.64, 7.10, 0.36,
         '测试通过不等于实车正确：理论、单测、编译、实车四层分别验收。',
         size=7.4, color=LGRAY)

add_text(s, 7.95, 1.58, 4.7, 0.22, '建议验收指标（待实测）', size=8.8, color=BLUE, bold=True)
metrics = [('静态定位','≤ 10 mm / 1°'),('动态定位','≤ 20 mm / 2°'),('迷宫全程','≤ 180 s')]
for i,(lab,val) in enumerate(metrics):
    xx = 7.95 + i*1.62
    add_rect(s, xx, 1.86, 1.50, 0.66, fill=PALE, line_color=LIGHT, line_w=0.7,
             shape=MSO_SHAPE.ROUNDED_RECTANGLE, radius=0.10)
    add_text(s, xx+0.08, 1.91, 1.34, 0.18, lab, size=6.6, color=LGRAY, bold=True)
    add_text(s, xx+0.08, 2.11, 1.34, 0.32, val, size=9.4, color=NAVY, bold=True)
add_text(s, 7.95, 2.62, 4.8, 0.18, '建议值，最终以赛场实测与规则为准。', size=6.8, color=LGRAY)

add_text(s, 7.95, 2.95, 4.6, 0.22, '下一步路线', size=8.8, color=BLUE, bold=True)
steps = [
    ('01','标定','轮径 / 轮距 / 传感器安装 / IMU 零偏'),
    ('02','定位','静态最小二乘 → 3 状态 EKF'),
    ('03','导航','路径跟踪 / 到点判断 / 重定位'),
    ('04','任务','抓取 / 搬运 / 全程计时'),
]
for i,(n,h,t) in enumerate(steps):
    yy = 3.25 + i*0.64
    add_circle(s, 8.10, yy+0.16, 0.26, fill=BLUE, text=n, size=6.2)
    if i < len(steps)-1:
        add_line(s, 8.10, yy+0.30, 8.10, yy+0.60, color=LIGHT, w=1.4)
    add_text(s, 8.34, yy, 0.75, 0.22, h, size=8.6, color=NAVY, bold=True)
    add_text(s, 9.10, yy, 3.55, 0.24, t, size=7.6, color=GRAY)
# summary band
add_rect(s, 0.55, 6.12, 12.20, 0.62, fill=NAVY)
add_text(s, 0.78, 6.22, 4.0, 0.22, '已落地：底盘 + IMU 航向 + 四向测距读取', size=8.0, color=WHITE, bold=True)
add_text(s, 4.85, 6.22, 4.8, 0.22, '下一阶段：EKF + 地图 + 规划 + 抓取', size=8.0, color=RGBColor(0xDC,0xE9,0xF5))
add_text(s, 9.55, 6.18, 3.0, 0.28, '谢谢观看 · 欢迎讨论', size=11.5, color=WHITE, bold=True, align=PP_ALIGN.RIGHT)


# =====================================================================
# 机械 × 程序：接口对照与待确认项
# =====================================================================
s = new_content('机械 × 程序：接口对照与待确认项',
                'MECHANISM  ×  SOFTWARE  ·  PARAMETER MATCH  ·  SENSOR LAYOUT  ·  ARM INTERFACE',
                '3 项待确认', ORANGE, None)
rows = [
    ['底盘构型','四轮全向、四电机直驱、正方形框架','X-Drive IK；R=14.75 cm，r=5 cm，1:1','一致'],
    ['测距布置','“四角激光测距”','7=前、8=后、9=左、10=右；四向观测','需确认'],
    ['机械臂','单自由度；抓取位 / 存放位；机械限位','GRAB / DELIVER；缺 STOW 与到位反馈','待对齐'],
    ['进度','底盘制造完成；抓取机构 3D 完成','驱动 / IMU / ToF 读取已实现；EKF / 规划待做','机械先行'],
    ['电气接口','原稿未给限位开关与传感器安装角','电机 1–4，IMU 6，ToF 7–10','需核对'],
]
manual_table(s, 0.55, 1.60, 7.55, ['对比项','机械方案（原稿）','程序实现','结论'], rows,
             [0.15,0.31,0.37,0.17], row_h=0.60, header_h=0.42, fs=8.0, header_fs=7.8, status_col=3)
# left checklist
add_rect(s, 0.55, 5.32, 7.55, 0.84, fill=RGBColor(0xF7,0xFA,0xFD), line_color=LIGHT, line_w=0.7,
         shape=MSO_SHAPE.ROUNDED_RECTANGLE, radius=0.06)
add_text(s, 0.72, 5.40, 3.5, 0.20, '联调前必须闭环的 3 个接口', size=8.6, color=BLUE, bold=True)
checks = ['□ 四个测距传感器的安装位置与射线方向',
          '□ 机械臂到位开关 / 限位信号',
          '□ 抓取 / 存放两个姿态的标定时间']
for i,t in enumerate(checks):
    add_text(s, 0.72 + (i%2)*3.65, 5.64 + (i//2)*0.22, 3.55, 0.20, t, size=7.2, color=GRAY)
# right cards
cards = [
    ('① 四角 vs 四边：观测模型要按安装角标定',
     '机械原稿写“四角激光测距”，程序配置是前 / 后 / 左 / 右（7–10）。若传感器在角上且射线斜置，'
     '观测模型必须用实测安装位置与方向；否则墙距约束的方向会错。'),
    ('② 单自由度机械臂：状态机缺 STOW 与到位反馈',
     '机械只有抓取位 / 存放位两个限位姿态，程序状态机目前是 GRAB → DELIVER。'
     '需确认是否有到位开关 / 电流检测；若无，只能用时间 + 堵转保护。状态机应增加 STOW（收纳）。'),
]
for i,(h,t) in enumerate(cards):
    yy = 1.60 + i*2.10
    add_rect(s, 8.35, yy, 4.40, 1.92, fill=WHITE, line_color=LIGHT, line_w=0.8,
             shape=MSO_SHAPE.ROUNDED_RECTANGLE, radius=0.05)
    add_rect(s, 8.35, yy, 0.05, 1.92, fill=(ORANGE if i==0 else MID))
    add_text(s, 8.52, yy+0.12, 4.05, 0.38, h, size=9.2, color=NAVY, bold=True, line=1.05)
    add_text(s, 8.52, yy+0.52, 4.05, 1.30, t, size=7.6, color=GRAY, line=1.14)
add_callout(s, 0.55, 6.28, 12.20, 0.42,
            '结论：机械基线支持现有 X-Drive 运动学；先对齐“传感器安装几何”与“机械臂到位反馈”，再联调。',
            color=MID, fill=RGBColor(0xEC,0xF4,0xFB), size=8.4, bold=True)
add_text(s, 0.55, 6.78, 12.2, 0.18,
         '机械部分按原稿保留（页码 02/08–07/08）；程序部分仅作架构说明。',
         size=6.8, color=LGRAY)


# =====================================================================
# 中期进度、风险与下一步
# =====================================================================
s = new_content('中期进度、风险与下一步',
                'STATUS  ·  RISKS  ·  NEXT MILESTONES',
                '中期检查', ORANGE, None)
# left: three status columns
cols = [
    ('已完成', GREEN, ['底盘制造','机械臂 3D 设计','底盘驱动 / IMU 航向','四路 ToF 读取']),
    ('进行中', ORANGE, ['整车装配','测距安装与标定','航向 PID 实车整定']),
    ('待开始', LGRAY, ['定位融合','地图 / 路径规划','夹取 / 收纳联调','全程计时']),
]
for i,(head,col,items) in enumerate(cols):
    x = 0.55 + i*2.38
    add_rect(s, x, 1.58, 2.22, 0.36, fill=col, line_color=None,
             shape=MSO_SHAPE.ROUNDED_RECTANGLE, radius=0.12)
    add_text(s, x, 1.62, 2.22, 0.26, head, size=9.2, color=WHITE, bold=True, align=PP_ALIGN.CENTER)
    add_rect(s, x, 1.98, 2.22, 2.18, fill=RGBColor(0xF7,0xFA,0xFD), line_color=LIGHT, line_w=0.8,
             shape=MSO_SHAPE.ROUNDED_RECTANGLE, radius=0.06)
    for j,item in enumerate(items):
        yy = 2.10 + j*0.44
        add_circle(s, x+0.18, yy+0.09, 0.09, fill=col)
        add_text(s, x+0.30, yy-0.02, 1.82, 0.28, item, size=8.0, color=GRAY)
# right: risks
add_text(s, 7.78, 1.56, 4.8, 0.22, '关键风险与应对', size=9.4, color=BLUE, bold=True)
risks = [
    ('① 测距安装几何未定', '四角还是四边、射线是否斜置尚未锁定；装配后实测安装位置与方向，再更新观测模型。'),
    ('② 机械臂到位反馈未定', '原稿只有机械限位；需确认到位开关 / 电流检测，状态机补 STOW（收纳）。'),
    ('③ 联调窗口偏短', '先分功能验证底盘 / 测距 / 机械臂，再整机联调；不把问题留到最后。'),
]
for i,(h,t) in enumerate(risks):
    yy = 1.86 + i*1.02
    add_rect(s, 7.78, yy, 4.95, 0.90, fill=WHITE, line_color=LIGHT, line_w=0.8,
             shape=MSO_SHAPE.ROUNDED_RECTANGLE, radius=0.06)
    add_rect(s, 7.78, yy, 0.05, 0.90, fill=(ORANGE if i<2 else MID))
    add_text(s, 7.94, yy+0.10, 4.65, 0.22, h, size=8.4, color=NAVY, bold=True)
    add_text(s, 7.94, yy+0.35, 4.65, 0.50, t, size=7.4, color=GRAY, line=1.12)
# bottom: milestones
add_text(s, 0.55, 4.48, 4.0, 0.22, '下一阶段里程碑', size=9.4, color=BLUE, bold=True)
milestones = [
    ('本周内','整车装配','机械臂、夹爪、传感器与线束整体安装。'),
    ('下周开始','分功能调试','底盘运动、测距、机械臂动作与夹爪抓取。'),
    ('分功能后','整机联调','行驶—对准—抓取—收纳—转运流程衔接。'),
]
for i,(when,h,t) in enumerate(milestones):
    x = 0.55 + i*4.05
    add_rect(s, x, 4.78, 3.88, 0.98, fill=PALE, line_color=LIGHT, line_w=0.8,
             shape=MSO_SHAPE.ROUNDED_RECTANGLE, radius=0.06)
    add_text(s, x+0.16, 4.85, 3.55, 0.20, when, size=7.2, color=ORANGE, bold=True)
    add_text(s, x+0.16, 5.06, 1.4, 0.22, h, size=9.0, color=NAVY, bold=True)
    add_text(s, x+1.45, 5.07, 2.30, 0.52, t, size=7.2, color=GRAY, line=1.08)
    if i < 2:
        add_line(s, x+3.92, 5.27, x+4.02, 5.27, color=MID, w=1.5, arrow=True)
# closing band
add_rect(s, 0.55, 6.12, 12.20, 0.60, fill=NAVY)
add_text(s, 0.78, 6.22, 7.2, 0.22,
         '下一步：以整车装配为节点，先分功能验证，再整机联调。',
         size=8.4, color=WHITE, bold=True)
add_text(s, 8.55, 6.18, 4.0, 0.28, '谢谢观看 · 欢迎讨论', size=11.5, color=WHITE, bold=True, align=PP_ALIGN.RIGHT)

# ---------- reorder slides ----------
# remove the original thanks slide
sldIds = list(prs.slides._sldIdLst)
all_now = list(prs.slides)
for sid, sl in zip(sldIds, all_now):
    if sl is thanks_slide:
        prs.part.drop_rel(sid.rId)
        prs.slides._sldIdLst.remove(sid)

order = [cover, toc, sec1] + content_slides[0:4] + [sec2] + content_slides[4:8] + [sec3] + content_slides[8:12] + [sec4] + content_slides[12:14] + [content_slides[14]]
all_slides = list(prs.slides)
sldIds = list(prs.slides._sldIdLst)
idx = {id(s): i for i, s in enumerate(all_slides)}
ordered_ids = [sldIds[idx[id(s)]] for s in order]
for el in list(prs.slides._sldIdLst):
    prs.slides._sldIdLst.remove(el)
for el in ordered_ids:
    prs.slides._sldIdLst.append(el)

# unique shape ids
uid = 1
for sl in prs.slides:
    for el in sl.shapes._spTree.iter(qn('p:cNvPr')):
        el.set('id', str(uid)); uid += 1

cp = prs.core_properties
cp.title = 'VEX V5 全向迷宫机器人 · 程序实现'
cp.subject = 'X-Drive 底盘 · IMU 航向闭环 · 四向测距融合定位'
cp.author = 'VEX V5 迷宫机器人项目组'
cp.last_modified_by = 'VEX V5 迷宫机器人项目组'
cp.keywords = 'VEX V5; PROS; X-Drive; EKF; 四向测距; 迷宫机器人'
cp.comments = '基于 Car/ 工程的实际代码与实车调试记录；定位/规划为下一阶段方案。'
prs.save(OUT)
print('saved', OUT, 'slides', len(prs.slides))
