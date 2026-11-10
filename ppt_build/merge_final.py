import copy
from io import BytesIO
from pptx import Presentation
from pptx.oxml.ns import qn

MECH = '/home/jc/下载/走迷宫任务小车——机械方案与阶段进展汇报(1).pptx'
PROG = '/home/jc/下载/VEX_V5_迷宫机器人_程序实现.pptx'
OUT = '/home/jc/下载/走迷宫任务小车——机械+程序完整版.pptx'

base = Presentation(MECH)
prog = Presentation(PROG)
mech_slides = list(base.slides)          # 7 original slides, untouched
print('mechanical slides', len(mech_slides), 'program slides', len(prog.slides))

def copy_program_slide(src, dst_prs, layout):
    dest = dst_prs.slides.add_slide(layout)
    for sh in list(dest.shapes):
        sh._element.getparent().remove(sh._element)
    ridmap = {}
    for sh in src.shapes:
        el = copy.deepcopy(sh._element)
        # program slides use 微软雅黑; the mechanical package embeds a full MiSans
        for node in el.iter():
            for k, v in list(node.attrib.items()):
                if v == '微软雅黑':
                    node.set(k, 'MiSans')
        for blip in el.iter(qn('a:blip')):
            old = blip.get(qn('r:embed'))
            if old:
                if old not in ridmap:
                    rel = src.part.rels[old]
                    _, newrid = dest.part.get_or_add_image_part(BytesIO(rel.target_part.blob))
                    ridmap[old] = newrid
                blip.set(qn('r:embed'), ridmap[old])
        dest.shapes._spTree.insert_element_before(el, 'p:extLst')
    return dest

prog_copies = []
for src in prog.slides:
    prog_copies.append(copy_program_slide(src, base, base.slide_layouts[0]))

# Final order: combined cover/TOC/task/goals -> mechanical section (7 pages)
# -> comparison -> program architecture/code map -> chassis/localization/planning.
order = [
    prog_copies[0], prog_copies[1], prog_copies[2], prog_copies[3], prog_copies[4],
    *mech_slides,
    prog_copies[20],
    prog_copies[5], prog_copies[6], prog_copies[7], prog_copies[8], prog_copies[9],
    prog_copies[10], prog_copies[11], prog_copies[12], prog_copies[13], prog_copies[14],
    prog_copies[15], prog_copies[16], prog_copies[17], prog_copies[18], prog_copies[19],
]
assert len(order) == len(base.slides) == 28, (len(order), len(base.slides))

all_slides = list(base.slides)
sld_ids = list(base.slides._sldIdLst)
idx = {id(s): i for i, s in enumerate(all_slides)}
ordered_ids = [sld_ids[idx[id(s)]] for s in order]
for el in list(base.slides._sldIdLst):
    base.slides._sldIdLst.remove(el)
for el in ordered_ids:
    base.slides._sldIdLst.append(el)

cp = base.core_properties
cp.title = '走迷宫任务小车 · 机械 + 程序完整版'
cp.subject = '机械方案与阶段进展 + X-Drive 底盘 / IMU 航向 / 四向测距融合定位'
cp.author = '第八组'
cp.last_modified_by = '第八组'
cp.keywords = 'VEX V5; 走迷宫; 全向底盘; 机械臂; X-Drive; EKF'
cp.comments = '机械部分 7 页按原稿迁移；程序部分基于 Car/ 工程。'

base.save(OUT)
print('saved', OUT, 'slides', len(base.slides))
