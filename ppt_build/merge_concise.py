import copy
from io import BytesIO
from pptx import Presentation
from pptx.oxml.ns import qn
from pptx.util import Inches

MECH = '/home/jc/下载/走迷宫任务小车——机械方案与阶段进展汇报(1).pptx'
PROG = '/home/jc/下载/VEX_V5_迷宫机器人_程序实现.pptx'
OUT = '/home/jc/下载/走迷宫任务小车——机械+程序_5分钟版.pptx'

base = Presentation(MECH)
prog = Presentation(PROG)
mech_slides = list(base.slides)

def copy_program_slide(src, dst_prs, layout, drop_footer=True):
    dest = dst_prs.slides.add_slide(layout)
    for sh in list(dest.shapes):
        sh._element.getparent().remove(sh._element)
    ridmap = {}
    for sh in src.shapes:
        # drop the programme footer line / page number in the 5-min edit
        if drop_footer and sh.top is not None and sh.top >= Inches(7.0):
            continue
        el = copy.deepcopy(sh._element)
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

# selected programme slides: cover, TOC, section 01, task, comparison, architecture, status
selected = [0, 1, 2, 3, 20, 5, 19]
prog_copies = [copy_program_slide(prog.slides[i], base, base.slide_layouts[0]) for i in selected]

order = [prog_copies[0], prog_copies[1], prog_copies[2], prog_copies[3],
         *mech_slides,
         prog_copies[4], prog_copies[5], prog_copies[6]]
assert len(order) == len(base.slides) == 14, (len(order), len(base.slides))

all_slides = list(base.slides)
sld_ids = list(base.slides._sldIdLst)
idx = {id(s): i for i, s in enumerate(all_slides)}
ordered_ids = [sld_ids[idx[id(s)]] for s in order]
for el in list(base.slides._sldIdLst):
    base.slides._sldIdLst.remove(el)
for el in ordered_ids:
    base.slides._sldIdLst.append(el)

cp = base.core_properties
cp.title = '走迷宫任务小车 · 机械 + 程序（5 分钟版）'
cp.subject = '机械方案与进展 + 程序架构与接口对比'
cp.author = '第八组'
cp.last_modified_by = '第八组'
cp.keywords = 'VEX V5; 走迷宫; 全向底盘; 机械臂; 程序架构'
cp.comments = '机械部分 7 张幻灯片按原稿迁移；程序部分只保留架构与接口对比。'

base.save(OUT)
print('saved', OUT, 'slides', len(base.slides))
