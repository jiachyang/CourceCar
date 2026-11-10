import copy
from io import BytesIO
from pptx import Presentation
from pptx.oxml.ns import qn
from pptx.util import Inches

MECH = '/home/jc/下载/走迷宫任务小车——机械方案与阶段进展汇报(1).pptx'
PROG = '/home/jc/下载/VEX_V5_迷宫机器人_程序实现.pptx'
OUT = '/home/jc/下载/走迷宫任务小车——中期汇报_5min.pptx'

base = Presentation(MECH)
prog = Presentation(PROG)
mech = list(base.slides)

def copy_slide(src, dst_prs):
    dest = dst_prs.slides.add_slide(dst_prs.slide_layouts[0])
    for sh in list(dest.shapes):
        sh._element.getparent().remove(sh._element)
    ridmap = {}
    for sh in src.shapes:
        if sh.top is not None and sh.top >= Inches(7.0):
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

# cover, architecture, comparison, status
p_cover = copy_slide(prog.slides[0], base)
p_arch  = copy_slide(prog.slides[5], base)
p_comp  = copy_slide(prog.slides[20], base)
p_stat  = copy_slide(prog.slides[19], base)

order = [p_cover] + mech + [p_arch, p_comp, p_stat]
assert len(order) == len(base.slides) == 11, (len(order), len(base.slides))
all_slides = list(base.slides)
sld_ids = list(base.slides._sldIdLst)
idx = {id(s): i for i, s in enumerate(all_slides)}
ordered = [sld_ids[idx[id(s)]] for s in order]
for el in list(base.slides._sldIdLst):
    base.slides._sldIdLst.remove(el)
for el in ordered:
    base.slides._sldIdLst.append(el)

cp = base.core_properties
cp.title = '走迷宫任务小车 · 中期汇报（5 min）'
cp.subject = '机械方案与进展 + 程序架构与接口'
cp.author = '第八组'
cp.last_modified_by = '第八组'
cp.keywords = '走迷宫; 全向底盘; 机械臂; 中期汇报'
cp.comments = '机械部分 7 张幻灯片按原稿迁移；程序部分只保留架构、接口与进度。'

base.save(OUT)
print('saved', OUT, 'slides', len(base.slides))
