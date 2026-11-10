import copy
from io import BytesIO
from pptx import Presentation
from pptx.oxml.ns import qn

src=Presentation('/home/jc/下载/走迷宫任务小车——机械方案与阶段进展汇报(1).pptx')
dst=Presentation('/home/jc/下载/VEX_V5_迷宫机器人_程序实现.pptx')
s0=src.slides[2]  # chassis photos
news=dst.slides.add_slide(dst.slide_layouts[6])
for sh in list(news.shapes):
    sh._element.getparent().remove(sh._element)
ridmap={}
for sh in s0.shapes:
    el=copy.deepcopy(sh._element)
    # map images
    for blip in el.iter(qn('a:blip')):
        old=blip.get(qn('r:embed'))
        if old:
            if old not in ridmap:
                rel=s0.part.rels[old]
                blob=rel.target_part.blob
                _,newrid=news.part.get_or_add_image_part(BytesIO(blob))
                ridmap[old]=newrid
            blip.set(qn('r:embed'),ridmap[old])
    news.shapes._spTree.insert_element_before(el,'p:extLst')
dst.save('/tmp/test_merge.pptx')
print('saved',len(dst.slides))
