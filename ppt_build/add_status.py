STAT = r'''
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
'''
