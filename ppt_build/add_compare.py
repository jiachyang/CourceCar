CMP = r'''
# =====================================================================
# 机械 ↔ 程序：接口对照与待确认项
# =====================================================================
s = new_content('机械 ↔ 程序：接口对照与待确认项',
                'MECHANISM  ×  SOFTWARE  ·  PARAMETER MATCH  ·  SENSOR LAYOUT  ·  ARM INTERFACE',
                '2 项待确认', ORANGE, None)
rows = [
    ['底盘构型','四轮全向、四电机直驱、正方形框架','X-Drive IK；R=14.75 cm，r=5 cm，1:1','一致'],
    ['测距布置','“四角激光测距”','7=前、8=后、9=左、10=右；四向观测','需确认'],
    ['机械臂','单自由度；抓取位 / 存放位；机械限位','GRAB / DELIVER；缺 STOW 与到位反馈','待对齐'],
    ['进度','底盘制造完成；抓取机构 3D 完成','驱动 / IMU / ToF 读取已实现；EKF / 规划待做','机械先行'],
    ['电气接口','原稿未给限位开关与传感器安装角','电机 1–4，IMU 6，ToF 7–10','需核对'],
]
manual_table(s, 0.55, 1.60, 7.55, ['对比项','机械方案（原稿）','程序实现（Car/）','结论'], rows,
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
     'p_i = p + R(θ)r_i、n_i = R(θ)u_i 必须用实测安装位置与方向；否则墙距约束的方向会错。'),
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
         '机械部分 7 页按原稿保留（约 5 min，原页码 02/08–07/08）；程序部分从下一页开始。',
         size=6.8, color=LGRAY)
'''
