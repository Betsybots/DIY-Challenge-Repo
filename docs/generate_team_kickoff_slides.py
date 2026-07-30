#!/usr/bin/env python3
"""
generate_team_kickoff_slides.py
Produces docs/Team_Kickoff_Slides.pdf — PPT-style, one topic per slide.
Run:  python3 docs/generate_team_kickoff_slides.py
"""

import os
from reportlab.lib.pagesizes import landscape, A4
from reportlab.lib.units import cm
from reportlab.lib import colors
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.enums import TA_LEFT, TA_CENTER
from reportlab.platypus import (
    BaseDocTemplate, PageTemplate, Frame, NextPageTemplate,
    Paragraph, Spacer, Table, TableStyle,
    PageBreak, HRFlowable,
)

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_PATH  = os.path.join(REPO_ROOT, 'docs', 'Team_Kickoff_Slides.pdf')

PW, PH = landscape(A4)
M  = 1.8*cm
CW = PW - 2*M

# ─── Palette ─────────────────────────────────────────────────────────────────
NAVY   = colors.HexColor('#0F172A')
BLUE   = colors.HexColor('#1D4ED8')
LBLUE  = colors.HexColor('#DBEAFE')
TEAL   = colors.HexColor('#0D9488')
GREEN  = colors.HexColor('#16A34A')
PURPLE = colors.HexColor('#7C3AED')
ORANGE = colors.HexColor('#EA580C')
RED    = colors.HexColor('#B91C1C')
DGREY  = colors.HexColor('#334155')
MGREY  = colors.HexColor('#64748B')
LGREY  = colors.HexColor('#F8FAFC')
WHITE  = colors.white
SLATE  = colors.HexColor('#1E293B')

# ─── Styles ──────────────────────────────────────────────────────────────────
_styles = {}
def st(name, **kw):
    if name in _styles:
        name = f'{name}_{len(_styles)}'
    s = ParagraphStyle(name, **kw)
    _styles[name] = s
    return s

H1      = st('H1', fontName='Helvetica-Bold', fontSize=24, leading=30, textColor=NAVY,  alignment=TA_LEFT, spaceBefore=4, spaceAfter=2)
H2      = st('H2', fontName='Helvetica-Bold', fontSize=12, leading=16, textColor=WHITE, alignment=TA_LEFT)
BODY    = st('BD', fontName='Helvetica',      fontSize=11.5, leading=17, textColor=DGREY, alignment=TA_LEFT, spaceAfter=2)
WBULLET = st('WB', fontName='Helvetica',      fontSize=10.5, leading=15, textColor=WHITE, leftIndent=12, spaceAfter=3)
SMALL   = st('SM', fontName='Helvetica',      fontSize=9, leading=13, textColor=MGREY, alignment=TA_LEFT)
TAG_S   = st('TG', fontName='Helvetica-Bold', fontSize=9, leading=12, textColor=WHITE, alignment=TA_CENTER)

def SP(n=8):   return Spacer(1, n)
def HR(c=BLUE, thick=1.5): return HRFlowable(width='100%', thickness=thick, color=c, spaceAfter=6, spaceBefore=2)
def PB():      return PageBreak()


# ─── Page Decorations ─────────────────────────────────────────────────────────
def draw_slide(canvas, doc):
    canvas.saveState()
    # White background
    canvas.setFillColor(WHITE); canvas.rect(0, 0, PW, PH, fill=1, stroke=0)
    # Top accent bar
    canvas.setFillColor(BLUE); canvas.rect(0, PH - 0.4*cm, PW, 0.4*cm, fill=1, stroke=0)
    # Bottom footer area
    canvas.setFillColor(colors.HexColor('#F1F5F9'))
    canvas.rect(0, 0, PW, 1.0*cm, fill=1, stroke=0)
    canvas.setFont('Helvetica', 7.5); canvas.setFillColor(MGREY)
    canvas.drawString(M, 0.35*cm, 'Betsybots  ·  DIY Challenge 2026  ·  Team Kickoff')
    canvas.drawRightString(PW - M, 0.35*cm, f'Slide {doc.page}')
    # Thin separator above footer
    canvas.setStrokeColor(colors.HexColor('#E2E8F0')); canvas.setLineWidth(0.5)
    canvas.line(0, 1.0*cm, PW, 1.0*cm)
    canvas.restoreState()

def draw_cover(canvas, doc):
    canvas.saveState()
    canvas.setFillColor(NAVY); canvas.rect(0, 0, PW, PH, fill=1, stroke=0)
    # Decorative accent stripe
    canvas.setFillColor(TEAL); canvas.rect(0, PH*0.42, PW, 0.25*cm, fill=1, stroke=0)
    # Bottom gradient hint
    canvas.setFillColor(colors.HexColor('#162044')); canvas.rect(0, 0, PW, PH*0.15, fill=1, stroke=0)
    canvas.restoreState()


# ─── Reusable Components ──────────────────────────────────────────────────────
def slide_title(title, subtitle=None):
    out = [SP(4), Paragraph(title, H1)]
    if subtitle:
        out.append(Paragraph(subtitle, BODY))
    out.append(HR(BLUE, 1.5))
    out.append(SP(6))
    return out


def info_box(title, bullets, bg=BLUE, width=None):
    """Colored card with a bold title and bullet items."""
    w = (width or CW) - 24
    rows = [[Paragraph(title, H2)]] + [[Paragraph(f'• {b}', WBULLET)] for b in bullets]
    inner = Table(rows, colWidths=[w])
    inner.setStyle(TableStyle([
        ('BACKGROUND',    (0,0),(-1,-1), bg),
        ('TOPPADDING',    (0,0),(0,0), 10),
        ('TOPPADDING',    (0,1),(-1,-1), 2),
        ('BOTTOMPADDING', (0,0),(-1,-1), 5),
        ('LEFTPADDING',   (0,0),(-1,-1), 14),
        ('RIGHTPADDING',  (0,0),(-1,-1), 14),
        ('ROUNDEDCORNERS', [6]),
    ]))
    return inner


def two_col(left, right, split=0.5):
    """Place two flowables side by side."""
    wl = CW * split - 0.3*cm
    wr = CW * (1-split) - 0.3*cm
    t = Table([[left, right]], colWidths=[wl, wr], hAlign='LEFT')
    t.setStyle(TableStyle([
        ('VALIGN',(0,0),(-1,-1),'TOP'),
        ('LEFTPADDING',(0,0),(-1,-1),0),('RIGHTPADDING',(0,0),(-1,-1),0),
        ('TOPPADDING',(0,0),(-1,-1),0),('BOTTOMPADDING',(0,0),(-1,-1),0),
    ]))
    return t


def stacked(*items, width=None):
    """Vertically stack flowables into one flowable (no KeepTogether needed)."""
    w = width or CW*0.49
    rows = [[item] for item in items]
    t = Table(rows, colWidths=[w])
    t.setStyle(TableStyle([
        ('TOPPADDING',(0,0),(-1,-1),0),('BOTTOMPADDING',(0,0),(-1,-1),0),
        ('LEFTPADDING',(0,0),(-1,-1),0),('RIGHTPADDING',(0,0),(-1,-1),0),
    ]))
    return t


def step_row(num, title, desc, color=BLUE):
    num_t = Table([[Paragraph(str(num), st(f'SN{num}', fontName='Helvetica-Bold',
        fontSize=18, textColor=WHITE, alignment=TA_CENTER))]], colWidths=[1.1*cm])
    num_t.setStyle(TableStyle([
        ('BACKGROUND',(0,0),(-1,-1),color),
        ('TOPPADDING',(0,0),(-1,-1),7),('BOTTOMPADDING',(0,0),(-1,-1),7),
        ('LEFTPADDING',(0,0),(-1,-1),0),('RIGHTPADDING',(0,0),(-1,-1),0),
        ('ROUNDEDCORNERS',[4]),
    ]))
    txt_t = Table([
        [Paragraph(f'<b>{title}</b>', st(f'SRT{num}', fontName='Helvetica-Bold', fontSize=11.5, textColor=NAVY, leading=15))],
        [Paragraph(desc, st(f'SRD{num}', fontName='Helvetica', fontSize=10.5, textColor=DGREY, leading=14))],
    ], colWidths=[CW - 1.8*cm])
    txt_t.setStyle(TableStyle([
        ('BACKGROUND',(0,0),(-1,-1),LGREY),
        ('TOPPADDING',(0,0),(-1,-1),5),('BOTTOMPADDING',(0,0),(-1,-1),5),
        ('LEFTPADDING',(0,0),(-1,-1),12),('RIGHTPADDING',(0,0),(-1,-1),12),
        ('ROUNDEDCORNERS',[4]),
    ]))
    row = Table([[num_t, txt_t]], colWidths=[1.3*cm, CW - 1.3*cm])
    row.setStyle(TableStyle([
        ('VALIGN',(0,0),(-1,-1),'MIDDLE'),
        ('LEFTPADDING',(0,0),(-1,-1),0),('RIGHTPADDING',(0,0),(-1,-1),0),
        ('TOPPADDING',(0,0),(-1,-1),0),('BOTTOMPADDING',(0,0),(-1,-1),0),
    ]))
    return row


def dod_table(items):
    hdr = Paragraph('<b>✓ Definition of Done</b>', st('DH', fontName='Helvetica-Bold',
        fontSize=12, textColor=GREEN, leading=16))
    rows = [[hdr]]
    for i in items:
        rows.append([Paragraph(f'  ✓  {i}', st(f'DI', fontName='Helvetica',
            fontSize=10.5, leading=15, textColor=colors.HexColor('#D1FAE5'),
            leftIndent=8, spaceAfter=2))])
    t = Table(rows, colWidths=[CW])
    t.setStyle(TableStyle([
        ('BACKGROUND',(0,0),(-1,-1),colors.HexColor('#052E16')),
        ('TOPPADDING',(0,0),(0,0),10),('BOTTOMPADDING',(0,0),(0,0),4),
        ('TOPPADDING',(0,1),(-1,-1),3),('BOTTOMPADDING',(0,1),(-1,-1),3),
        ('LEFTPADDING',(0,0),(-1,-1),16),('RIGHTPADDING',(0,0),(-1,-1),16),
        ('ROUNDEDCORNERS',[6]),
    ]))
    return t


def flow_arrow(items_colors):
    cells, widths = [], []
    for i, (label, bg) in enumerate(items_colors):
        c = Table([[Paragraph(label, st(f'FL{i}', fontName='Helvetica-Bold',
            fontSize=9.5, textColor=WHITE, alignment=TA_CENTER, leading=13))]],
            colWidths=[3.8*cm])
        c.setStyle(TableStyle([
            ('BACKGROUND',(0,0),(-1,-1),bg),
            ('TOPPADDING',(0,0),(-1,-1),10),('BOTTOMPADDING',(0,0),(-1,-1),10),
            ('ROUNDEDCORNERS',[8]),
        ]))
        cells.append(c); widths.append(4.0*cm)
        if i < len(items_colors) - 1:
            cells.append(Paragraph('→', st(f'AR{i}', fontName='Helvetica-Bold',
                fontSize=18, textColor=MGREY, alignment=TA_CENTER)))
            widths.append(0.8*cm)
    t = Table([cells], colWidths=widths)
    t.setStyle(TableStyle([
        ('VALIGN',(0,0),(-1,-1),'MIDDLE'),
        ('LEFTPADDING',(0,0),(-1,-1),0),('RIGHTPADDING',(0,0),(-1,-1),0),
        ('TOPPADDING',(0,0),(-1,-1),0),('BOTTOMPADDING',(0,0),(-1,-1),0),
    ]))
    return t


def section_bar(text, color=BLUE):
    t = Table([[Paragraph(text, st('SB', fontName='Helvetica-Bold', fontSize=10,
        textColor=WHITE, leading=14))]], colWidths=[CW])
    t.setStyle(TableStyle([
        ('BACKGROUND',(0,0),(-1,-1),color),
        ('TOPPADDING',(0,0),(-1,-1),6),('BOTTOMPADDING',(0,0),(-1,-1),6),
        ('LEFTPADDING',(0,0),(-1,-1),14),('RIGHTPADDING',(0,0),(-1,-1),14),
        ('ROUNDEDCORNERS',[4]),
    ]))
    return t


# ─── Build PDF ────────────────────────────────────────────────────────────────
def build_pdf():
    frame_cover = Frame(M, 0, CW, PH, leftPadding=0, rightPadding=0, topPadding=0, bottomPadding=0)
    frame_slide = Frame(M, 1.0*cm, CW, PH - 1.5*cm, leftPadding=0, rightPadding=0, topPadding=0.3*cm, bottomPadding=0)

    doc = BaseDocTemplate(OUT_PATH, pagesize=landscape(A4),
        leftMargin=M, rightMargin=M, topMargin=0, bottomMargin=0)
    doc.addPageTemplates([
        PageTemplate(id='cover', frames=[frame_cover], onPage=draw_cover),
        PageTemplate(id='slide', frames=[frame_slide], onPage=draw_slide),
    ])

    s = []

    # ══════════════════════════════════════════════════════════════════════════
    # SLIDE 1: COVER
    # ══════════════════════════════════════════════════════════════════════════
    s.append(NextPageTemplate('cover'))
    s += [SP(100),
          Paragraph('BETSYBOTS', st('OG', fontName='Helvetica-Bold', fontSize=12,
              textColor=TEAL, alignment=TA_LEFT, spaceAfter=6, letterSpacing=3)),
          SP(4),
          Paragraph('DIY Challenge 2026', st('CT', fontName='Helvetica-Bold', fontSize=42,
              leading=48, textColor=WHITE, alignment=TA_LEFT)),
          SP(8),
          Paragraph('Team Kickoff — Development Strategy & Sprint Plan', st('CS', fontName='Helvetica',
              fontSize=16, leading=22, textColor=colors.HexColor('#94A3B8'), alignment=TA_LEFT)),
          SP(30)]

    # Tech tags in a row
    tags = [('ROS 2 Humble', BLUE), ('FAST-LIO2', TEAL), ('Nav2', PURPLE),
            ('LIO-SAM', ORANGE), ('Hesai QT64', DGREY), ('Orin Nano 8GB', GREEN)]
    tag_cells, tag_widths = [], []
    for lb, bg in tags:
        c = Table([[Paragraph(lb, TAG_S)]], colWidths=[3.2*cm])
        c.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,-1),bg),
            ('TOPPADDING',(0,0),(-1,-1),5),('BOTTOMPADDING',(0,0),(-1,-1),5),
            ('ROUNDEDCORNERS',[10])]))
        tag_cells.append(c); tag_widths.append(3.4*cm)
    s.append(Table([tag_cells], colWidths=tag_widths))
    s += [SP(24),
          Paragraph('June 2026', st('CD', fontName='Helvetica', fontSize=10,
              textColor=MGREY, alignment=TA_LEFT)),
          PB()]

    # ══════════════════════════════════════════════════════════════════════════
    # SLIDE 2: WHY LLMs
    # ══════════════════════════════════════════════════════════════════════════
    s.append(NextPageTemplate('slide'))
    s += slide_title('Why We Use LLMs')
    w = CW*0.49
    s.append(two_col(
        info_box('Speed & Focus',
            ['Iterate on ROS2 nodes, Nav2 configs, launch files faster than from scratch',
             'Skip boilerplate: CMakeLists, package.xml, param YAML — let LLMs generate it',
             'Rapidly prototype ideas — cmd_vel logic, EKF tuning, costmap params'],
            BLUE, width=w),
        info_box('Where LLMs Excel',
            ['EKF/Nav2 YAML params: LLMs reason about parameter interactions',
             'Targeted answers on FAST-LIO2 / LIO-SAM without reading all source',
             'Generate analysis scripts on demand — plotting, debugging, validation'],
            TEAL, width=w),
    ))
    s.append(SP(10))
    s.append(info_box('Why This Matters for Us',
        ['LLM-assisted development is day-to-day work at CAT — directly relevant practical training',
         'Prompting effectively for robotics code is a transferable engineering skill'],
        DGREY, width=CW))
    s.append(PB())

    # ══════════════════════════════════════════════════════════════════════════
    # SLIDE 3: DEBUGGING WITH LLMs
    # ══════════════════════════════════════════════════════════════════════════
    s += slide_title('Debugging With LLMs', 'Generate post-run analysis scripts on demand, iterate with LLM help')
    w = CW*0.49
    s.append(two_col(
        stacked(
            info_box('Trajectory Accuracy Plots',
                ['Compare estimated path vs ground truth per lap',
                 'Quantify drift per section — pinpoint where localization fails'], BLUE, width=w),
            SP(6),
            info_box('/cmd_vel Comparison',
                ['/cmd_vel_safe vs /cmd_vel_nav overlaid in one time-series',
                 'See if safety layer triggers unexpectedly mid-run'], PURPLE, width=w),
            width=w),
        stacked(
            info_box('EKF Covariance Health',
                ['Plot innovation sequence over time',
                 'Catch filter divergence before competition day'], TEAL, width=w),
            SP(6),
            info_box('IMU Bias Drift',
                ['Raw IMU vs EKF-corrected accel/gyro across a full run',
                 'Data-driven justification to re-calibrate'], ORANGE, width=w),
            width=w),
    ))
    s.append(SP(8))
    s.append(info_box('Nav2 Costmap Animation',
        ['Pull costmap snapshots from bag → animate → see if inflation radius or raytrace range causes path failures'], DGREY, width=CW))
    s.append(PB())

    # ══════════════════════════════════════════════════════════════════════════
    # SLIDE 4: DEVELOPER SPLIT
    # ══════════════════════════════════════════════════════════════════════════
    s += slide_title('Developer Work Split', 'One repo — task-based ownership, no folder locks')
    s.append(info_box('Developer 1 — Integration & Calibration',
        ['System glue: startup sequence, cmd_vel mux, e-stop, URDF / robot description',
         'IMU calibration, LiDAR-IMU extrinsics, AprilTag EKF tuning on test track',
         'Repo management, branch health, integration testing, final stack validation'], BLUE, width=CW))
    s.append(SP(6))
    s.append(info_box('Developer 2 — Mapping & Localization',
        ['Prior map generation via LIO-SAM offline mapping — joystick-driven on test track',
         'Validate FAST-LIO2 + RTK EKF fusion, measure lap-to-lap localization accuracy',
         'Slot calibration numbers into configs, verify improvement with fresh runs'], TEAL, width=CW))
    s.append(SP(6))
    s.append(info_box('Developer 3 — Perception & Mission Logic',
        ['LiDAR cluster classifier: drive-through / go-around / ignore decisions',
         'Mission state machine: PUSH_THROUGH, NAVIGATE_AROUND, BLIND_DRIVE_MODE',
         'Offline bag replay so perception can be tested on a laptop without the robot'], PURPLE, width=CW))
    s.append(PB())

    # ══════════════════════════════════════════════════════════════════════════
    # SLIDE 5: OTHER WORK ITEMS
    # ══════════════════════════════════════════════════════════════════════════
    s += slide_title('Other Focused Work Items', 'Smaller tasks — each needs an owner')
    items_data = [
        ('Visual Start Detector', 'Camera node watching for race start signal → avoids 5-sec manual-start penalty', BLUE),
        ('RViz2 Debug Config', 'Currently launches blank. Needs robot model, LiDAR scan, costmap, Nav2 path, TF tree', PURPLE),
        ('Competition Waypoints', 'Obstacle zone entries, narrow passage point, start/finish. Defined after prior map', TEAL),
        ('Offline Bag Replay', 'Test perception on laptop without robot. Critical for Dev 3 between robot sessions', ORANGE),
        ('AprilTag EKF Tuning', 'Print tags, survey with RTK, drive laps, record bag, post-process to tune Q/R params', GREEN),
    ]
    rows = []
    for name, desc, c in items_data:
        rows.append([
            Paragraph(f'<b>{name}</b>', st(f'IT', fontName='Helvetica-Bold', fontSize=11, textColor=c, leading=14)),
            Paragraph(desc, st(f'ID', fontName='Helvetica', fontSize=10.5, textColor=DGREY, leading=14)),
        ])
    tbl = Table(rows, colWidths=[5.0*cm, CW - 5.3*cm])
    tbl.setStyle(TableStyle([
        ('ROWBACKGROUNDS',(0,0),(-1,-1),[WHITE, LGREY]),
        ('TOPPADDING',(0,0),(-1,-1),9), ('BOTTOMPADDING',(0,0),(-1,-1),9),
        ('LEFTPADDING',(0,0),(-1,-1),12), ('RIGHTPADDING',(0,0),(-1,-1),12),
        ('VALIGN',(0,0),(-1,-1),'MIDDLE'),
        ('LINEBELOW',(0,0),(-1,-1),0.4,colors.HexColor('#E2E8F0')),
        ('LINEBELOW',(0,0),(-1,0),1.2,BLUE),
    ]))
    s.append(tbl)
    s.append(PB())

    # ══════════════════════════════════════════════════════════════════════════
    # SLIDE 6: WHY NO CAMERA DETECTION
    # ══════════════════════════════════════════════════════════════════════════
    s += slide_title('Why No Camera-Based Object Detection?',
        'LiDAR-first perception is the right call for this outdoor challenge')
    w = CW*0.49
    s.append(two_col(
        info_box('Camera Problems Outdoors',
            ['Sun glare, harsh shadows, exposure shifts',
             'Dust, reflections, lighting variance morning→afternoon',
             'Needs large labelled dataset — we don\'t have one',
             'Adds inference compute on already-busy Orin'],
            RED, width=w),
        info_box('LiDAR Geometry Wins',
            ['Stable point cloud regardless of lighting',
             'Already running for odometry — zero extra cost',
             'PCL clustering is deterministic, no training data',
             'Map-diff catches new obstacles without any model'],
            GREEN, width=w),
    ))
    s.append(SP(10))
    s.append(info_box('Camera Still Has a Role — Just Not Primary Perception',
        ['AprilTag detection for EKF calibration on test track',
         'Visual start signal detector (avoid 5-sec penalty)',
         'Depth assist for chimes obstacle (LiDAR ground-level blind spot)'],
        DGREY, width=CW))
    s.append(PB())

    # ══════════════════════════════════════════════════════════════════════════
    # SLIDE 7: HOW WE WORK
    # ══════════════════════════════════════════════════════════════════════════
    s += slide_title('How We Work Together', 'Task-based ownership — small focused PRs, regular syncs')
    s.append(step_row(1, 'Pick a task → create a branch',
        'Naming:  feature/task-name  ·  calib/imu-noise  ·  fix/ekf-params', BLUE))
    s.append(SP(6))
    s.append(step_row(2, 'Own it end-to-end',
        'No folder-level permission gates. Touch config, code, launch files — whatever the task needs.', TEAL))
    s.append(SP(6))
    s.append(step_row(3, 'Open PR → 1 review → merge',
        'Small PRs scoped to one outcome. Describe changes, testing, and known risks.', PURPLE))
    s.append(SP(6))
    s.append(step_row(4, 'Sync every 2 days (15 min max)',
        'What changed · What outputs are ready to hand off · What blocks the next session', ORANGE))
    s.append(SP(10))
    s.append(info_box('Branch Protection on main',
        ['Direct pushes blocked — every change goes through a PR with ≥1 approval',
         'main is always launchable — nobody breaks the build for others'],
        DGREY, width=CW))
    s.append(PB())

    # ══════════════════════════════════════════════════════════════════════════
    # SLIDE 8: WEEK 1 GOAL
    # ══════════════════════════════════════════════════════════════════════════
    s += slide_title('Week 1 — Calibration Lock',
        'Make sensor geometry and noise trustworthy before anything else')
    s.append(Paragraph(
        'Map quality, localization accuracy, and obstacle detection all depend on correct calibration. '
        'A bad calibration bakes errors into everything downstream — including the map itself.',
        BODY))
    s.append(SP(8))
    s.append(section_bar('CALIBRATION ORDER — must follow this sequence exactly', RED))
    s.append(SP(10))
    s.append(flow_arrow([
        ('1  IMU Noise\nCalibration', BLUE),
        ('2  LiDAR-IMU\nExtrinsics', TEAL),
        ('3  Camera-LiDAR\nExtrinsics', PURPLE),
        ('4  Record\nPrior Map', GREEN),
    ]))
    s.append(SP(14))
    s.append(dod_table([
        'IMU noise values measured → applied to FAST-LIO config → committed',
        'LiDAR-IMU extrinsic transform → applied to config and URDF → committed',
        'Camera-LiDAR extrinsic transform → applied to URDF → committed',
        'Validation run: stable localization, no TF mismatch, no drift spikes',
        'All changes in one PR, reviewed and merged to main',
    ]))
    s.append(PB())

    # ══════════════════════════════════════════════════════════════════════════
    # SLIDE 9: WEEK 1 TASK SPLIT
    # ══════════════════════════════════════════════════════════════════════════
    s += slide_title('Week 1 — Who Does What')
    w = CW*0.49
    left_col = info_box('Dev 1 — Integration Lead',
        ['Prepare calibration branch & team checklist',
         'Verify all scripts run on robot + laptop before sessions',
         'Apply calibration outputs into config / URDF as numbers arrive',
         'Run final validation: TF tree, topic rates, no frame conflicts',
         'Open PR with test notes → review → merge',
         'Write tune_ekf_apriltag.py skeleton',
         'Build working RViz2 debug config'],
        BLUE, width=w)

    right_col = stacked(
        info_box('Dev 2 — IMU + LiDAR-IMU',
            ['Run IMU noise calibration, extract noise params',
             'Update FAST-LIO noise values from results',
             'Run LiDAR-IMU extrinsic calib (figure-8 protocol)',
             'Deliver transform values to Dev 1',
             'Re-run to confirm FAST-LIO stable with new values'],
            TEAL, width=w),
        SP(6),
        info_box('Dev 3 — Camera-LiDAR + Data QA',
            ['Prepare calibration target board',
             'Run camera-lidar data collection session',
             'Process bag offline → solve transform',
             'Deliver transform values to Dev 1',
             'Record clean post-calibration validation bag'],
            PURPLE, width=w),
        width=w)

    s.append(two_col(left_col, right_col))
    s.append(PB())

    # ══════════════════════════════════════════════════════════════════════════
    # SLIDE 10: FULL ROADMAP → Aug 15 lock
    # ══════════════════════════════════════════════════════════════════════════
    s += slide_title('Jun → Aug 15 Roadmap', 'Competition Oct 1 — features locked by Aug 15 (45-day buffer for fixes & runs)')

    phases = [
        ('JUNE', 'Jun 1–28', 'Calibration &\nMapping',
         ['IMU noise + LiDAR-IMU extrinsics',
          'Camera-LiDAR extrinsics',
          'Prior map recording (LIO-SAM)',
          'FAST-LIO + RTK EKF validation',
          'AprilTag EKF tuning sessions'], BLUE),
        ('JULY', 'Jul 1–26', 'Perception &\nNav Tuning',
         ['LiDAR cluster classifier',
          'Zone-based behavior switching',
          'Nav2 param tuning (costmap, DWB)',
          'Offline bag replay workflow',
          'Lap accuracy < 10 cm drift'], TEAL),
        ('AUGUST', 'Aug 1–15', 'Mission Logic &\nIntegration',
         ['Mission state machine (zones)',
          'Visual start detector',
          'Competition waypoint set',
          'Full startup → run → stop flow',
          'E-stop stress testing'], PURPLE),
        ('AUG 16 –\nSEP 30', '45-day buffer', 'Bug Fixes &\nPractice Runs',
         ['Multiple full-course dry runs',
          'Race-day runbook written',
          'Backup recovery procedures',
          'Hardware spares checked',
          'Nothing new — only fixes'], GREEN),
    ]
    w_each = CW / 4 - 0.2*cm
    phase_cells = []
    for label, dates, goal, items, bg in phases:
        rows = [
            [Paragraph(label, st(f'PL', fontName='Helvetica-Bold', fontSize=9, textColor=colors.HexColor('#94A3B8'), leading=12))],
            [Paragraph(dates, st(f'PD', fontName='Helvetica', fontSize=8, textColor=colors.HexColor('#CBD5E1'), leading=11))],
            [SP(3)],
            [Paragraph(goal, st(f'PG', fontName='Helvetica-Bold', fontSize=11, textColor=WHITE, leading=14, spaceAfter=3))],
        ] + [[Paragraph(f'• {i}', st(f'PI', fontName='Helvetica', fontSize=9,
                textColor=colors.HexColor('#E2E8F0'), leading=12, leftIndent=4, spaceAfter=1))]
             for i in items]
        t = Table(rows, colWidths=[w_each - 0.4*cm])
        t.setStyle(TableStyle([
            ('BACKGROUND',(0,0),(-1,-1),bg),
            ('TOPPADDING',(0,0),(-1,-1),4),('BOTTOMPADDING',(0,0),(-1,-1),3),
            ('LEFTPADDING',(0,0),(-1,-1),12),('RIGHTPADDING',(0,0),(-1,-1),8),
            ('ROUNDEDCORNERS',[6]),
        ]))
        phase_cells.append(t)

    s.append(Table([phase_cells], colWidths=[w_each]*4))
    s.append(SP(8))
    s.append(info_box('Weekly Regroups — every Friday',
        ['What passed consistently · what failed repeatedly · top 3 blockers for next week',
         'Aug 15 hard lock: no new features after this date — 45 days of bug fixes and practice runs only',
         'Oct 1: Competition day'],
        DGREY, width=CW))

    doc.build(s)
    print(f'[OK] PDF written to: {OUT_PATH}')


if __name__ == '__main__':
    build_pdf()
