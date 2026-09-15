#!/usr/bin/env python3
"""
generate_mapping_guide.py — Mapping & Localization Workflow Guide
Produces docs/Mapping_Localization_Guide.pdf using ReportLab.
Run from the repo root:   python3 docs/generate_mapping_guide.py
"""

import os
import tempfile as _tmpmod

from reportlab.lib.pagesizes import A4
from reportlab.lib.units import cm
from reportlab.lib import colors
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.enums import TA_LEFT, TA_CENTER, TA_JUSTIFY
from reportlab.platypus import (
    SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle,
    PageBreak, HRFlowable, Preformatted,
)

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

# ── Output path ───────────────────────────────────────────────────────────────
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_PATH  = os.path.join(REPO_ROOT, 'docs', 'Mapping_Localization_Guide.pdf')

# ── Colour palette ─────────────────────────────────────────────────────────────
C_NAVY   = colors.HexColor('#1A2744')
C_BLUE   = colors.HexColor('#2563EB')
C_TEAL   = colors.HexColor('#0F766E')
C_ORANGE = colors.HexColor('#EA580C')
C_GREEN  = colors.HexColor('#16A34A')
C_LGREY  = colors.HexColor('#F1F5F9')
C_MGREY  = colors.HexColor('#94A3B8')
C_BLACK  = colors.HexColor('#0F172A')
C_WHITE  = colors.white


# ─────────────────────────────────────────────────────────────────────────────
# Styles
# ─────────────────────────────────────────────────────────────────────────────
def make_styles():
    base = getSampleStyleSheet()
    s = {}
    s['title'] = ParagraphStyle('DocTitle', parent=base['Title'],
        fontSize=26, leading=32, textColor=C_WHITE, spaceAfter=6, alignment=TA_CENTER)
    s['subtitle'] = ParagraphStyle('DocSubtitle',
        fontSize=12, leading=17, textColor=colors.HexColor('#CBD5E1'),
        spaceAfter=4, alignment=TA_CENTER)
    s['h1'] = ParagraphStyle('H1', fontSize=16, leading=20, textColor=C_NAVY,
        spaceBefore=18, spaceAfter=6, fontName='Helvetica-Bold')
    s['h2'] = ParagraphStyle('H2', fontSize=13, leading=17, textColor=C_BLUE,
        spaceBefore=12, spaceAfter=4, fontName='Helvetica-Bold')
    s['h3'] = ParagraphStyle('H3', fontSize=11, leading=15, textColor=C_NAVY,
        spaceBefore=8, spaceAfter=3, fontName='Helvetica-BoldOblique')
    s['body'] = ParagraphStyle('Body', fontSize=10, leading=14, textColor=C_BLACK,
        spaceAfter=6, alignment=TA_JUSTIFY)
    s['bullet'] = ParagraphStyle('Bullet', fontSize=10, leading=13, textColor=C_BLACK,
        spaceAfter=3, leftIndent=14, bulletIndent=4)
    s['code'] = ParagraphStyle('Code', fontName='Courier',
        fontSize=8.5, leading=12, textColor=C_BLACK,
        backColor=C_LGREY, borderPad=6, spaceAfter=6)
    s['code_label'] = ParagraphStyle('CodeLabel', fontName='Helvetica-Bold',
        fontSize=8, textColor=C_MGREY, spaceAfter=1)
    s['warn'] = ParagraphStyle('Warn', fontSize=9.5, leading=13,
        textColor=colors.HexColor('#7C2D12'),
        backColor=colors.HexColor('#FEF3C7'),
        borderPad=6, spaceAfter=6, fontName='Helvetica')
    s['note'] = ParagraphStyle('Note', fontSize=9.5, leading=13,
        textColor=colors.HexColor('#1E3A5F'),
        backColor=colors.HexColor('#DBEAFE'),
        borderPad=6, spaceAfter=6)
    s['tip'] = ParagraphStyle('Tip', fontSize=9.5, leading=13,
        textColor=colors.HexColor('#065F46'),
        backColor=colors.HexColor('#D1FAE5'),
        borderPad=6, spaceAfter=6)
    s['th'] = ParagraphStyle('TH', fontName='Helvetica-Bold',
        fontSize=9, textColor=C_WHITE, alignment=TA_CENTER)
    s['tc'] = ParagraphStyle('TC', fontName='Helvetica',
        fontSize=9, textColor=C_BLACK, leading=13)
    s['tc_sm'] = ParagraphStyle('TC_sm', fontName='Helvetica',
        fontSize=8, textColor=C_BLACK, leading=11)
    return s


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────
def H(text, level, styles):
    return Paragraph(text, styles[level])

def P(text, styles):
    return Paragraph(text, styles['body'])

def B(items, styles):
    return [Paragraph(f'• {item}', styles['bullet']) for item in items]

def code_block(label, text, styles):
    out = []
    if label:
        out.append(Paragraph(label, styles['code_label']))
    out.append(Preformatted(text, styles['code']))
    return out

def hr(styles):
    return HRFlowable(width='100%', thickness=0.5, color=C_MGREY, spaceAfter=8)

def warn_box(text, styles):
    return Paragraph('WARNING:  ' + text, styles['warn'])

def note_box(text, styles):
    return Paragraph('NOTE:  ' + text, styles['note'])

def tip_box(text, styles):
    return Paragraph('TIP:  ' + text, styles['tip'])

def SP(n=6):
    return Spacer(1, n)


# ─────────────────────────────────────────────────────────────────────────────
# Diagram: Full SLAM data-flow architecture
# ─────────────────────────────────────────────────────────────────────────────
def _make_architecture_diagram():
    tmp = _tmpmod.NamedTemporaryFile(suffix='.png', delete=False)
    fig, ax = plt.subplots(figsize=(14, 7))
    fig.patch.set_facecolor('white')
    ax.set_facecolor('white')
    ax.set_xlim(0, 14)
    ax.set_ylim(0, 7)
    ax.axis('off')

    def box(x, y, w, h, label, sublabel, color, tcolor='white'):
        ax.add_patch(FancyBboxPatch((x, y), w, h,
            boxstyle='round,pad=0.12', facecolor=color, edgecolor='#475569', lw=1.5))
        ax.text(x + w/2, y + h*0.62, label,
            ha='center', va='center', fontsize=9, fontweight='bold', color=tcolor)
        if sublabel:
            ax.text(x + w/2, y + h*0.28, sublabel,
                ha='center', va='center', fontsize=7, color=tcolor, alpha=0.85)

    def arrow(x1, y1, x2, y2, label='', color='#475569'):
        ax.annotate('', xy=(x2, y2), xytext=(x1, y1),
            arrowprops=dict(arrowstyle='->', color=color, lw=2.0))
        if label:
            mx, my = (x1+x2)/2, (y1+y2)/2
            ax.text(mx + 0.05, my + 0.12, label, fontsize=7, color='#374151',
                ha='left', va='bottom')

    # Sensors column
    box(0.3, 5.2, 2.8, 1.3, 'Hesai QT64', '/hesai/points\n64-beam, 20 Hz', '#1A2744')
    box(0.3, 3.5, 2.8, 1.3, 'IMU', '/imu/data  200 Hz', '#1A2744')
    box(0.3, 1.8, 2.8, 1.3, 'RTK GPS', '/gps/fix  10 Hz', '#065F46')

    # FAST-LIO2
    box(3.8, 4.5, 2.8, 1.5, 'FAST-LIO2', 'Lidar-Inertial Odometry\n/lidar_odometry', '#1D4ED8')
    arrow(3.1, 5.85, 3.8, 5.3, '/hesai/points')
    arrow(3.1, 4.15, 3.8, 4.9, '/imu/data')

    # EKF1
    box(3.8, 2.5, 2.8, 1.5, 'EKF1 — odom frame', '/lidar_odometry\n+ /imu/data → /odometry/filtered', '#0F766E')
    arrow(5.2, 4.5, 5.2, 4.05)
    arrow(3.1, 3.85, 3.8, 3.15, '/imu/data')

    # navsat_transform
    box(7.4, 4.5, 2.8, 1.5, 'navsat_transform', '/gps/fix + heading →\n/odometry/gps', '#065F46')
    arrow(3.1, 2.45, 7.4, 5.25)
    arrow(5.2, 3.25, 7.4, 5.0)

    # EKF2
    box(7.4, 2.5, 2.8, 1.5, 'EKF2 — map frame', '/odometry/filtered\n+ /odometry/gps → /odometry/global', '#1D4ED8')
    arrow(8.8, 4.5, 8.8, 4.05)
    arrow(5.2, 3.1, 7.4, 3.1)

    # Nav2
    box(11.0, 2.5, 2.7, 1.5, 'Nav2', 'MPPI + SmacHybrid\ncostmap → /cmd_vel', '#7C3AED')
    arrow(10.2, 3.25, 11.0, 3.25, 'map→odom TF')

    # LIO-SAM (mapping mode)
    box(7.4, 0.3, 2.8, 1.5, 'LIO-SAM (mapping)', 'Factor-graph SLAM\nSaves CornerMap.pcd', '#92400E')
    arrow(5.2, 2.5, 7.4, 1.3)

    ax.text(7.0, 6.8, 'RUNTIME MODE', fontsize=11, fontweight='bold',
        color='#1D4ED8', ha='center')
    ax.text(8.1, 0.05, 'MAPPING MODE (offline_mapping.launch.py)',
        fontsize=9, color='#92400E', ha='center')

    plt.tight_layout(pad=0.2)
    fig.savefig(tmp.name, dpi=130, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    return tmp.name


# ─────────────────────────────────────────────────────────────────────────────
# Cover page
# ─────────────────────────────────────────────────────────────────────────────
def _on_cover(canvas, doc):
    w, h = A4
    canvas.setFillColor(C_NAVY)
    canvas.rect(0, h - 8.3*cm, w, 8.3*cm, fill=1, stroke=0)
    canvas.setFillColor(C_TEAL)
    canvas.rect(0, h - 8.65*cm, w, 0.35*cm, fill=1, stroke=0)

def _on_page(canvas, doc):
    w, h = A4
    canvas.setFillColor(C_LGREY)
    canvas.rect(0, 0, w, 1.4*cm, fill=1, stroke=0)
    canvas.setFont('Helvetica', 8)
    canvas.setFillColor(C_MGREY)
    canvas.drawString(2*cm, 0.55*cm, 'DIY Challenge — Mapping & Localization Workflow Guide')
    canvas.drawRightString(w - 2*cm, 0.55*cm, f'Page {doc.page}')


# ─────────────────────────────────────────────────────────────────────────────
# Main builder
# ─────────────────────────────────────────────────────────────────────────────
def build_pdf():
    styles = make_styles()
    doc = SimpleDocTemplate(
        OUT_PATH, pagesize=A4,
        leftMargin=2*cm, rightMargin=2*cm,
        topMargin=2*cm, bottomMargin=2.2*cm,
    )

    def H1(t): return H(t, 'h1', styles)
    def H2(t): return H(t, 'h2', styles)
    def H3(t): return H(t, 'h3', styles)
    def Pb(t): return P(t, styles)
    def Bl(items): return B(items, styles)
    def CB(lbl, txt): return code_block(lbl, txt, styles)
    def HR(): return hr(styles)
    def WARN(t): return warn_box(t, styles)
    def NOTE(t): return note_box(t, styles)
    def TIP(t): return tip_box(t, styles)

    story = []

    # ── Cover ──────────────────────────────────────────────────────────────────
    story.append(SP(60))
    story.append(Paragraph('DIY Challenge Robot', styles['title']))
    story.append(SP(4))
    story.append(Paragraph('Mapping &amp; Localization<br/>Workflow Guide', styles['title']))
    story.append(SP(10))
    story.append(Paragraph(
        'LIO-SAM Prior-Map Generation • FAST-LIO2 Runtime Odometry<br/>'
        'EKF Fusion • GPS Integration • Nav2 Bring-Up',
        styles['subtitle']))
    story.append(SP(6))
    story.append(Paragraph('ROS 2 Humble • Jetson JetPack 5.x • Hesai QT64',
        styles['subtitle']))
    story.append(PageBreak())

    # ── TOC ───────────────────────────────────────────────────────────────────
    toc_data = [
        [Paragraph('Section', styles['th']), Paragraph('Topic', styles['th'])],
        ['1', 'Architecture Overview'],
        ['2', 'Pre-requisites & Environment Setup'],
        ['3', 'Mapping Run (LIO-SAM)'],
        ['4', 'Saving & Validating the Map'],
        ['5', 'Runtime Localization (FAST-LIO2 + EKF)'],
        ['6', 'GPS Fusion (navsat_transform + EKF2)'],
        ['7', 'Verifying Localization in RViz2'],
        ['8', 'Troubleshooting'],
    ]
    toc_table = Table(toc_data, colWidths=[2*cm, 14*cm])
    toc_table.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), C_NAVY),
        ('TEXTCOLOR',  (0,0), (-1,0), C_WHITE),
        ('ROWBACKGROUNDS', (0,1), (-1,-1), [C_WHITE, C_LGREY]),
        ('FONTNAME',   (0,1), (0,-1), 'Helvetica-Bold'),
        ('FONTSIZE',   (0,1), (-1,-1), 10),
        ('ALIGN',      (0,0), (0,-1), 'CENTER'),
        ('VALIGN',     (0,0), (-1,-1), 'MIDDLE'),
        ('GRID',       (0,0), (-1,-1), 0.4, C_MGREY),
        ('TOPPADDING', (0,0), (-1,-1), 5),
        ('BOTTOMPADDING', (0,0), (-1,-1), 5),
    ]))
    story += [H1('Table of Contents'), SP(4), toc_table, PageBreak()]

    # ── §1 Architecture Overview ───────────────────────────────────────────────
    story.append(H1('1  Architecture Overview'))
    story.append(Pb(
        'The robot uses a two-phase SLAM architecture. During a <b>mapping session</b> '
        'you drive the full competition area while LIO-SAM builds a globally consistent '
        '3-D point-cloud map using a GTSAM factor graph. At competition time the '
        '<b>runtime localization stack</b> loads that prior map and runs FAST-LIO2 + '
        'dual EKF for real-time pose estimation at 50 Hz.'
    ))

    arch_png = _make_architecture_diagram()
    from reportlab.platypus import Image as RLImage
    story.append(RLImage(arch_png, width=16*cm, height=8*cm))
    story.append(Pb('<i>Figure 1 — Full SLAM data-flow. Mapping mode (bottom) is offline; '
        'runtime mode (top) runs during competition.</i>'))
    story.append(SP(8))

    story.append(H2('1.1  Mapping Pipeline — LIO-SAM'))
    story += Bl([
        '<b>imuPreintegration</b> — integrates raw /imu/data between lidar scan timestamps '
        'to produce high-frequency pose predictions that motion-compensate each sweep.',
        '<b>imageProjection</b> — projects the raw 3-D PointCloud2 into a 2-D range image; '
        'applies IMU-derived distortion correction; segments ground plane.',
        '<b>featureExtraction</b> — computes per-point smoothness: low-smoothness points '
        'become edge features (walls, poles), high-smoothness become planar features (floor).',
        '<b>mapOptimization</b> — GTSAM factor graph using lidar, IMU, optional GPS, and '
        'loop-closure factors. Saves CornerMap.pcd, SurfaceMap.pcd, trajectory.pcd on request.',
    ])
    story.append(SP(6))

    story.append(H2('1.2  Runtime Localization Stack'))
    story += Bl([
        '<b>FAST-LIO2</b> (fastlio_mapping) — lidar-inertial odometry via iKD-Tree; '
        'publishes /lidar_odometry at lidar rate.',
        '<b>EKF1</b> (robot_localization, odom frame) — fuses /lidar_odometry + '
        '/imu/data at 50 Hz → /odometry/filtered + odom→base_link TF.',
        '<b>navsat_transform</b> — converts /gps/fix lat/lon to /odometry/gps in '
        'map frame using EKF1 heading for datum initialization.',
        '<b>EKF2</b> (robot_localization, map frame) — fuses /odometry/filtered + '
        '/odometry/gps at 30 Hz → /odometry/global + map→odom TF.',
    ])
    story.append(PageBreak())

    # ── §2 Pre-requisites ─────────────────────────────────────────────────────
    story.append(H1('2  Pre-requisites &amp; Environment Setup'))
    story.append(H2('2.1  Required Build State'))
    story += Bl([
        'third_party_ws built and installed: <tt>source third_party_ws/install/setup.bash</tt>',
        'DIY workspace built: <tt>colcon build --symlink-install</tt> inside ros2_ws/',
        'Calibration files in place (see Calibration Integration Guide)',
        'Competition area map driven at least once (§3) — or a prior map already in maps/',
    ])
    story.append(SP(4))
    story.append(H2('2.2  Source the Environment'))
    story += CB('Source Jetson profile (run once per shell session):', '''\
# On Jetson
source scripts/env.sh jetson

# Verify key variables
echo $DIY_HESAI_IP        # should be 192.168.1.201
echo $DIY_MICRO_ROS_SERIAL # should be /dev/ttyACM0
echo $ROS_DOMAIN_ID        # should be set
''')
    story.append(H2('2.3  Hardware Readiness Checks'))
    story += CB('Run health_check.sh before any mapping or localization session:', '''\
bash scripts/health_check.sh
# Checks: node liveness, TF tree completeness, topic rates
# Must show PASS for: hesai/points, imu/data, micro_ros_agent
''')
    story.append(WARN(
        'Do not proceed to mapping if health_check.sh reports any FAIL. '
        'IMU and lidar must both be publishing before starting LIO-SAM.'
    ))
    story.append(PageBreak())

    # ── §3 Mapping Run ────────────────────────────────────────────────────────
    story.append(H1('3  Mapping Run (LIO-SAM)'))
    story.append(Pb(
        'The mapping launch starts the four LIO-SAM nodes '
        '(imuPreintegration → imageProjection → featureExtraction → mapOptimization) '
        'plus an optional RViz window so you can monitor map quality in real time.'
    ))

    story.append(H2('3.1  Start the Mapping Session'))
    story += CB('Terminal 1 — start mapping:', '''\
source scripts/env.sh jetson
ros2 launch localization offline_mapping.launch.py use_rviz:=true
''')
    story += CB('Terminal 2 — start hardware drivers (lidar + IMU) separately if not already up:', '''\
# Hesai lidar driver
ros2 launch hesai_ros_driver manager.launch.py lidar_ip:=$DIY_HESAI_IP

# IMU (via micro-ROS if STM32 is the IMU gateway)
ros2 run micro_ros_agent micro_ros_agent serial --dev $DIY_MICRO_ROS_SERIAL -b 921600
''')
    story.append(NOTE(
        'The offline_mapping.launch.py file does NOT start hardware drivers. '
        'They must be running before or alongside the mapping launch.'
    ))
    story.append(SP(4))

    story.append(H2('3.2  Drive the Mapping Area'))
    story += Bl([
        'Drive at <b>0.3–0.5 m/s</b> maximum — faster motion increases lidar distortion.',
        'Cover the entire competition area at least once, including all corners.',
        'Make <b>at least two loops</b> so LIO-SAM can form loop-closure factors.',
        'Keep IMU data continuous — pausing is fine, but do not unplug while mapping.',
        'Watch RViz: green/blue map cloud should grow steadily without sudden jumps.',
        'If the map drifts badly, stop, fix calibration, and restart the session.',
    ])
    story.append(TIP(
        'For best loop-closure: drive a figure-8 or return to the start point exactly. '
        'LIO-SAM triggers ICP-based closure when the robot re-visits a keyframe within '
        'the configured radius (default ~6 m in params.yaml).'
    ))
    story.append(SP(4))

    story.append(H2('3.3  Monitoring in RViz'))
    story += Bl([
        '<b>/lio_sam/mapping/cloud_registered</b> — accumulated point-cloud map',
        '<b>/lio_sam/mapping/trajectory</b> — robot path markers',
        '<b>/lio_sam/mapping/loop_closure</b> — green lines = accepted loop closures',
    ])
    story.append(PageBreak())

    # ── §4 Save & Validate ───────────────────────────────────────────────────
    story.append(H1('4  Saving &amp; Validating the Map'))
    story.append(H2('4.1  Save the Map via Service Call'))
    story += CB('Save the map while mapOptimization is still running:', '''\
ros2 service call /lio_sam/save_map \\
  lio_sam/srv/SaveMap \\
  "{resolution: 0.2, destination: ''}"

# Default output: ~/Documents/lio_sam_directory/
#   CornerMap.pcd    — edge features (walls, poles)
#   SurfaceMap.pcd   — planar features (floor)
#   trajectory.pcd   — robot path
#   global_map.pcd   — merged full map
''')
    story.append(WARN(
        'Call /lio_sam/save_map BEFORE killing the mapping launch. '
        'Terminating the process without saving loses the map.'
    ))
    story.append(SP(4))

    story.append(H2('4.2  Copy Map to challenge_bringup'))
    story += CB('Move map files into the package:', '''\
# Create a dated directory for backups
MAP_DATE=$(date +%Y%m%d_%H%M)
mkdir -p src/challenge_bringup/maps/map_$MAP_DATE

cp ~/Documents/lio_sam_directory/*.pcd \\
   src/challenge_bringup/maps/map_$MAP_DATE/

# Point static_map.yaml at the new map
# Edit maps/static_map.yaml: set pcd_files to the new paths
''')

    story.append(H2('4.3  Update static_map.yaml'))
    story += CB('src/challenge_bringup/maps/static_map.yaml', '''\
# Update pcd_file to your new map directory
pcd_file: "maps/map_20240601_1430/global_map.pcd"
resolution: 0.2
''')
    story.append(SP(4))

    story.append(H2('4.4  Validate Map Quality'))
    story += CB('Quick visual validation with pcl_viewer (apt install pcl-tools):', '''\
pcl_viewer ~/Documents/lio_sam_directory/global_map.pcd
# Look for: clear walls, no double-surfaces, no ghost features
''')
    story += Bl([
        'Walls and pillars should appear as tight vertical planes (< 5 cm thickness).',
        'Floor should be a single horizontal plane with no gaps.',
        'Run laps again if map shows large drift artefacts — check lidar-IMU calibration.',
    ])
    story.append(PageBreak())

    # ── §5 Runtime Localization ───────────────────────────────────────────────
    story.append(H1('5  Runtime Localization (FAST-LIO2 + EKF)'))
    story.append(Pb(
        'The runtime localization launch starts FAST-LIO2 in scan-matching mode '
        '(using the prior PCD map), EKF1 for odom-frame fusion, and optionally '
        'navsat_transform + EKF2 for GPS-anchored global position.'
    ))

    story.append(H2('5.1  Launch Localization'))
    story += CB('Start runtime localization (GPS enabled):', '''\
source scripts/env.sh jetson
ros2 launch localization localization.launch.py \\
  mode:=runtime \\
  use_gps:=true \\
  config_file:=fast_lio_hesai_qt64.yaml
''')
    story += CB('GPS-denied environment (indoor / tunnel):', '''\
ros2 launch localization localization.launch.py \\
  mode:=runtime \\
  use_gps:=false
# FAST-LIO2 PGO provides map→odom without GPS
''')
    story.append(SP(4))

    story.append(H2('5.2  Data Flow in Runtime Mode'))
    data_flow = [
        [Paragraph('Step', styles['th']), Paragraph('Publisher', styles['th']),
         Paragraph('Topic', styles['th']), Paragraph('Consumer', styles['th'])],
        ['1', 'Hesai driver', '/hesai/points', 'FAST-LIO2'],
        ['2', 'IMU driver', '/imu/data', 'FAST-LIO2, EKF1'],
        ['3', 'FAST-LIO2', '/lidar_odometry', 'EKF1'],
        ['4', 'EKF1', '/odometry/filtered', 'EKF2, Nav2'],
        ['5', 'EKF1', 'odom→base_link TF', 'TF tree'],
        ['6', 'GPS receiver', '/gps/fix', 'navsat_transform'],
        ['7', 'navsat_transform', '/odometry/gps', 'EKF2'],
        ['8', 'EKF2', '/odometry/global', 'Nav2'],
        ['9', 'EKF2', 'map→odom TF', 'TF tree'],
    ]
    df_table = Table(data_flow, colWidths=[1*cm, 4*cm, 5*cm, 4.5*cm])
    df_table.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), C_NAVY),
        ('ROWBACKGROUNDS', (0,1), (-1,-1), [C_WHITE, C_LGREY]),
        ('FONTNAME', (0,1), (0,-1), 'Helvetica-Bold'),
        ('FONTSIZE', (0,0), (-1,-1), 9),
        ('ALIGN', (0,0), (0,-1), 'CENTER'),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('GRID', (0,0), (-1,-1), 0.4, C_MGREY),
        ('TOPPADDING', (0,0), (-1,-1), 4),
        ('BOTTOMPADDING', (0,0), (-1,-1), 4),
    ]))
    story += [H2('Runtime Topic Flow'), SP(4), df_table, SP(8)]

    story.append(H2('5.3  FAST-LIO2 Configuration Key Parameters'))
    story += CB('src/localization/config/fast_lio_hesai_qt64.yaml (excerpt):', '''\
common:
  lid_topic: /hesai/points
  imu_topic: /imu/data
  time_sync_en: false

preprocess:
  lidar_type: 2          # Velodyne-style PointCloud2
  scan_line: 64          # QT64 line count
  scan_rate: 20          # Hz — match hardware setting
  blind: 0.3             # metres — minimum lidar range

mapping:
  acc_cov: 0.1           # CALIB: replace from IMU calibration
  gyr_cov: 0.1           # CALIB: replace from IMU calibration
  b_acc_cov: 0.0001
  b_gyr_cov: 0.0001
  extrinsic_T: [0, 0, 0] # CALIB: lidar→IMU translation (metres)
  extrinsic_R: [1,0,0, 0,1,0, 0,0,1] # CALIB: rotation matrix
  extrinsic_est_en: true # Set false after calibration converges
''')
    story.append(PageBreak())

    # ── §6 GPS Fusion ────────────────────────────────────────────────────────
    story.append(H1('6  GPS Fusion (navsat_transform + EKF2)'))
    story.append(Pb(
        'GPS fusion gives the robot absolute global position, preventing '
        'long-term map drift over the competition course. It requires a valid '
        'heading from EKF1 before it can initialize the UTM datum.'
    ))

    story.append(H2('6.1  navsat_transform Configuration'))
    story += CB('src/localization/config/navsat_transform.yaml:', '''\
navsat_transform_node:
  ros__parameters:
    delay: 3.0                        # seconds to wait for valid heading
    magnetic_declination_radians: 0.0 # CALIB: NOAA value for competition site
    yaw_offset: 1.5707963267948966    # CALIB: robot +X → magnetic North angle
    use_odometry_yaw: false
    broadcast_utm_transform: true     # publishes UTM→map TF for RViz
    zero_altitude: true               # z=0 in map frame
    publish_filtered_gps: true
    frequency: 10.0
    two_d_mode: true
''')
    story.append(WARN(
        'magnetic_declination_radians must be set for the competition GPS location. '
        'Look up the value at ngdc.noaa.gov/geomag/calculators/magcalc.shtml and enter '
        'the competition venue coordinates. A wrong value causes systematic heading error.'
    ))
    story.append(SP(4))

    story.append(H2('6.2  EKF2 (Global Frame) Configuration'))
    story += CB('src/localization/config/ekf_global.yaml (key settings):', '''\
ekf_filter_node_map:
  ros__parameters:
    frequency: 30.0
    two_d_mode: true
    world_frame: map           # EKF2 publishes map→odom TF

    odom0: /odometry/filtered  # EKF1 output — full pose + velocity
    odom0_config: [true, true, false,   # x y z
                   false, false, true,  # roll pitch yaw
                   true, true, false,   # vx vy vz
                   false, false, true,  # vroll vpitch vyaw
                   false, false, false]

    odom1: /odometry/gps       # GPS-derived position from navsat_transform
    odom1_config: [true, true, false,   # x y only — no heading from GPS
                   false, false, false,
                   false, false, false,
                   false, false, false,
                   false, false, false]
''')
    story.append(NOTE(
        'GPS fix quality determines EKF2 accuracy. RTK fixed solution gives ~2 cm, '
        'DGNSS gives ~20 cm, single-point GPS gives ~1–3 m. The EKF2 will '
        'automatically use the per-fix covariance from /gps/fix.position_covariance '
        'to weight the GPS measurement appropriately.'
    ))
    story.append(PageBreak())

    # ── §7 RViz Verification ─────────────────────────────────────────────────
    story.append(H1('7  Verifying Localization in RViz2'))
    story.append(H2('7.1  Launch RViz'))
    story += CB('Run localization with RViz enabled:', '''\
ros2 launch localization localization.launch.py use_rviz:=true
# Uses challenge_bringup/rviz/localization.rviz config
''')

    story.append(H2('7.2  Key Topics to Monitor'))
    rviz_topics = [
        [Paragraph('Topic', styles['th']), Paragraph('Display', styles['th']),
         Paragraph('What Good Looks Like', styles['th'])],
        ['/hesai/points', 'PointCloud2', 'Tight point cluster around obstacles, no smear'],
        ['/tf (TF tree)', 'TF / Robot Model', 'All frames: map→odom→base_link→lidar/imu/camera'],
        ['/odometry/filtered', 'Odometry', 'Smooth path, no jumps; covariance ellipse fits robot'],
        ['/odometry/gps', 'Odometry', 'GPS odometry aligns with lidar odometry (< 0.5 m offset)'],
        ['/odometry/global', 'Odometry', 'Globally consistent; minimal divergence from /odometry/filtered'],
        ['/gps/filtered', 'NavSatFix → points', 'GPS track overlaps driven path'],
    ]
    rv_table = Table(rviz_topics, colWidths=[4.5*cm, 3.5*cm, 8.5*cm])
    rv_table.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), C_NAVY),
        ('ROWBACKGROUNDS', (0,1), (-1,-1), [C_WHITE, C_LGREY]),
        ('FONTSIZE', (0,0), (-1,-1), 8.5),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('GRID', (0,0), (-1,-1), 0.4, C_MGREY),
        ('TOPPADDING', (0,0), (-1,-1), 4),
        ('BOTTOMPADDING', (0,0), (-1,-1), 4),
    ]))
    story += [SP(4), rv_table, SP(8)]

    story.append(H2('7.3  TF Tree Sanity Check'))
    story += CB('Verify the complete TF chain:', '''\
ros2 run tf2_tools view_frames
# Expected chain: map → odom → base_link → lidar_link
#                                         → imu_link
#                                         → camera_link
# Any gap means a node failed to start or a topic remapping is wrong.
''')
    story.append(SP(4))

    story.append(H2('7.4  Quick Sanity Numbers'))
    story += Bl([
        '<b>Localization frequency</b>: /odometry/filtered should publish at ~50 Hz.',
        '<b>Pose covariance</b>: diagonal entries < 0.1 in x/y when FAST-LIO2 is locked.',
        '<b>GPS offset</b>: if /odometry/gps x/y deviates > 2 m from /odometry/filtered '
        'in a static test, check yaw_offset in navsat_transform.yaml.',
        '<b>IMU data rate</b>: /imu/data must publish at ≥ 200 Hz for FAST-LIO2.',
    ])
    story.append(PageBreak())

    # ── §8 Troubleshooting ───────────────────────────────────────────────────
    story.append(H1('8  Troubleshooting'))
    tc_sm = styles['tc_sm']
    ts_data = [
        [Paragraph('Symptom', styles['th']), Paragraph('Likely Cause', styles['th']),
         Paragraph('Fix', styles['th'])],
        [Paragraph('LIO-SAM map drifts / explodes', tc_sm),
         Paragraph('IMU extrinsic wrong or imu_topic rate too low', tc_sm),
         Paragraph('Re-run lidar_imu_calib; confirm /imu/data ≥ 200 Hz', tc_sm)],
        [Paragraph('FAST-LIO2 prints "no point cloud" at startup', tc_sm),
         Paragraph('/hesai/points not publishing or wrong topic name', tc_sm),
         Paragraph('Check lid_topic in fast_lio_hesai_qt64.yaml; echo topic', tc_sm)],
        [Paragraph('EKF1 covariance grows unbounded', tc_sm),
         Paragraph('Lidar odometry diverged; no IMU data arriving', tc_sm),
         Paragraph('Restart FAST-LIO2; check /imu/data; verify ekf_local.yaml', tc_sm)],
        [Paragraph('navsat_transform outputs nothing', tc_sm),
         Paragraph('Valid heading not yet available from EKF1', tc_sm),
         Paragraph('Wait 3 s for FAST-LIO2 to init (delay: 3.0 in navsat yaml)', tc_sm)],
        [Paragraph('map→odom TF missing', tc_sm),
         Paragraph('EKF2 not running or use_gps:=false with no fallback', tc_sm),
         Paragraph('Check ekf_global node status; start with use_gps:=false if no GPS', tc_sm)],
        [Paragraph('Nav2 skips goal immediately', tc_sm),
         Paragraph('map→odom TF stale or base→footprint missing', tc_sm),
         Paragraph('Check TF times: ros2 run tf2_ros tf2_echo map odom', tc_sm)],
        [Paragraph('/gps/fix not publishing', tc_sm),
         Paragraph('GPS antenna not connected or serial port wrong', tc_sm),
         Paragraph('Check /dev/ttyUSB* permission; verify nmea_serial_driver params', tc_sm)],
    ]
    ts_table = Table(ts_data, colWidths=[4.5*cm, 5.5*cm, 6.5*cm])
    ts_table.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), C_NAVY),
        ('ROWBACKGROUNDS', (0,1), (-1,-1), [C_WHITE, C_LGREY]),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('GRID', (0,0), (-1,-1), 0.4, C_MGREY),
        ('TOPPADDING', (0,0), (-1,-1), 4),
        ('BOTTOMPADDING', (0,0), (-1,-1), 4),
    ]))
    story += [SP(4), ts_table, SP(12)]


    story.append(H2('8.1  Log Inspection Commands'))
    story += CB('Useful diagnostics:', '''\
# EKF1 state
ros2 topic echo /odometry/filtered --once

# FAST-LIO2 convergence flag
ros2 topic echo /lidar_odometry --once

# GPS fix quality
ros2 topic echo /gps/fix --once | grep -E "status|covariance"

# TF tree snapshot
ros2 run tf2_tools view_frames && evince frames.pdf

# Record a short debug bag
ros2 bag record -o /tmp/debug_bag \\
  /hesai/points /imu/data /odometry/filtered \\
  /gps/fix /odometry/gps /tf /tf_static
''')
    story.append(HR())
    story.append(Pb(
        '<i>End of Mapping &amp; Localization Workflow Guide. '
        'For calibration parameter locations see the Calibration Integration Guide. '
        'For Nav2 parameter tuning see the Nav2 Tuning Guide.</i>'
    ))

    doc.build(story, onFirstPage=_on_cover, onLaterPages=_on_page)
    print(f'Written: {OUT_PATH}')


if __name__ == '__main__':
    build_pdf()
