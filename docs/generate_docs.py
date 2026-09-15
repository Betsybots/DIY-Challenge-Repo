#!/usr/bin/env python3
"""
generate_docs.py — DIY Challenge Robot Documentation Generator
Produces docs/DIY_Challenge_Robot_Guide.pdf using ReportLab.
Run from the repo root:   python3 docs/generate_docs.py
"""

import os
import sys
from datetime import date

# ── ReportLab imports ─────────────────────────────────────────────────────────
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import cm, mm
from reportlab.lib import colors
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.enums import TA_LEFT, TA_CENTER, TA_JUSTIFY
from reportlab.platypus import (
    SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle,
    PageBreak, HRFlowable, ListFlowable, ListItem, KeepTogether,
    Preformatted, Image as RLImage,
)
from reportlab.platypus.tableofcontents import TableOfContents

# ── Matplotlib (diagram generation) ────────────────────────────────────────────
import tempfile as _tmpmod
import matplotlib
matplotlib.use('Agg')   # non-interactive backend; must be set before pyplot import
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch

# ── Output path ───────────────────────────────────────────────────────────────
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_PATH  = os.path.join(REPO_ROOT, 'docs', 'DIY_Challenge_Robot_Guide.pdf')

# ── Colour palette ────────────────────────────────────────────────────────────
C_NAVY   = colors.HexColor('#1A2744')
C_BLUE   = colors.HexColor('#2563EB')
C_CYAN   = colors.HexColor('#0EA5E9')
C_GREEN  = colors.HexColor('#16A34A')
C_ORANGE = colors.HexColor('#EA580C')
C_RED    = colors.HexColor('#DC2626')
C_LGREY  = colors.HexColor('#F1F5F9')
C_MGREY  = colors.HexColor('#94A3B8')
C_BLACK  = colors.HexColor('#0F172A')
C_WHITE  = colors.white


# ─────────────────────────────────────────────────────────────────────────────
# Style sheet
# ─────────────────────────────────────────────────────────────────────────────
def make_styles():
    base = getSampleStyleSheet()
    s = {}

    s['title'] = ParagraphStyle(
        'DocTitle', parent=base['Title'],
        fontSize=28, leading=34, textColor=C_WHITE,
        spaceAfter=6, alignment=TA_CENTER,
    )
    s['subtitle'] = ParagraphStyle(
        'DocSubtitle',
        fontSize=13, leading=18, textColor=colors.HexColor('#CBD5E1'),
        spaceAfter=4, alignment=TA_CENTER,
    )
    s['h1'] = ParagraphStyle(
        'H1',
        fontSize=16, leading=20, textColor=C_NAVY,
        spaceBefore=18, spaceAfter=6,
        fontName='Helvetica-Bold',
        borderPad=(0,0,2,0),
    )
    s['h2'] = ParagraphStyle(
        'H2',
        fontSize=13, leading=17, textColor=C_BLUE,
        spaceBefore=12, spaceAfter=4,
        fontName='Helvetica-Bold',
    )
    s['h3'] = ParagraphStyle(
        'H3',
        fontSize=11, leading=15, textColor=C_NAVY,
        spaceBefore=8, spaceAfter=3,
        fontName='Helvetica-BoldOblique',
    )
    s['body'] = ParagraphStyle(
        'Body',
        fontSize=10, leading=14, textColor=C_BLACK,
        spaceAfter=6, alignment=TA_JUSTIFY,
    )
    s['bullet'] = ParagraphStyle(
        'Bullet',
        fontSize=10, leading=13, textColor=C_BLACK,
        spaceAfter=3, leftIndent=14, bulletIndent=4,
    )
    s['code'] = ParagraphStyle(
        'Code',
        fontName='Courier',
        fontSize=8.5, leading=12, textColor=C_BLACK,
        backColor=C_LGREY, borderPad=6,
        spaceAfter=6,
    )
    s['code_label'] = ParagraphStyle(
        'CodeLabel',
        fontName='Helvetica-Bold',
        fontSize=8, textColor=C_MGREY,
        spaceAfter=1,
    )
    s['warn'] = ParagraphStyle(
        'Warn',
        fontSize=9.5, leading=13, textColor=colors.HexColor('#7C2D12'),
        backColor=colors.HexColor('#FEF3C7'),
        borderPad=6, spaceAfter=6,
        fontName='Helvetica',
    )
    s['note'] = ParagraphStyle(
        'Note',
        fontSize=9.5, leading=13, textColor=colors.HexColor('#1E3A5F'),
        backColor=colors.HexColor('#DBEAFE'),
        borderPad=6, spaceAfter=6,
    )
    s['calib'] = ParagraphStyle(
        'Calib',
        fontSize=9.5, leading=13, textColor=colors.HexColor('#065F46'),
        backColor=colors.HexColor('#D1FAE5'),
        borderPad=6, spaceAfter=6,
    )
    s['toc'] = ParagraphStyle(
        'TOC',
        fontSize=10, leading=14, textColor=C_BLUE,
        spaceAfter=2,
    )
    s['footer'] = ParagraphStyle(
        'Footer',
        fontSize=8, textColor=C_MGREY, alignment=TA_CENTER,
    )
    return s


# ─────────────────────────────────────────────────────────────────────────────
# Helper shortcuts
# ─────────────────────────────────────────────────────────────────────────────
def H(s, style_key, styles):
    return Paragraph(s, styles[style_key])

def P(s, styles):
    return Paragraph(s, styles['body'])

def B(items, styles):
    """Bullet list from a list of strings."""
    elems = []
    for item in items:
        elems.append(Paragraph(f'• {item}', styles['bullet']))
    return elems

def code_block(label, text, styles):
    """Grey code box with optional label line above."""
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

def calib_box(text, styles):
    return Paragraph('CALIBRATION REQUIRED:  ' + text, styles['calib'])


# ─────────────────────────────────────────────────────────────────────────────
# Diagram generators — matplotlib flowcharts embedded as PNG images in the PDF
# ─────────────────────────────────────────────────────────────────────────────
_DS = '#0EA5E9'   # sensor blue
_DP = '#16A34A'   # processing green
_DC = '#EA580C'   # control orange
_DN = '#7C3AED'   # nav purple
_DA = '#374151'   # actuator dark


def _mpl_box(ax, cx, cy, w, h, text, fc, tc='white', fs=8.5):
    """Draw a rounded-rectangle node on ax."""
    ax.add_patch(FancyBboxPatch(
        (cx - w/2, cy - h/2), w, h,
        boxstyle='round,pad=0.12', facecolor=fc,
        edgecolor='white', linewidth=1.5, zorder=3))
    ax.text(cx, cy, text, ha='center', va='center',
            color=tc, fontsize=fs, fontweight='bold', zorder=4,
            multialignment='center', linespacing=1.3)


def _mpl_arrow(ax, x1, y1, x2, y2, label='', rad=0.0,
               ac='#64748B', lc='#374151', lfs=6.8):
    """Draw an annotated curved arrow between two points."""
    ax.annotate('', xy=(x2, y2), xytext=(x1, y1),
                arrowprops=dict(arrowstyle='->', color=ac, lw=1.3,
                                connectionstyle=f'arc3,rad={rad}'), zorder=2)
    if label:
        mx, my = (x1 + x2) / 2, (y1 + y2) / 2
        ax.text(mx, my, label, ha='center', va='center',
                fontsize=lfs, color=lc, style='italic', zorder=5,
                bbox=dict(fc='white', ec='#CBD5E1', pad=1.5,
                          boxstyle='round,pad=0.2'))


def _make_arch_diagram():
    """System architecture / node-communication graph.  Returns temp PNG path."""
    tmp = _tmpmod.NamedTemporaryFile(suffix='.png', delete=False)
    fig, ax = plt.subplots(figsize=(13.5, 8.2))
    ax.set_xlim(0, 13.5)
    ax.set_ylim(0.2, 8.4)
    ax.axis('off')
    fig.patch.set_facecolor('white')

    # Vertical column dividers
    ax.axvline(3.3,  color='#E2E8F0', lw=0.8, zorder=0)
    ax.axvline(7.85, color='#E2E8F0', lw=0.8, zorder=0)

    # Column headers
    ax.text(1.5,  8.25, 'SENSORS',             ha='center', fontsize=8, color=_DS, fontweight='bold')
    ax.text(5.7,  8.25, 'SLAM / LOCALISATION', ha='center', fontsize=8, color=_DP, fontweight='bold')
    ax.text(10.7, 8.25, 'CONTROL & NAV',        ha='center', fontsize=8, color=_DC, fontweight='bold')

    # Sensor nodes
    _mpl_box(ax, 1.5, 7.3, 2.2, 0.65, 'Hesai QT64\nLidar',   _DS)
    _mpl_box(ax, 1.5, 6.1, 2.2, 0.65, 'D435i\nIMU',          _DS)
    _mpl_box(ax, 1.5, 4.2, 2.2, 0.65, 'RTK GPS',             _DS)
    _mpl_box(ax, 1.5, 2.0, 2.2, 0.65, 'STM32\n(micro-ROS)',  _DS)

    # SLAM / EKF nodes
    _mpl_box(ax, 5.7, 7.3, 2.7, 0.65, 'fastlio_mapping\n(FAST-LIO2)', _DP)
    _mpl_box(ax, 5.7, 5.9, 2.7, 0.65, 'EKF1\nekf_filter_node_odom',  _DP)
    _mpl_box(ax, 5.7, 4.2, 2.7, 0.65, 'navsat_transform',             _DP)
    _mpl_box(ax, 5.7, 2.8, 2.7, 0.65, 'EKF2\nekf_filter_node_map',   _DP)

    # Control / Nav nodes
    _mpl_box(ax, 10.7, 5.9, 2.7, 0.65, 'Nav2\n(MPPI planner)', _DN)
    _mpl_box(ax, 10.7, 2.8, 2.7, 0.65, 'estop_controller',     _DC)
    _mpl_box(ax, 10.7, 1.5, 2.7, 0.65, 'cmd_vel_mux',          _DC)

    # Actuator
    _mpl_box(ax, 6.5, 1.5, 2.6, 0.65, 'differential-drive\n(motor driver)', _DA)

    # Sensor -> processing
    _mpl_arrow(ax, 2.6,  7.3,  4.35, 7.3,  '/hesai/points')
    _mpl_arrow(ax, 2.6,  6.2,  4.35, 7.1,  '/imu/data', rad=0.25)  # IMU -> fastlio (arced)
    _mpl_arrow(ax, 2.6,  5.95, 4.35, 5.9,  '/imu/data')            # IMU -> EKF1
    _mpl_arrow(ax, 2.6,  4.2,  4.35, 4.2,  '/gps/fix')
    _mpl_arrow(ax, 2.6,  2.0,  9.35, 2.65, '/stm32/heartbeat  /stm32/estop')

    # SLAM vertical chain
    _mpl_arrow(ax, 5.7, 6.97, 5.7, 6.23, '/lidar_odometry')     # fastlio -> EKF1
    _mpl_arrow(ax, 5.7, 5.57, 5.7, 4.53, '/odometry/filtered')  # EKF1 -> navsat
    _mpl_arrow(ax, 5.7, 3.87, 5.7, 3.13, '/odometry/gps')       # navsat -> EKF2

    # EKF -> Nav2
    _mpl_arrow(ax, 7.05, 5.9,  9.35, 5.9,  '/odometry/filtered')
    _mpl_arrow(ax, 7.05, 2.8,  9.35, 5.57, '/odometry/global', rad=-0.3)

    # Control chain
    _mpl_arrow(ax, 10.7, 2.47, 10.7, 1.83, '/estop_active')
    _mpl_arrow(ax, 10.7, 5.57, 10.7, 1.83, '/cmd_vel_nav', rad=0.4)  # Nav2 -> mux (arced)
    _mpl_arrow(ax, 9.35, 1.5,  7.8,  1.5,  '/cmd_vel_safe')           # mux -> motor

    handles = [mpatches.Patch(color=c, label=l) for c, l in [
        (_DS, 'Sensors'), (_DP, 'SLAM / EKF'),
        (_DC, 'Control'), (_DN, 'Nav2'), (_DA, 'Actuators'),
    ]]
    ax.legend(handles=handles, loc='lower right', fontsize=8,
              framealpha=0.95, edgecolor='#CBD5E1', ncol=2)

    plt.savefig(tmp.name, dpi=150, bbox_inches='tight',
                facecolor='white', edgecolor='none')
    plt.close()
    return tmp.name


def _make_loc_pipeline():
    """EKF localisation pipeline diagram.  Returns temp PNG path."""
    tmp = _tmpmod.NamedTemporaryFile(suffix='.png', delete=False)
    fig, ax = plt.subplots(figsize=(13.0, 5.5))
    ax.set_xlim(0, 13.0)
    ax.set_ylim(0.2, 5.5)
    ax.axis('off')
    fig.patch.set_facecolor('white')

    _IC = '#0369A1'  # input topic
    _OC = '#0891B2'  # output / TF

    # Input topics
    _mpl_box(ax, 1.3, 4.8, 2.3, 0.60, '/hesai/points',        _IC, fs=8)
    _mpl_box(ax, 1.3, 3.9, 2.3, 0.60, '/imu/data',            _IC, fs=8)
    _mpl_box(ax, 1.3, 2.2, 2.3, 0.60, '/gps/fix',             _IC, fs=8)
    _mpl_box(ax, 1.3, 1.2, 2.3, 0.60, '/imu/data\n(heading)', _IC, fs=7.5)

    # Processing nodes
    _mpl_box(ax, 5.0, 4.35, 2.8, 0.65, 'fastlio_mapping\n(FAST-LIO2)', _DP)
    _mpl_box(ax, 5.0, 3.1,  2.8, 0.65, 'EKF1\nekf_filter_node_odom',  _DP)
    _mpl_box(ax, 5.0, 1.85, 2.8, 0.65, 'navsat_transform',             _DP)
    _mpl_box(ax, 9.2, 2.5,  2.8, 0.65, 'EKF2\nekf_filter_node_map',   _DP)

    # Output topics / TF
    _mpl_box(ax, 11.5, 3.8, 2.8, 0.65, '/odometry/filtered\nTF: odom->base_link', _OC, fs=8)
    _mpl_box(ax, 11.5, 1.5, 2.8, 0.65, '/odometry/global\nTF: map->odom',          _OC, fs=8)

    # Inputs -> fastlio
    _mpl_arrow(ax, 2.45, 4.8,  3.6,  4.55)
    _mpl_arrow(ax, 2.45, 3.95, 3.6,  4.2)
    # IMU -> EKF1
    _mpl_arrow(ax, 2.45, 3.75, 3.6,  3.1,  '/imu/data')
    # fastlio -> EKF1
    _mpl_arrow(ax, 5.0,  4.02, 5.0,  3.43, '/lidar_odometry')
    # EKF1 -> /odometry/filtered output
    _mpl_arrow(ax, 6.4,  3.4,  10.1, 3.95, '/odometry/filtered  (50 Hz)')
    # EKF1 -> navsat (heading init)
    _mpl_arrow(ax, 5.0,  2.77, 5.0,  2.18, '/odometry/filtered')
    # GPS -> navsat
    _mpl_arrow(ax, 2.45, 2.2,  3.6,  1.95, '/gps/fix')
    # IMU heading -> navsat
    _mpl_arrow(ax, 2.45, 1.2,  3.6,  1.65)
    # navsat -> EKF2
    _mpl_arrow(ax, 6.4,  1.85, 7.8,  2.3,  '/odometry/gps')
    # EKF1 -> EKF2 (heading)
    _mpl_arrow(ax, 6.4,  3.0,  7.8,  2.65, '/odometry/filtered', rad=0.15)
    # EKF2 -> /odometry/global output
    _mpl_arrow(ax, 10.6, 2.5,  10.1, 1.7,  '/odometry/global  (30 Hz)')

    plt.savefig(tmp.name, dpi=150, bbox_inches='tight',
                facecolor='white', edgecolor='none')
    plt.close()
    return tmp.name


# ─────────────────────────────────────────────────────────────────────────────
# Page template (header/footer)
# ─────────────────────────────────────────────────────────────────────────────
def make_page_template(canvas, doc):
    """Canvas hook: draws header bar and footer on every page."""
    canvas.saveState()
    W, H = A4

    # ── Header bar ────────────────────────────────────────────────────────────
    canvas.setFillColor(C_NAVY)
    canvas.rect(0, H - 1.2*cm, W, 1.2*cm, fill=1, stroke=0)
    canvas.setFont('Helvetica-Bold', 9)
    canvas.setFillColor(C_WHITE)
    canvas.drawString(1.5*cm, H - 0.85*cm, 'DIY Challenge Robot — Development & Operations Guide')
    canvas.drawRightString(W - 1.5*cm, H - 0.85*cm, f'Page {doc.page}')

    # ── Footer ────────────────────────────────────────────────────────────────
    canvas.setFont('Helvetica', 7.5)
    canvas.setFillColor(C_MGREY)
    canvas.drawCentredString(W/2, 0.7*cm,
        f'DIY-Challenge-Repo  |  ROS 2 Humble  |  Generated {date.today().isoformat()}')

    canvas.restoreState()


def make_title_page(canvas, doc):
    """Special first-page template with full navy background."""
    canvas.saveState()
    W, H = A4

    # Background
    canvas.setFillColor(C_NAVY)
    canvas.rect(0, 0, W, H, fill=1, stroke=0)

    # Accent stripe
    canvas.setFillColor(C_CYAN)
    canvas.rect(0, H*0.52, W, 4, fill=1, stroke=0)

    # Footer strip
    canvas.setFillColor(colors.HexColor('#0F1F3D'))
    canvas.rect(0, 0, W, 1.8*cm, fill=1, stroke=0)
    canvas.setFont('Helvetica', 7.5)
    canvas.setFillColor(C_MGREY)
    canvas.drawCentredString(W/2, 0.65*cm,
        f'DIY-Challenge-Repo  |  ROS 2 Humble  |  Generated {date.today().isoformat()}')

    canvas.restoreState()


# ─────────────────────────────────────────────────────────────────────────────
# Content builders
# ─────────────────────────────────────────────────────────────────────────────
def build_story(styles):
    story = []

    # ════════════════════════════════════════════════════════════════════════
    # COVER PAGE
    # ════════════════════════════════════════════════════════════════════════
    story.append(Spacer(1, 5*cm))
    story.append(H('DIY Challenge Robot', 'title', styles))
    story.append(Spacer(1, 0.4*cm))
    story.append(H('Development &amp; Operations Guide', 'subtitle', styles))
    story.append(Spacer(1, 0.3*cm))
    story.append(H('ROS 2 Humble  ·  Jetson Nano  ·  Hesai QT64  ·  FAST-LIO2  ·  Nav2', 'subtitle', styles))
    story.append(Spacer(1, 0.5*cm))
    story.append(H(f'Revision: {date.today().isoformat()}', 'subtitle', styles))
    story.append(PageBreak())

    # ════════════════════════════════════════════════════════════════════════
    # TABLE OF CONTENTS
    # ════════════════════════════════════════════════════════════════════════
    story.append(H('Table of Contents', 'h1', styles))
    story.append(hr(styles))
    toc_items = [
        ('1. Repository Structure &amp; Architecture', '3'),
        ('2. Package Reference', '5'),
        ('3. Configuration &amp; Calibration', '9'),
        ('4. Device Profiles', '12'),
        ('5. First-Time Setup on Robot Hardware', '14'),
        ('6. Calibration Procedures', '17'),
        ('7. Competition Day Operations', '21'),
        ('8. Developer Workflows (Laptop)', '24'),
        ('9. Scripts Reference', '26'),
        ('10. Troubleshooting', '29'),
        ('11. Extending the Codebase', '32'),
    ]
    for title, page in toc_items:
        row = Table(
            [[Paragraph(title, styles['toc']), Paragraph(page, styles['toc'])]],
            colWidths=[14*cm, 2*cm],
        )
        row.setStyle(TableStyle([
            ('ALIGN', (1,0), (1,0), 'RIGHT'),
            ('BOTTOMPADDING', (0,0), (-1,-1), 4),
            ('TOPPADDING', (0,0), (-1,-1), 4),
            ('LINEBELOW', (0,0), (-1,-1), 0.3, C_MGREY),
        ]))
        story.append(row)
    story.append(PageBreak())

    # ════════════════════════════════════════════════════════════════════════
    # SECTION 1 — REPOSITORY STRUCTURE & ARCHITECTURE
    # ════════════════════════════════════════════════════════════════════════
    story.append(H('1. Repository Structure &amp; Architecture', 'h1', styles))
    story.append(hr(styles))

    story.append(H('1.1 Workspace Layout', 'h2', styles))
    story.append(P(
        'The repository lives at <b>~/ros2_ws/src/DIY-Challenge-Repo/</b>. '
        'The directory is itself inside the colcon workspace so that '
        '<code>colcon build</code> run from <b>~/ros2_ws/</b> automatically '
        'discovers all packages inside <b>src/</b>.', styles))

    tree_text = """\
DIY-Challenge-Repo/
├── docs/                    ← SAD, installation guide, this PDF
├── profiles/                ← Per-device environment files
│   ├── jetson.env           ← Full hardware stack (competition)
│   ├── raspi.env            ← Lightweight relay/health role
│   └── laptop.env           ← Debug / replay / calibration
├── scripts/                 ← Shell scripts for every workflow
│   ├── env.sh               ← Environment bootstrap (source, not execute)
│   ├── run_robot.sh         ← Competition launch
│   ├── debug_robot.sh       ← Debug launch (+ RViz, verbose logs)
│   ├── record_bag.sh        ← Record all SAD topics to rosbag
│   ├── replay_bag.sh        ← Replay bag through software stack
│   ├── calibrate_imu.sh     ← Allan variance IMU calibration
│   ├── calibrate_extrinsics.sh ← Lidar↔IMU extrinsic calibration
│   ├── health_check.sh      ← Pre-flight hardware + TF validation
│   └── deploy_bundle.sh     ← Push calibration/map bundle to robot
├── calibration/             ← Created by calibration scripts; .gitignored
├── src/                     ← ROS 2 packages (colcon root)
│   ├── challenge_bringup/   ← Top-level launch + Nav2/map config
│   ├── diy_cmd_vel_mux/     ← cmd_vel arbitration node
│   ├── diy_estop_controller/← STM32 E-stop mirror node
│   ├── robot_description/← URDF / TF tree
│   └── localization/    ← FAST-LIO2 + EKF1 + EKF2 + navsat
└── third_party_ws/          ← Pre-built SLAM & sensor packages
    ├── src/FAST_LIO/
    ├── src/LIO-SAM/
    ├── src/imu_utils_ros2_humble/
    ├── src/lidar_imu_calib/
    ├── src/ndt_omp_ros2/
    └── src/livox_ros_driver2/"""
    story += code_block('Directory tree', tree_text, styles)

    story.append(H('1.2 System Architecture Overview', 'h2', styles))
    story.append(P(
        'The system is divided into three compute roles. Each role loads a '
        'matching profile which enables the appropriate set of drivers and '
        'nodes.', styles))

    roles_data = [
        ['Role', 'Hardware', 'Responsibilities'],
        ['Jetson Nano\n(competition)', 'QT64 + D435i + GPS\n+ STM32 via micro-ROS',
         'Lidar driver → FAST-LIO2 → EKF\n→ Nav2 → cmd_vel_mux → motors'],
        ['Raspberry Pi\n(optional)', 'STM32 via micro-ROS\n+ GPS relay',
         'Lightweight E-stop bridge &\nhealth monitoring only'],
        ['Laptop\n(developer)', 'No hardware\n(bag replay or sim)',
         'Debug, calibration, offline\nmapping, bag analysis'],
    ]
    roles_table = Table(roles_data, colWidths=[3.5*cm, 5*cm, 8*cm])
    roles_table.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), C_NAVY),
        ('TEXTCOLOR',  (0,0), (-1,0), C_WHITE),
        ('FONTNAME',   (0,0), (-1,0), 'Helvetica-Bold'),
        ('FONTSIZE',   (0,0), (-1,-1), 9),
        ('ROWBACKGROUNDS', (0,1), (-1,-1), [C_WHITE, C_LGREY]),
        ('GRID', (0,0), (-1,-1), 0.4, C_MGREY),
        ('VALIGN', (0,0), (-1,-1), 'TOP'),
        ('TOPPADDING', (0,0), (-1,-1), 5),
        ('BOTTOMPADDING', (0,0), (-1,-1), 5),
        ('LEFTPADDING', (0,0), (-1,-1), 6),
    ]))
    story.append(roles_table)
    story.append(Spacer(1, 6))

    story.append(H('1.3 Node Communication Graph', 'h2', styles))
    story.append(P(
        'Colour key: '
        '<font color="#0EA5E9"><b>blue = sensors</b></font>  '
        '<font color="#16A34A"><b>green = SLAM/EKF</b></font>  '
        '<font color="#EA580C"><b>orange = control</b></font>  '
        '<font color="#7C3AED"><b>purple = Nav2</b></font>  '
        '<font color="#374151"><b>dark = actuators</b></font>. '
        'Arrows are labelled with the ROS topic.', styles))
    _arch_png = _make_arch_diagram()
    story.append(RLImage(_arch_png, width=16.5*cm, height=10.0*cm))
    story.append(Spacer(1, 6))
    story.append(PageBreak())

    # ════════════════════════════════════════════════════════════════════════
    # SECTION 2 — PACKAGE REFERENCE
    # ════════════════════════════════════════════════════════════════════════
    story.append(H('2. Package Reference', 'h1', styles))
    story.append(hr(styles))

    # ── 2.1 challenge_bringup ─────────────────────────────────────────────
    story.append(H('2.1  challenge_bringup', 'h2', styles))
    story.append(P(
        '<b>Type:</b> ament_cmake (launch + config only) | '
        '<b>Depends on:</b> nav2_bringup, all diy_* packages', styles))
    story.append(P(
        'Top-level bringup package. Contains the master launch file and all '
        'Nav2 / collision-monitor configuration. This is the single entry '
        'point for every deployment — run_robot.sh calls '
        '<code>ros2 launch challenge_bringup challenge_master.launch.py</code>.', styles))

    story.append(H('Key files', 'h3', styles))
    pkg_files = [
        ['File', 'Purpose'],
        ['launch/challenge_master.launch.py', 'Top-level launch; accepts all DIY_* flags as args'],
        ['launch/joystick_drive.launch.py', 'Joystick teleop sub-launch (joy_node + teleop_twist_joy)'],
        ['config/nav2_params.yaml', 'Nav2: costmaps, Smac planner, MPPI controller, behavior server'],
        ['config/collision_monitor_params.yaml', 'Collision monitor safety zones'],
        ['maps/static_map.yaml', 'Competition course static map (replace with your map)'],
    ]
    tbl = Table(pkg_files, colWidths=[7*cm, 9.5*cm])
    tbl.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), C_BLUE),
        ('TEXTCOLOR',  (0,0), (-1,0), C_WHITE),
        ('FONTNAME',   (0,0), (-1,0), 'Helvetica-Bold'),
        ('FONTSIZE',   (0,0), (-1,-1), 9),
        ('ROWBACKGROUNDS', (0,1), (-1,-1), [C_WHITE, C_LGREY]),
        ('GRID', (0,0), (-1,-1), 0.3, C_MGREY),
        ('TOPPADDING', (0,0), (-1,-1), 4),
        ('BOTTOMPADDING', (0,0), (-1,-1), 4),
        ('LEFTPADDING', (0,0), (-1,-1), 5),
        ('VALIGN', (0,0), (-1,-1), 'TOP'),
    ]))
    story.append(tbl)
    story.append(Spacer(1, 8))

    # ── 2.2 diy_cmd_vel_mux ───────────────────────────────────────────────
    story.append(H('2.2  diy_cmd_vel_mux', 'h2', styles))
    story.append(P(
        '<b>Type:</b> ament_python | <b>Node:</b> cmd_vel_mux_node | '
        '<b>Executable:</b> cmd_vel_mux_node', styles))
    story.append(P(
        'Arbitrates command velocity from joystick teleop, Nav2, and an '
        'E-stop signal into a single safe /cmd_vel_safe output. The motor '
        'driver <i>only</i> reads /cmd_vel_safe — it never sees Nav2 or '
        'joystick topics directly.', styles))

    story.append(H('Topics', 'h3', styles))
    mux_topics = [
        ['Topic', 'Type', 'Direction', 'Description'],
        ['/cmd_vel_joy', 'geometry_msgs/Twist', 'SUB', 'Joystick teleop output'],
        ['/cmd_vel_nav', 'geometry_msgs/Twist', 'SUB', 'Nav2 FollowPath controller'],
        ['/estop_active', 'std_msgs/Bool', 'SUB', 'Advisory E-stop from diy_estop_controller'],
        ['/cmd_vel_safe', 'geometry_msgs/Twist', 'PUB', 'To motor driver (always safe/zero on E-stop)'],
        ['/mux_mode', 'std_msgs/String', 'PUB', 'Current mode: JOYSTICK | AUTONOMOUS | ESTOP_LOCK'],
    ]
    t = Table(mux_topics, colWidths=[4.5*cm, 4.5*cm, 2*cm, 5.5*cm])
    t.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), C_BLUE),
        ('TEXTCOLOR',  (0,0), (-1,0), C_WHITE),
        ('FONTNAME',   (0,0), (-1,0), 'Helvetica-Bold'),
        ('FONTSIZE',   (0,0), (-1,-1), 8.5),
        ('ROWBACKGROUNDS', (0,1), (-1,-1), [C_WHITE, C_LGREY]),
        ('GRID', (0,0), (-1,-1), 0.3, C_MGREY),
        ('TOPPADDING', (0,0), (-1,-1), 4),
        ('BOTTOMPADDING', (0,0), (-1,-1), 4),
        ('LEFTPADDING', (0,0), (-1,-1), 5),
    ]))
    story.append(t)
    story.append(Spacer(1, 4))
    story.append(P(
        '<b>Runtime mode switch:</b> '
        '<code>ros2 param set /cmd_vel_mux_node mode AUTONOMOUS</code> '
        '(or JOYSTICK). ESTOP_LOCK cannot be set manually.', styles))

    # ── 2.3 diy_estop_controller ─────────────────────────────────────────
    story.append(H('2.3  diy_estop_controller', 'h2', styles))
    story.append(P(
        '<b>Type:</b> ament_python | <b>Node:</b> estop_controller_node', styles))
    story.append(P(
        'Mirrors the STM32 hardware E-stop state into the ROS 2 graph. '
        '<b>Important:</b> This node is advisory only — the actual motor cut '
        'happens in STM32 firmware before any ROS message is even sent. The '
        'dual-watchdog design (explicit flag + heartbeat) means the software '
        'stack goes safe even if the micro-ROS bridge drops.', styles))
    story.append(warn_box(
        'The STM32 heartbeat timeout is 800 ms. If micro_ros_agent crashes or '
        'the USB cable is unplugged, /estop_active will go True within 800 ms '
        'and cmd_vel_mux will publish zero velocity.', styles))

    # ── 2.4 robot_description ─────────────────────────────────────────
    story.append(H('2.4  robot_description', 'h2', styles))
    story.append(P(
        '<b>Type:</b> ament_cmake | Provides: URDF Xacro + static TF tree', styles))
    story.append(P(
        'Defines the robot body geometry and all sensor placements as URDF '
        'joints. Currently all joint transforms are placeholder identity '
        'values marked <b>CALIB:</b> — they must be updated with real '
        'calibrated extrinsics before SLAM will produce accurate maps.', styles))
    story.append(calib_box(
        'All joint xyz/rpy in urdf/robot.urdf.xacro are set to identity. '
        'Run calibrate_extrinsics.sh and copy the resulting transforms into '
        'the URDF before any mapping session.', styles))

    # ── 2.5 localization ─────────────────────────────────────────────
    story.append(H('2.5  localization', 'h2', styles))
    story.append(P(
        '<b>Type:</b> ament_cmake (configs + launch) | '
        '<b>Depends on:</b> fast_lio, robot_localization, robot_description', styles))
    story.append(P(
        'Launches the full localisation stack: FAST-LIO2 lidar-inertial '
        'odometry feeding two robot_localization EKF instances and a '
        'navsat_transform node for GPS fusion.', styles))

    story.append(H('Localisation pipeline', 'h3', styles))
    story.append(P(
        'Input topics (dark blue) feed FAST-LIO2 and the EKF nodes (green). '
        'Outputs (teal) are the topics and TF transforms consumed by Nav2.', styles))
    _loc_png = _make_loc_pipeline()
    story.append(RLImage(_loc_png, width=16.5*cm, height=7.0*cm))
    story.append(Spacer(1, 6))
    story.append(PageBreak())

    # ════════════════════════════════════════════════════════════════════════
    # SECTION 3 — CONFIGURATION & CALIBRATION FILES
    # ════════════════════════════════════════════════════════════════════════
    story.append(H('3. Configuration &amp; Calibration', 'h1', styles))
    story.append(hr(styles))

    story.append(H('3.1  FAST-LIO2 Config: fast_lio_hesai_qt64.yaml', 'h2', styles))
    story.append(P(
        'Located at <b>src/localization/config/fast_lio_hesai_qt64.yaml</b>. '
        'Configures FAST-LIO2 for the Hesai QT64 lidar.', styles))

    fastlio_params = [
        ['Parameter', 'Value', 'Notes'],
        ['lidar_type', '2', 'PointCloud2 format (non-Livox Velodyne-style)'],
        ['scan_line', '64', 'QT64 has 64 beam channels'],
        ['scan_rate', '20', '20 Hz output rate from QT64'],
        ['blind', '0.3', 'Ignore returns closer than 0.3 m (near-field noise)'],
        ['det_range', '80.0', 'Maximum effective range in metres'],
        ['extrinsic_T', 'CALIB: [0,0,0]', 'Lidar→IMU translation; update after calibration'],
        ['extrinsic_R', 'CALIB: identity', 'Lidar→IMU rotation matrix; update after calibration'],
        ['extrinsic_est_en', 'true', 'Online estimation active until lidar_imu_calib completes'],
        ['dense_publish_en', 'false', 'Bandwidth optimisation for Jetson Nano'],
    ]
    t = Table(fastlio_params, colWidths=[4*cm, 3.5*cm, 9*cm])
    t.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), C_NAVY),
        ('TEXTCOLOR',  (0,0), (-1,0), C_WHITE),
        ('FONTNAME',   (0,0), (-1,0), 'Helvetica-Bold'),
        ('FONTSIZE',   (0,0), (-1,-1), 8.5),
        ('ROWBACKGROUNDS', (0,1), (-1,-1), [C_WHITE, C_LGREY]),
        ('GRID', (0,0), (-1,-1), 0.3, C_MGREY),
        ('TOPPADDING', (0,0), (-1,-1), 3),
        ('BOTTOMPADDING', (0,0), (-1,-1), 3),
        ('LEFTPADDING', (0,0), (-1,-1), 5),
        ('VALIGN', (0,0), (-1,-1), 'TOP'),
    ]))
    story.append(t)

    story.append(H('3.2  EKF1 Local: ekf_local.yaml', 'h2', styles))
    story.append(P(
        'Fuses FAST-LIO2 lidar odometry (50 Hz) with IMU angular velocity '
        '(400 Hz) in the <b>odom frame</b>. Output: /odometry/filtered at '
        '50 Hz — the primary odometry source for Nav2 local MPPI controller.', styles))
    story += B([
        '<b>odom0</b>: /lidar_odometry — x, y, yaw, vx, vy, vyaw used',
        '<b>imu0</b>: /imu/data — gyroscope angular velocity only (vroll, vpitch, vyaw)',
        '<b>two_d_mode: true</b> — flattens to 2D for ground robot',
        '<b>frequency: 50 Hz</b> — matches FAST-LIO2 output rate',
    ], styles)

    story.append(H('3.3  EKF2 Global: ekf_global.yaml', 'h2', styles))
    story.append(P(
        'Fuses /odometry/filtered with GPS position in the <b>map frame</b>. '
        'Output: /odometry/global at 30 Hz — provides absolute position '
        'correction for long-range navigation.', styles))
    story.append(note_box(
        'EKF2 only produces useful output when GPS has a valid fix (HDOP < 2.0). '
        'In GPS-denied environments (indoors / under trees), FAST-LIO2\'s internal '
        'PGO provides the map→odom transform instead.', styles))

    story.append(H('3.4  Nav2 Configuration: nav2_params.yaml', 'h2', styles))
    nav2_params = [
        ['Component', 'Plugin / Setting', 'Key Value'],
        ['Global planner', 'SmacPlannerHybrid', 'tolerance: 0.25 m'],
        ['Local controller', 'MPPIController', '40 steps × 50 ms, batch 1000, vx_max 0.6 m/s'],
        ['MPPI critics', '8 critics', 'ObstaclesCritic, PathAlign, GoalCritic, ...'],
        ['Global costmap', 'static+obstacle+inflation', 'Inflation: 0.45 m from /hesai/points'],
        ['Local costmap', 'voxel+inflation', '4 m × 4 m rolling, 0.05 m/cell'],
        ['Behavior server', 'spin, backup, drive, wait', 'Recovery behaviours for Nav2 BT'],
        ['BT Navigator', 'navigate_w_replanning_and_recovery', 'Default recovery BT'],
    ]
    t = Table(nav2_params, colWidths=[4*cm, 5*cm, 7.5*cm])
    t.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), C_NAVY),
        ('TEXTCOLOR',  (0,0), (-1,0), C_WHITE),
        ('FONTNAME',   (0,0), (-1,0), 'Helvetica-Bold'),
        ('FONTSIZE',   (0,0), (-1,-1), 8.5),
        ('ROWBACKGROUNDS', (0,1), (-1,-1), [C_WHITE, C_LGREY]),
        ('GRID', (0,0), (-1,-1), 0.3, C_MGREY),
        ('TOPPADDING', (0,0), (-1,-1), 3),
        ('BOTTOMPADDING', (0,0), (-1,-1), 3),
        ('LEFTPADDING', (0,0), (-1,-1), 5),
    ]))
    story.append(t)
    story.append(PageBreak())

    # ════════════════════════════════════════════════════════════════════════
    # SECTION 4 — DEVICE PROFILES
    # ════════════════════════════════════════════════════════════════════════
    story.append(H('4. Device Profiles', 'h1', styles))
    story.append(hr(styles))

    story.append(P(
        'The profiles system makes the same repository runnable on three '
        'different hardware configurations without modifying any source file. '
        'Each profile is a Bash environment file in <b>profiles/</b> that '
        'exports <code>DIY_*</code> variables.', styles))

    story.append(H('How profiles work', 'h2', styles))
    story += code_block('Loading a profile manually', """\
# Load the Jetson profile before launching:
source ~/ros2_ws/src/DIY-Challenge-Repo/profiles/jetson.env

# Or let env.sh auto-detect from hostname:
source ~/ros2_ws/src/DIY-Challenge-Repo/scripts/env.sh""", styles)

    story.append(H('Profile environment variables', 'h2', styles))
    env_vars = [
        ['Variable', 'Type', 'Purpose'],
        ['DIY_ROBOT_PROFILE', 'string', 'Profile name: jetson | raspi | laptop'],
        ['DIY_USE_HESAI', 'bool', 'Enable Hesai QT64 lidar driver node'],
        ['DIY_USE_REALSENSE', 'bool', 'Enable RealSense D435i camera node'],
        ['DIY_USE_GPS', 'bool', 'Enable RTK GPS & navsat_transform'],
        ['DIY_USE_MICRO_ROS', 'bool', 'Enable micro-ROS agent (STM32 bridge)'],
        ['DIY_USE_MOTOR_DRIVER', 'bool', 'Enable differential-drive motor node'],
        ['DIY_USE_NAV2', 'bool', 'Enable Nav2 autonomous navigation stack'],
        ['DIY_USE_LOCALIZATION', 'bool', 'Enable FAST-LIO2 + EKF1/EKF2'],
        ['DIY_MUX_MODE', 'string', 'Startup mode for cmd_vel_mux: JOYSTICK | AUTONOMOUS'],
        ['DIY_FASTLIO_CONFIG', 'string', 'YAML filename in localization/config/'],
        ['DIY_HESAI_IP', 'string', 'Lidar IP address (default: 192.168.1.201)'],
        ['DIY_MICRO_ROS_SERIAL', 'string', 'Serial device for micro-ROS (e.g. /dev/ttyACM0)'],
        ['DIY_BAG_OUTPUT_DIR', 'path', 'Directory for rosbag recordings'],
        ['DIY_BAG_SPLIT_DURATION', 'int', 'Bag split interval in seconds'],
    ]
    t = Table(env_vars, colWidths=[5.5*cm, 2*cm, 9*cm])
    t.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), C_NAVY),
        ('TEXTCOLOR',  (0,0), (-1,0), C_WHITE),
        ('FONTNAME',   (0,0), (-1,0), 'Helvetica-Bold'),
        ('FONTSIZE',   (0,0), (-1,-1), 8.5),
        ('ROWBACKGROUNDS', (0,1), (-1,-1), [C_WHITE, C_LGREY]),
        ('GRID', (0,0), (-1,-1), 0.3, C_MGREY),
        ('TOPPADDING', (0,0), (-1,-1), 3),
        ('BOTTOMPADDING', (0,0), (-1,-1), 3),
        ('LEFTPADDING', (0,0), (-1,-1), 5),
        ('VALIGN', (0,0), (-1,-1), 'TOP'),
    ]))
    story.append(t)
    story.append(PageBreak())

    # ════════════════════════════════════════════════════════════════════════
    # SECTION 5 — FIRST-TIME SETUP ON ROBOT HARDWARE
    # ════════════════════════════════════════════════════════════════════════
    story.append(H('5. First-Time Setup on Robot Hardware', 'h1', styles))
    story.append(hr(styles))

    story.append(H('5.1  Prerequisites', 'h2', styles))
    story += B([
        'Ubuntu 22.04 (Jetpack 5.x for Jetson Nano) with ROS 2 Humble installed',
        'micro_ros_ws built at ~/micro_ros_ws (follow micro-ROS Humble build guide)',
        'HesaiLidar_ROS_2.0 installed at ~/ros2_ws/src/Betsybot-Software/src/HesaiLidar_ROS_2.0/',
        'Git and colcon-common-extensions installed',
        'Python 3 with pip',
        'USB permissions for STM32: user added to dialout group (<code>sudo usermod -aG dialout $USER</code>)',
        'Ethernet interface configured for lidar: static IP 192.168.1.100/24 on the lidar port',
    ], styles)

    story.append(H('5.2  Clone and build (first time)', 'h2', styles))
    story.append(P(
        'The repository uses <b>git submodules</b> for all six third-party '
        'packages (FAST-LIO2, LIO-SAM, imu_utils, lidar_imu_calib, '
        'livox_ros_driver2, ndt_omp_ros2). You must clone with '
        '<code>--recurse-submodules</code> to pull them in, or the '
        'third-party workspace build will fail with missing source errors.', styles))
    story += code_block('Option A — automated setup (recommended)', """\
# 1. Clone the repo WITH its submodules in one command
cd ~/ros2_ws/src
git clone --recurse-submodules \\
    https://github.com/vasu536/DIY-Challenge-Repo.git

# 2. Run the setup script — handles patches, rosdep, and all colcon builds
cd DIY-Challenge-Repo
bash setup.sh jetson          # or: bash setup.sh laptop / raspi

# 3. After setup.sh completes, source the environment
source scripts/env.sh jetson

# 4. Verify everything is running
scripts/health_check.sh jetson""", styles)
    story.append(note_box(
        'setup.sh does the following automatically: initialises submodules, '
        'applies the three CMakeLists.txt build-fix patches in patches/, '
        'runs rosdep install, builds third_party_ws, then builds all five '
        'DIY packages. It is safe to run multiple times.', styles))
    story += code_block('Option B — manual step-by-step (if setup.sh fails)', """\
# 1. Clone with submodules
cd ~/ros2_ws/src
git clone --recurse-submodules \\
    https://github.com/vasu536/DIY-Challenge-Repo.git
cd DIY-Challenge-Repo

# 2. Apply build-fix patches to three third-party packages
git -C third_party_ws/src/lidar_imu_calib  apply ../../patches/lidar_imu_calib.patch
git -C third_party_ws/src/livox_ros_driver2 apply ../../patches/livox_ros_driver2.patch
git -C third_party_ws/src/ndt_omp_ros2     apply ../../patches/ndt_omp_ros2.patch

# 3. Source ROS 2 and install dependencies
source /opt/ros/humble/setup.bash
rosdep install --from-paths third_party_ws/src src --ignore-src -r -y

# 4. Build third-party workspace
cd third_party_ws && colcon build --symlink-install && cd ..

# 5. Source the full environment (Jetson profile)
source scripts/env.sh jetson

# 6. Build all DIY packages
cd ~/ros2_ws
colcon build --symlink-install \\
    --packages-select \\
        challenge_bringup \\
        diy_cmd_vel_mux \\
        diy_estop_controller \\
        robot_description \\
        localization

# 7. Run preflight health check
~/ros2_ws/src/DIY-Challenge-Repo/scripts/health_check.sh jetson""", styles)
    story.append(warn_box(
        'If you cloned WITHOUT --recurse-submodules, the third_party_ws/src/ '
        'directories will be empty. Fix it by running: '
        'git submodule update --init --recursive', styles))

    story.append(H('5.3  Source overlay order (CRITICAL)', 'h2', styles))
    story.append(P(
        'ROS 2 workspaces must be sourced in the correct order — later '
        'sources override earlier ones. If you source out of order, nodes '
        'from the wrong workspace may be invoked.', styles))
    story += code_block('Correct overlay sequence (env.sh handles this automatically)', """\
1. source /opt/ros/humble/setup.bash                          # ROS 2 base
2. source ~/micro_ros_ws/install/setup.bash                   # micro-ROS
3. source ~/ros2_ws/src/Betsybot-Software/src/HesaiLidar_ROS_2.0/install/setup.bash
4. source ~/ros2_ws/src/DIY-Challenge-Repo/third_party_ws/install/setup.bash
5. source ~/ros2_ws/install/setup.bash                        # DIY packages""", styles)
    story.append(note_box(
        'scripts/env.sh performs all 5 steps automatically when sourced. '
        'Never manually source individual setup.bash files for production use.', styles))

    story.append(H('5.4  Network configuration', 'h2', styles))
    story += code_block('Hesai QT64 Ethernet setup (example for eth0)', """\
# Set a static IP on the Jetson's lidar ethernet port
sudo ip addr add 192.168.1.100/24 dev eth0
sudo ip link set eth0 up

# Verify the lidar is reachable (default QT64 IP: 192.168.1.201)
ping 192.168.1.201

# To make permanent, add to /etc/netplan/01-netcfg.yaml:
#   network:
#     ethernets:
#       eth0:
#         addresses: [192.168.1.100/24]""", styles)

    story.append(H('5.5  micro-ROS agent: build and first-time setup', 'h2', styles))
    story.append(P(
        'The micro-ROS agent bridges the STM32 firmware (which runs a micro-ROS '
        'executor) to the ROS 2 network over a USB serial link. It must be built '
        'once as a separate workspace and is started automatically by '
        '<b>challenge_master.launch.py</b> (when <code>use_micro_ros:=true</code>). '
        'This section covers the one-time build and optional persistence setup.', styles))

    story.append(H('Build micro_ros_ws (one-time)', 'h3', styles))
    story += code_block('Build the micro-ROS agent workspace', """\
# 1. Create the workspace
mkdir -p ~/micro_ros_ws/src
cd ~/micro_ros_ws

# 2. Clone the micro-ROS agent for ROS 2 Humble
git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# 3. Source ROS 2 and install dependencies
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -y

# 4. Build the setup tools
colcon build --symlink-install
source install/setup.bash

# 5. Create and build the micro-ROS agent
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/setup.bash

# 6. Verify the agent binary is available
ros2 run micro_ros_agent micro_ros_agent --help | head -5""", styles)

    story.append(H('USB permissions (one-time)', 'h3', styles))
    story.append(P(
        'The STM32 appears as <code>/dev/ttyACM0</code> (or similar). '
        'Add your user to the <b>dialout</b> group so the agent can open it '
        'without sudo. This only needs to be done once; log out and back in '
        'afterwards for the group change to take effect.', styles))
    story += code_block('Grant serial port access', """\
sudo usermod -aG dialout $USER

# Log out and back in, then verify:
groups | grep dialout

# Identify which device node the STM32 uses:
ls /dev/ttyACM* /dev/ttyUSB*
# (re-plug the USB cable and watch dmesg if the device does not appear)
dmesg | tail -5""", styles)

    story.append(H('Test: verify STM32 topics appear', 'h3', styles))
    story += code_block('Manual test before using the launch file', """\
# Terminal 1 — start the agent manually
source ~/micro_ros_ws/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 921600

# Terminal 2 — check that the STM32 topics appear (within ~2 seconds)
source ~/ros2_ws/src/DIY-Challenge-Repo/scripts/env.sh jetson
ros2 topic list | grep stm32
# Expected output:
#   /stm32/heartbeat
#   /stm32/estop

# Verify heartbeat is publishing at ~10 Hz
ros2 topic hz /stm32/heartbeat""", styles)

    story.append(H('Optional: run agent as a systemd service', 'h3', styles))
    story.append(P(
        'For competition use, running the agent as a systemd service means it '
        'starts automatically at boot and restarts if it crashes — no manual '
        'terminal needed.', styles))
    story += code_block('Create /etc/systemd/system/micro-ros-agent.service', """\
[Unit]
Description=micro-ROS Agent (STM32 bridge)
After=network.target

[Service]
User=YOUR_USERNAME
ExecStart=/bin/bash -c \\
    "source /opt/ros/humble/setup.bash && \\
     source /home/YOUR_USERNAME/micro_ros_ws/install/setup.bash && \\
     ros2 run micro_ros_agent micro_ros_agent serial \\
         --dev /dev/ttyACM0 -b 921600"
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target""", styles)
    story += code_block('Enable and start the service', """\
# Replace YOUR_USERNAME in the service file first, then:
sudo systemctl daemon-reload
sudo systemctl enable micro-ros-agent.service
sudo systemctl start  micro-ros-agent.service

# Check status
sudo systemctl status micro-ros-agent.service""", styles)
    story.append(note_box(
        'When the agent runs as a systemd service, the challenge_master.launch.py '
        'node block for micro_ros_agent is redundant but harmless — the launch '
        'file will simply fail to bind the serial port (already in use) and '
        'the system will continue normally via the service. To avoid the '
        'conflict, set use_micro_ros:=false in the launch args when the '
        'service is active.', styles))

    story.append(H('5.6  Accessing the Jetson Nano (SSH / headless setup)', 'h2', styles))
    story.append(P(
        'For competition use the Jetson Nano typically runs headless (no monitor). '
        'All interaction is over SSH from a laptop on the same local network.', styles))

    story.append(H('First-time physical setup', 'h3', styles))
    story += B([
        '<b>JetPack 5.x default credentials</b>: username <code>ubuntu</code>, '
        'password <code>ubuntu</code> (change immediately after first login)',
        'First boot requires a monitor + keyboard to complete the initial Ubuntu wizard '
        '— set hostname to <code>jetson</code> and enable OpenSSH server',
        'Alternative: serial console via the 40-pin header UART (115200 baud, pins 8/10/6) '
        'if no monitor is available',
    ], styles)

    story.append(H('Finding the Jetson IP and connecting', 'h3', styles))
    story += code_block('SSH into the Jetson Nano', """\
# The Jetson advertises itself via mDNS as jetson.local (same LAN / WiFi)
ping jetson.local

# If mDNS does not work, scan the subnet for the Jetson
nmap -sn 192.168.1.0/24 | grep -A2 "NVIDIA\\|jetson"

# Connect (replace jetson.local with the IP if nmap was needed)
ssh ubuntu@jetson.local

# First time: change the default password immediately
passwd

# Verify ROS 2 environment is reachable
source /opt/ros/humble/setup.bash && ros2 node list""", styles)

    story.append(H('Passwordless SSH (recommended for scripts)', 'h3', styles))
    story += code_block('Set up key-based authentication from your laptop', """\
# On your LAPTOP — generate a key if you do not have one already
ssh-keygen -t ed25519 -C "diy-challenge"

# Copy the public key to the Jetson
ssh-copy-id ubuntu@jetson.local

# Verify: this should open a shell with no password prompt
ssh ubuntu@jetson.local""", styles)

    story.append(H('Optional: VNC desktop session', 'h3', styles))
    story += code_block('Enable VNC for a full GUI via RealVNC or TigerVNC', """\
# On the Jetson: enable the built-in VNC server (JetPack 5.x includes it)
sudo systemctl enable vncserver-x11-serviced.service
sudo systemctl start  vncserver-x11-serviced.service

# On your laptop: connect with any VNC client to
#   jetson.local:5900  (password set via the VNC app on first run)
# Or tunnel via SSH for security:
ssh -L 5900:localhost:5900 ubuntu@jetson.local
# then connect your VNC client to localhost:5900""", styles)

    story.append(H('5.7  Jetson Nano performance: power mode, clocks, and GPU', 'h2', styles))

    story.append(H('Does the navigation stack use the GPU?', 'h3', styles))
    story.append(P(
        'Short answer: <b>no</b> — the current navigation pipeline runs entirely on the '
        'CPU. FAST-LIO2 uses an ikd-Tree with Eigen-based Kalman filter iterations; '
        'LIO-SAM uses GTSAM factor graphs with OpenMP-parallelised CPU threads; '
        'Nav2 and the EKF nodes are single-threaded ROS 2 processes. '
        'The Jetson\'s 128-core Maxwell GPU (CUDA 11.x) is idle during normal '
        'robot operation. It would only be used if you added a neural-network '
        'inference node (e.g. YOLO for obstacle detection via TensorRT).', styles))
    story.append(P(
        'Despite the GPU not being used by the stack, <b>performance tuning is '
        'still critical</b>: the quad-core Cortex-A57 runs FAST-LIO2 at ~50 Hz '
        'and all Nav2 planners simultaneously. Without locking the clocks to '
        'maximum the CPU will throttle and introduce planning jitter.', styles))

    story.append(H('Set maximum performance mode (nvpmodel + jetson_clocks)', 'h3', styles))
    story += code_block('Run before every competition attempt', """\
# Show current power mode (default at boot is often 5W / mode 1)
sudo nvpmodel -q

# Switch to MAXN (10W) mode — uses all 4 CPU cores at full speed
sudo nvpmodel -m 0

# Lock CPU, GPU, and EMC clocks to their maximum
sudo jetson_clocks

# Verify: CPU frequencies should all be at max (e.g. 1.479 GHz for Nano)
cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_cur_freq

# Check overall thermal + power state with jtop (install once: pip3 install jetson-stats)
jtop""", styles)
    story.append(warn_box(
        'nvpmodel and jetson_clocks settings reset on reboot. Add them to a '
        'systemd service or to /etc/rc.local so they are applied automatically '
        'at startup — especially important on competition day.', styles))

    story.append(H('Make max-performance persistent across reboots', 'h3', styles))
    story += code_block('Create /etc/systemd/system/jetson-maxperf.service', """\
[Unit]
Description=Set Jetson to MAXN power mode and lock clocks
After=multi-user.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/bin/sh -c "nvpmodel -m 0 && jetson_clocks"

[Install]
WantedBy=multi-user.target""", styles)
    story += code_block('Enable the service', """\
sudo systemctl daemon-reload
sudo systemctl enable jetson-maxperf.service
sudo systemctl start  jetson-maxperf.service

# Check it ran successfully
sudo systemctl status jetson-maxperf.service""", styles)

    story.append(H('Add a swap file (recommended; Jetson Nano has only 4 GB RAM)', 'h3', styles))
    story += code_block('Create a 4 GB swap file — run once', """\
# Create the swap file
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

# Make permanent across reboots
echo '/swapfile swap swap defaults 0 0' | sudo tee -a /etc/fstab

# Verify
free -h | grep Swap""", styles)
    story.append(note_box(
        'The swap file is especially helpful during colcon builds of FAST-LIO2 '
        'and LIO-SAM. At runtime the system should stay within physical RAM; '
        'if tegrastats shows heavy swap usage during operation the robot has '
        'too many nodes running simultaneously.', styles))

    story.append(H('Monitor CPU/GPU during a run', 'h3', styles))
    story += code_block('Real-time monitoring tools', """\
# tegrastats (built-in, prints every 1 second)
tegrastats --interval 1000

# jtop (rich TUI dashboard — install once)
pip3 install jetson-stats
sudo systemctl restart jtop.service   # needed after install
jtop

# Key things to watch in tegrastats output:
#   RAM  xxx/3964MB  — keep below ~3 GB during operation
#   CPU  [xx% xx% xx% xx%]  — should all be near 100% during SLAM
#   GPU  xx%  — will be near 0% (navigation stack is CPU-only)
#   Temp SOC xx.xC  — keep below 80°C; throttling starts at 85°C""", styles)

    story.append(H('Verify CUDA installation (optional)', 'h3', styles))
    story += code_block('Confirm CUDA 11.x is present from JetPack', """\
# Check CUDA compiler version
nvcc --version
# Expected: Cuda compilation tools, release 11.4 (or similar JetPack 5.x version)

# Check GPU is visible to CUDA
python3 -c "import subprocess; subprocess.run(['nvidia-smi'])"
# Or:
ls /dev/nvidia*    # should list /dev/nvidia0

# If you later add TensorRT / YOLO inference:
python3 -c "import tensorrt; print(tensorrt.__version__)"
python3 -c "import torch; print(torch.cuda.is_available())" """, styles)

    story.append(PageBreak())

    # ════════════════════════════════════════════════════════════════════════
    # SECTION 6 — CALIBRATION PROCEDURES
    # ════════════════════════════════════════════════════════════════════════
    story.append(H('6. Calibration Procedures', 'h1', styles))
    story.append(hr(styles))

    story.append(P(
        'Calibration is required before the first mapping session and whenever '
        'a sensor is physically moved or replaced. The scripts in scripts/ '
        'automate each step. Always calibrate in this order:', styles))
    story += B([
        'Step 1: IMU Allan Variance (calibrate_imu.sh) — 2–3 hours',
        'Step 2: Lidar↔IMU extrinsics (calibrate_extrinsics.sh) — 30 minutes',
        'Step 3: Update YAML + URDF with results',
        'Step 4: Prior map generation session (offline_mapping.launch.py)',
    ], styles)
    story.append(Spacer(1,4))

    story.append(H('6.1  IMU Allan Variance Calibration', 'h2', styles))
    story.append(P(
        'Measures IMU noise density (white noise σ) and bias instability '
        '(random walk) from a 2–3 hour static recording. Outputs are used '
        'in FAST-LIO2 config as <b>gyr_cov</b>, <b>acc_cov</b>, '
        '<b>b_gyr_cov</b>, <b>b_acc_cov</b>.', styles))
    story += code_block('Run IMU calibration', """\
# Ensure robot is on a stable, vibration-free surface (e.g. foam pad)
# Hesai lidar and all motors must be OFF during recording

source ~/ros2_ws/src/DIY-Challenge-Repo/scripts/env.sh jetson

# Start the IMU source (micro-ROS agent must be running)
# Then in a second terminal:
./scripts/calibrate_imu.sh jetson --duration 7200

# After completion, find the result YAML in calibration/
# Copy gyr_n, gyr_w, acc_n, acc_w into:
#   src/localization/config/fast_lio_hesai_qt64.yaml""", styles)
    story.append(calib_box(
        'After IMU calibration, set extrinsic_est_en: false only AFTER '
        'completing Step 2 (extrinsics). Leave it true until both calibrations '
        'are done so FAST-LIO2 can self-correct.', styles))

    story.append(H('6.2  Lidar↔IMU Extrinsic Calibration', 'h2', styles))
    story.append(P(
        'Calibrates the rigid transform (translation + rotation) between the '
        'lidar sensor frame and the IMU sensor frame. Requires 2 minutes of '
        'figure-8 excitation motion.', styles))
    story += code_block('Run extrinsic calibration', """\
# Robot must be mobile. Drive a figure-8 (≥ 2 m diameter circles) for 2 min.
# The script records a bag then runs lidar_imu_calib solver automatically.

./scripts/calibrate_extrinsics.sh jetson --duration 120

# After completion:
# 1. Open calibration/lidar_imu_extrinsics_<timestamp>.txt
# 2. Copy extrinsic_T and extrinsic_R into fast_lio_hesai_qt64.yaml
# 3. Update joint xyz/rpy in urdf/robot.urdf.xacro
# 4. Set extrinsic_est_en: false in fast_lio_hesai_qt64.yaml
# 5. Rebuild: colcon build --packages-select localization robot_description""", styles)

    story.append(H('6.3  Prior Map Generation', 'h2', styles))
    story.append(P(
        'After sensor calibration, drive the competition course once to '
        'build a prior map using LIO-SAM. This map is used by Nav2 as the '
        'global static layer.', styles))
    story += code_block('Offline mapping session', """\
# Start the offline_mapping.launch.py (LIO-SAM with RViz)
source scripts/env.sh jetson
ros2 launch localization offline_mapping.launch.py use_rviz:=true

# Drive the entire competition course manually
# Watch RViz to confirm loop-closure events (green segments)

# When done, save the map:
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap \\
    "{resolution: 0.1, destination: '/tmp/competition_map'}"

# Copy map files to challenge_bringup/maps/:
cp /tmp/competition_map/GlobalMap.pcd src/challenge_bringup/maps/
# Convert to Nav2 occupancy grid:
ros2 run nav2_map_server map_saver_cli -f src/challenge_bringup/maps/static_map""", styles)
    story.append(PageBreak())

    # ════════════════════════════════════════════════════════════════════════
    # SECTION 7 — COMPETITION DAY OPERATIONS
    # ════════════════════════════════════════════════════════════════════════
    story.append(H('7. Competition Day Operations', 'h1', styles))
    story.append(hr(styles))

    story.append(H('7.1  Pre-flight checklist', 'h2', styles))
    story.append(P(
        'Run health_check.sh before every run. It validates:', styles))
    story += B([
        'TF tree completeness (map → odom → base_link → lidar_link)',
        'Critical topic rates (/hesai/points ≥ 10 Hz, /imu/data ≥ 150 Hz)',
        'E-stop state (/estop_active = false)',
        'Nav2 lifecycle manager nodes active',
        'Battery voltage within operating range (via /diagnostics)',
    ], styles)
    story += code_block('Pre-flight', """\
# On Jetson, run before every competition attempt:
./scripts/health_check.sh jetson

# Expected output: all checks PASS
# If any check FAILS, do NOT start the run until resolved""", styles)

    story.append(H('7.2  Competition launch sequence', 'h2', styles))
    story += code_block('Standard competition run', """\
# Terminal 1: Source environment
source ~/ros2_ws/src/DIY-Challenge-Repo/scripts/env.sh jetson

# Terminal 2: Start the full competition stack
./scripts/run_robot.sh jetson

# Wait for FAST-LIO2 to initialise (typically 5–10 s after lidar starts)
# Monitor /mux_mode — should show AUTONOMOUS when ready

# To switch to manual joystick at any time:
ros2 param set /cmd_vel_mux_node mode JOYSTICK

# To return to autonomous:
ros2 param set /cmd_vel_mux_node mode AUTONOMOUS""", styles)

    story.append(H('7.3  Recording during a run', 'h2', styles))
    story += code_block('Record competition bag (separate terminal)', """\
# Start recording BEFORE launching the stack (captures all startup messages)
./scripts/record_bag.sh jetson --label competition_run1

# Or add a custom label to describe the specific scenario:
./scripts/record_bag.sh jetson --label east_gate_approach""", styles)
    story.append(note_box(
        'Bags are written to DIY_BAG_OUTPUT_DIR (/mnt/usb/bags on Jetson). '
        'Ensure the USB SSD is mounted before recording. Bag files are '
        'split every 5 minutes (300 s) and compressed with zstd.', styles))

    story.append(H('7.4  Emergency procedures', 'h2', styles))
    story += B([
        '<b>Hardware E-stop button pressed:</b> Motors cut immediately by STM32. '
        'ROS sees /estop_active = True within one publish cycle. To resume: '
        'physically release the E-stop button, then verify /estop_active goes '
        'False, then issue <code>ros2 param set /cmd_vel_mux_node mode AUTONOMOUS</code>.',
        '<b>Nav2 stuck / loop:</b> Switch to JOYSTICK mode, manually drive clear, '
        'then switch back to AUTONOMOUS.',
        '<b>FAST-LIO2 diverged (map drifting):</b> Restart the localisation launch '
        'with <code>ros2 launch localization localization.launch.py</code>. '
        'FAST-LIO2 will reinitialise from the next lidar scan.',
        '<b>micro-ROS connection lost:</b> Reconnect USB, then '
        '<code>ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0</code>.',
    ], styles)
    story.append(PageBreak())

    # ════════════════════════════════════════════════════════════════════════
    # SECTION 8 — DEVELOPER WORKFLOWS (LAPTOP)
    # ════════════════════════════════════════════════════════════════════════
    story.append(H('8. Developer Workflows (Laptop)', 'h1', styles))
    story.append(hr(styles))

    story.append(H('8.1  Replaying a robot bag on laptop', 'h2', styles))
    story.append(P(
        'The replay workflow lets you run the full software stack against '
        'recorded sensor data on the laptop without any hardware. This is '
        'the primary debugging and algorithm-tuning workflow.', styles))
    story += code_block('Replay workflow', """\
# Copy a bag from the robot:
scp -r ubuntu@jetson.local:/mnt/usb/bags/jetson_run1_<timestamp> ~/bags/

# Replay at full speed with Nav2 and RViz:
./scripts/replay_bag.sh ~/bags/jetson_run1_<timestamp> laptop

# Replay at half speed for detailed analysis:
./scripts/replay_bag.sh ~/bags/jetson_run1_<timestamp> laptop --rate 0.5

# Replay without Nav2 (SLAM-only debugging):
./scripts/replay_bag.sh ~/bags/jetson_run1_<timestamp> laptop --no-nav2""", styles)

    story.append(H('8.2  Isolating a single node for debugging', 'h2', styles))
    story += code_block('Debug a single node', """\
# Example: debug cmd_vel_mux in isolation
./scripts/debug_robot.sh laptop --node diy_cmd_vel_mux/cmd_vel_mux_node

# While the node runs, inject test commands from another terminal:
ros2 topic pub /cmd_vel_nav geometry_msgs/Twist '{linear: {x: 0.3}}'
ros2 topic pub /estop_active std_msgs/Bool 'data: true'
ros2 topic echo /cmd_vel_safe
ros2 topic echo /mux_mode""", styles)

    story.append(H('8.3  Deploying updates to the robot', 'h2', styles))
    story += code_block('Deploy calibration bundle', """\
# After updating calibration YAMLs or maps on the laptop:
./scripts/deploy_bundle.sh jetson.local --user ubuntu

# Dry-run first to preview what will be transferred:
./scripts/deploy_bundle.sh jetson.local --dry-run

# After deploy, rebuild on the robot:
ssh ubuntu@jetson.local 'cd ~/ros2_ws && colcon build --symlink-install \\
    --packages-select localization robot_description challenge_bringup'""", styles)
    story.append(PageBreak())

    # ════════════════════════════════════════════════════════════════════════
    # SECTION 9 — SCRIPTS REFERENCE
    # ════════════════════════════════════════════════════════════════════════
    story.append(H('9. Scripts Reference', 'h1', styles))
    story.append(hr(styles))

    # Build paragraph styles for table cells — Paragraph objects wrap properly;
    # plain strings in Table cells get clipped when they exceed the cell width.
    _th = ParagraphStyle('ScriptTH', fontName='Helvetica-Bold',
                         fontSize=8, textColor=C_WHITE, leading=11)
    _tc = ParagraphStyle('ScriptName', fontName='Courier',
                         fontSize=8, textColor=C_BLACK, leading=11)
    _tu = ParagraphStyle('ScriptUsage', fontName='Courier',
                         fontSize=7.5, textColor=colors.HexColor('#1E3A5F'), leading=10)
    _tp = ParagraphStyle('ScriptPurpose', fontName='Helvetica',
                         fontSize=8, textColor=C_BLACK, leading=11)

    def _row(name, usage, purpose, header=False):
        s = _th if header else None
        return [
            Paragraph(name,    s or _tc),
            Paragraph(usage,   s or _tu),
            Paragraph(purpose, s or _tp),
        ]

    scripts_data = [
        _row('Script', 'Usage', 'Purpose', header=True),
        _row('env.sh',
             'source scripts/env.sh [profile]',
             'Bootstrap: sources all ROS 2 overlays and loads DIY_* profile '
             'variables. Must be sourced (not executed) before any other script.'),
        _row('run_robot.sh',
             './scripts/run_robot.sh [profile]',
             'Launches the full competition stack. Calls '
             'challenge_master.launch.py with all DIY_* flag overrides from '
             'the active profile.'),
        _row('debug_robot.sh',
             './scripts/debug_robot.sh [profile] [--node pkg/exe]',
             'Same as run_robot but enables RViz and sets log level to DEBUG. '
             'The --node flag runs a single node in isolation for quick testing.'),
        _row('record_bag.sh',
             './scripts/record_bag.sh [profile] [--label tag]',
             'Records all SAD-required topics to a timestamped rosbag. '
             'Automatically splits bags every DIY_BAG_SPLIT_DURATION seconds.'),
        _row('replay_bag.sh',
             './scripts/replay_bag.sh &lt;bag&gt; [profile] [--rate N]',
             'Replays a bag through the software stack at optional speed '
             'multiplier. Hardware driver nodes are skipped automatically.'),
        _row('calibrate_imu.sh',
             './scripts/calibrate_imu.sh [profile] [--duration N]',
             '2-3 hour static IMU recording followed by imu_utils Allan '
             'variance analysis. Copy the output noise params into '
             'fast_lio_hesai_qt64.yaml (gyr_cov, acc_cov, b_gyr_cov, b_acc_cov).'),
        _row('calibrate_extrinsics.sh',
             './scripts/calibrate_extrinsics.sh [profile]',
             'Figure-8 motion recording followed by lidar_imu_calib solver. '
             'Copy extrinsic_T and extrinsic_R results into the FAST-LIO2 '
             'config and the robot URDF joint offsets.'),
        _row('health_check.sh',
             './scripts/health_check.sh [profile]',
             'Pre-flight validation: checks ROS environment, required node '
             'liveness, topic rates, TF tree completeness, E-stop state, and '
             'Nav2 lifecycle state. Exits non-zero if any check fails.'),
        _row('deploy_bundle.sh',
             './scripts/deploy_bundle.sh &lt;host&gt; [--user u]',
             'rsyncs the calibration/, maps/, and config/ directories to the '
             'robot over SSH. Run after any calibration update or map rebuild.'),
    ]

    # colWidths: Script(3.8) + Usage(5.0) + Purpose(8.6) = 17.4 cm (full usable width)
    t = Table(scripts_data, colWidths=[3.8*cm, 5.0*cm, 8.6*cm],
              repeatRows=1)   # repeat header row if table spans pages
    t.setStyle(TableStyle([
        ('BACKGROUND',    (0,0), (-1,0), C_NAVY),
        ('ROWBACKGROUNDS',(0,1), (-1,-1), [C_WHITE, C_LGREY]),
        ('GRID',          (0,0), (-1,-1), 0.3, C_MGREY),
        ('TOPPADDING',    (0,0), (-1,-1), 6),
        ('BOTTOMPADDING', (0,0), (-1,-1), 6),
        ('LEFTPADDING',   (0,0), (-1,-1), 6),
        ('RIGHTPADDING',  (0,0), (-1,-1), 6),
        ('VALIGN',        (0,0), (-1,-1), 'TOP'),
    ]))
    story.append(t)
    story.append(PageBreak())

    # ════════════════════════════════════════════════════════════════════════
    # SECTION 10 — TROUBLESHOOTING
    # ════════════════════════════════════════════════════════════════════════
    story.append(H('10. Troubleshooting', 'h1', styles))
    story.append(hr(styles))

    issues = [
        (
            'FAST-LIO2 not publishing /lidar_odometry',
            [
                'Verify /hesai/points is being published: ros2 topic hz /hesai/points',
                'Check lidar_type in fast_lio_hesai_qt64.yaml — must be 2 for PointCloud2',
                'Confirm QT64 IP is reachable: ping 192.168.1.201',
                'Check FAST-LIO2 logs: ros2 node info /fastlio_mapping',
            ]
        ),
        (
            '/estop_active stays True after releasing the button',
            [
                'Check if micro-ROS heartbeat is arriving: ros2 topic hz /stm32/heartbeat',
                'If no heartbeat: verify micro-ROS agent is running and USB is connected',
                'Check /estop_reason topic to see which condition is firing',
                'If STM32 flag: check /stm32/estop_active topic value',
            ]
        ),
        (
            'Nav2 not producing goals / robot not moving',
            [
                'Verify /odometry/filtered is publishing at ~50 Hz',
                'Check TF tree: ros2 run tf2_tools view_frames',
                'Ensure map→odom transform exists (from FAST-LIO2 or EKF2)',
                'Check Nav2 lifecycle nodes: ros2 lifecycle get /planner_server',
                'Verify /mux_mode = AUTONOMOUS: ros2 topic echo /mux_mode',
            ]
        ),
        (
            'colcon build fails on localization / robot_description',
            [
                'Source environment before building: source scripts/env.sh jetson',
                'Check third_party_ws is built: ls third_party_ws/install/',
                'Run with verbose output: colcon build --event-handlers console_direct+',
                'Check for missing dependencies: rosdep check --from-paths src/',
            ]
        ),
        (
            'Joystick not working / cmd_vel not passing through',
            [
                'Verify joystick device: ls /dev/input/js*',
                'Check /mux_mode = JOYSTICK (not AUTONOMOUS or ESTOP_LOCK)',
                'Verify /cmd_vel_joy is publishing: ros2 topic hz /cmd_vel_joy',
                'Check joy_staleness_timeout_s parameter (default 2 s)',
            ]
        ),
        (
            'GPS not converging / EKF2 drifting',
            [
                'Check /gps/fix type — must be type 2 (DGPS/RTK) for accurate fusion',
                'Verify magnetic_declination_radians in navsat_transform.yaml for your location',
                'Allow 30+ seconds after startup for EKF2 heading initialisation',
                'Check /gps/filtered topic to see filtered GPS in map frame',
            ]
        ),
    ]

    for title, bullets in issues:
        story.append(H(title, 'h3', styles))
        story += B(bullets, styles)
        story.append(Spacer(1, 4))

    story.append(PageBreak())

    # ════════════════════════════════════════════════════════════════════════
    # SECTION 11 — EXTENDING THE CODEBASE
    # ════════════════════════════════════════════════════════════════════════
    story.append(H('11. Extending the Codebase', 'h1', styles))
    story.append(hr(styles))

    story.append(H('11.1  Adding a new sensor driver', 'h2', styles))
    story += B([
        'Add the driver node to challenge_master.launch.py with an IfCondition guard '
        'and a matching DIY_USE_<SENSOR> variable in each profiles/*.env file',
        'Add the sensor link to urdf/robot.urdf.xacro with a CALIB: joint',
        'Add the sensor topic to record_bag.sh REQUIRED_TOPICS list',
        'Add a topic-rate check in the HEALTH CHECKS section of health_check.sh',
    ], styles)

    story.append(H('11.2  Adding a new ROS 2 package', 'h2', styles))
    story += code_block('Create a new ament_python package', """\
cd ~/ros2_ws/src/DIY-Challenge-Repo/src
ros2 pkg create my_new_package \\
    --build-type ament_python \\
    --dependencies rclpy std_msgs geometry_msgs
# Add your node, update setup.py entry_points, colcon build""", styles)

    story.append(H('11.3  Swapping the SLAM algorithm', 'h2', styles))
    story.append(P(
        'To swap FAST-LIO2 for another algorithm (e.g. LOAM, LIO-SAM for '
        'runtime), update localization.launch.py to launch the new node and '
        'remap its odometry output to /lidar_odometry. EKF1 will pick it up '
        'automatically since it subscribes by topic name.', styles))

    story.append(H('11.4  Updating Nav2 parameters', 'h2', styles))
    story.append(P(
        'All Nav2 config lives in <b>src/challenge_bringup/config/nav2_params.yaml</b>. '
        'Key values to tune for a new course:', styles))
    story += B([
        'vx_max / wz_max — speed limits (respect course width and corner radii)',
        'inflation_radius — conservative by default at 0.45 m; reduce if course '
        'has narrow passages (minimum = robot_radius + 0.05 m)',
        'batch_size — increase MPPI batch to 2000 for smoother paths if Jetson CPU allows',
        'xy_goal_tolerance — 0.20 m default; increase to 0.35 m for loose waypoint following',
    ], styles)

    story.append(Spacer(1, 1*cm))
    story.append(hr(styles))
    story.append(H('End of Document', 'h2', styles))
    story.append(P(
        f'DIY-Challenge-Repo  |  ROS 2 Humble  |  Generated {date.today().isoformat()}  |  '
        'For questions, see docs/diy-sad.html or the repo README.', styles))

    return story


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────
def main():
    os.makedirs(os.path.dirname(OUT_PATH), exist_ok=True)
    styles = make_styles()

    doc = SimpleDocTemplate(
        OUT_PATH,
        pagesize=A4,
        leftMargin=1.8*cm,
        rightMargin=1.8*cm,
        topMargin=2.0*cm,
        bottomMargin=2.0*cm,
        title='DIY Challenge Robot — Development & Operations Guide',
        author='DIY Challenge Team',
        subject='ROS 2 Robot Documentation',
    )

    story = build_story(styles)

    # Use title page template for page 1, normal template for rest
    doc.build(
        story,
        onFirstPage=make_title_page,
        onLaterPages=make_page_template,
    )
    print(f'PDF generated: {OUT_PATH}')


if __name__ == '__main__':
    main()
