#!/usr/bin/env python3
"""
generate_camera_guide.py — RealSense D435i Camera Calibration Guide
Produces docs/Camera_Calibration_Guide.pdf using ReportLab.
Run from the repo root:   python3 docs/generate_camera_guide.py
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
    PageBreak, HRFlowable, Preformatted, Image as RLImage,
)

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

# ── Output path ───────────────────────────────────────────────────────────────
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_PATH  = os.path.join(REPO_ROOT, 'docs', 'Camera_Calibration_Guide.pdf')

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
C_RS     = colors.HexColor('#0071C5')   # Intel RealSense blue


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
    s['h2'] = ParagraphStyle('H2', fontSize=13, leading=17,
        textColor=C_BLUE,
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
# Diagram: D435i sensor layout
# ─────────────────────────────────────────────────────────────────────────────
def _make_d435i_diagram():
    tmp = _tmpmod.NamedTemporaryFile(suffix='.png', delete=False)
    fig, ax = plt.subplots(figsize=(13, 4))
    fig.patch.set_facecolor('white')
    ax.set_facecolor('white')
    ax.set_xlim(0, 14)
    ax.set_ylim(0, 4)
    ax.axis('off')

    # Camera body
    ax.add_patch(FancyBboxPatch((1.0, 1.2), 12.0, 1.6,
        boxstyle='round,pad=0.1', facecolor='#1E293B', edgecolor='#475569', lw=2))

    # Sensors from left to right: IR left, RGB, RGB-D projector, IR right, IMU
    sensors = [
        (2.2,  '#60A5FA', 'IR Left\n(infra1)\n848×480 @ 30fps'),
        (4.5,  '#F59E0B', 'RGB Color\n1920×1080\n@ 30fps'),
        (7.0,  '#A855F7', 'Depth\nProjector\n(structured light)'),
        (9.3,  '#60A5FA', 'IR Right\n(infra2)\n848×480 @ 30fps'),
        (11.5, '#34D399', 'BMI055\nIMU\n6-DOF @ 200Hz'),
    ]
    for x, c, label in sensors:
        ax.add_patch(plt.Circle((x, 2.0), 0.45, color=c, zorder=3))
        ax.text(x, 0.65, label, ha='center', va='top', fontsize=7.5,
                color='#1E293B', fontweight='bold')
        ax.plot([x, x], [1.55, 0.95], color='#94A3B8', lw=1, zorder=2)

    # Baseline annotation
    ax.annotate('', xy=(9.3, 3.4), xytext=(2.2, 3.4),
        arrowprops=dict(arrowstyle='<->', color='#F59E0B', lw=2))
    ax.text(5.75, 3.55, 'Stereo Baseline = 50 mm  (factory calibrated)',
        ha='center', fontsize=9, color='#92400E', fontweight='bold')

    ax.text(7, 4.0, 'Intel RealSense D435i — Sensor Layout',
        ha='center', fontsize=12, fontweight='bold', color='#1A2744')

    plt.tight_layout(pad=0.3)
    plt.savefig(tmp.name, dpi=140, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    return tmp.name


# ─────────────────────────────────────────────────────────────────────────────
# Diagram: calibration pipeline overview
# ─────────────────────────────────────────────────────────────────────────────
def _make_pipeline_diagram():
    tmp = _tmpmod.NamedTemporaryFile(suffix='.png', delete=False)
    fig, ax = plt.subplots(figsize=(13, 3.5))
    fig.patch.set_facecolor('white')
    ax.set_facecolor('white')
    ax.set_xlim(0, 14)
    ax.set_ylim(0, 3.5)
    ax.axis('off')

    def box(cx, cy, w, h, label, color, fs=9):
        ax.add_patch(FancyBboxPatch((cx - w/2, cy - h/2), w, h,
            boxstyle='round,pad=0.12', facecolor=color, edgecolor='white', lw=1.5, zorder=3))
        ax.text(cx, cy, label, ha='center', va='center', fontsize=fs,
                color='white', fontweight='bold', zorder=4, wrap=True)

    def arrow(x1, x2, y):
        ax.annotate('', xy=(x2 - 0.1, y), xytext=(x1 + 0.1, y),
            arrowprops=dict(arrowstyle='->', color='#64748B', lw=1.8))

    steps = [
        (1.5,  '#0F766E', 'Step 0\nVerify factory\nintrinsics'),
        (4.3,  '#2563EB', 'Step 1\nIntrinsic calib\n(if needed)'),
        (7.1,  '#7C3AED', 'Step 2\nCam↔Lidar\nextrinsic calib'),
        (9.9,  '#EA580C', 'Step 3\nUpdate URDF\n& configs'),
        (12.7, '#16A34A', 'Step 4\nVerify &\nvalidate'),
    ]
    for cx, c, label in steps:
        box(cx, 1.75, 2.2, 1.5, label, c)

    for i in range(len(steps) - 1):
        arrow(steps[i][0] + 1.1, steps[i+1][0] - 1.1, 1.75)

    ax.text(7, 3.2, 'Camera Calibration Pipeline — RealSense D435i on DIY Challenge Robot',
        ha='center', fontsize=11, fontweight='bold', color='#1A2744')

    plt.tight_layout(pad=0.3)
    plt.savefig(tmp.name, dpi=140, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    return tmp.name


# ─────────────────────────────────────────────────────────────────────────────
# Diagram: intrinsic parameter visualisation
# ─────────────────────────────────────────────────────────────────────────────
def _make_intrinsics_diagram():
    tmp = _tmpmod.NamedTemporaryFile(suffix='.png', delete=False)
    fig, ax = plt.subplots(figsize=(11, 5))
    fig.patch.set_facecolor('white')
    ax.set_facecolor('white')
    ax.set_xlim(-1, 12)
    ax.set_ylim(-0.5, 5.5)
    ax.axis('off')

    # Image plane rectangle
    ax.add_patch(FancyBboxPatch((5.5, 0.5), 5.0, 4.0,
        boxstyle='round,pad=0.05', facecolor='#F8FAFC', edgecolor='#334155', lw=2))

    # Camera origin
    ax.plot(0, 2.5, 'o', color='#1A2744', ms=10, zorder=5)
    ax.text(0, 2.0, 'Camera\ncentre\n(origin)', ha='center', fontsize=8.5, color='#1A2744')

    # optical axis
    ax.annotate('', xy=(5.5, 2.5), xytext=(0.2, 2.5),
        arrowprops=dict(arrowstyle='->', color='#64748B', lw=1.5))
    ax.text(2.7, 2.7, 'Optical axis (z)', ha='center', fontsize=8, color='#64748B')

    # principal point
    ax.plot(8.5, 2.5, '+', color=C_BLUE.hexval() if hasattr(C_BLUE,'hexval') else '#2563EB',
            ms=16, mew=2.5, zorder=5)
    ax.text(8.5, 1.9, 'Principal point\n(cx, cy)', ha='center', fontsize=8.5,
            color='#2563EB', fontweight='bold')

    # cx annotation
    ax.annotate('', xy=(8.5, 0.5), xytext=(5.5, 0.5),
        arrowprops=dict(arrowstyle='<->', color='#2563EB', lw=1.5))
    ax.text(7.0, 0.2, 'cx  (horiz offset from left)', ha='center', fontsize=8, color='#2563EB')

    # cy annotation
    ax.annotate('', xy=(10.5, 2.5), xytext=(10.5, 0.5),
        arrowprops=dict(arrowstyle='<->', color='#7C3AED', lw=1.5))
    ax.text(11.3, 1.5, 'cy', ha='center', fontsize=9, color='#7C3AED', fontweight='bold')

    # focal length
    ax.annotate('', xy=(5.5, 3.5), xytext=(0.2, 3.5),
        arrowprops=dict(arrowstyle='<->', color='#EA580C', lw=1.5))
    ax.text(2.7, 3.8, 'f  (focal length in pixels)', ha='center', fontsize=8.5, color='#EA580C')

    ax.text(5.5, 5.1, 'Camera Intrinsic Parameters', ha='left', fontsize=11,
            fontweight='bold', color='#1A2744')
    ax.text(5.5, 4.75, 'K = [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]',
            ha='left', fontsize=9.5, color='#1A2744',
            fontfamily='monospace')

    plt.tight_layout(pad=0.3)
    plt.savefig(tmp.name, dpi=140, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    return tmp.name


# ─────────────────────────────────────────────────────────────────────────────
# Cover page
# ─────────────────────────────────────────────────────────────────────────────
def cover_page(styles):
    story = []
    story.append(Spacer(1, 3.5*cm))
    story.append(Paragraph('Camera Calibration Guide', styles['title']))
    story.append(Spacer(1, 0.4*cm))
    story.append(Paragraph(
        'Intel RealSense D435i — Intrinsic, Extrinsic & Camera-to-LiDAR Calibration',
        styles['subtitle']))
    story.append(Spacer(1, 0.2*cm))
    story.append(Paragraph(
        'DIY Challenge Repo  ·  ROS 2 Humble  ·  Hesai QT64  ·  Jetson Nano',
        styles['subtitle']))
    story.append(Spacer(1, 2.5*cm))

    tbl_data = [
        [Paragraph('Camera', styles['th']),
         Paragraph('Intel RealSense D435i (stereo IR + RGB + IMU)', styles['tc'])],
        [Paragraph('LiDAR', styles['th']),
         Paragraph('Hesai QT64 3D lidar', styles['tc'])],
        [Paragraph('Compute', styles['th']),
         Paragraph('NVIDIA Jetson Nano 4 GB / JetPack 5.x', styles['tc'])],
        [Paragraph('ROS 2 Version', styles['th']),
         Paragraph('Humble', styles['tc'])],
        [Paragraph('Intrinsic source', styles['th']),
         Paragraph('Factory EEPROM (auto-loaded by RealSense driver) — manual recal optional', styles['tc'])],
        [Paragraph('Extrinsic (cam↔lidar)', styles['th']),
         Paragraph('Must be measured/calibrated — placeholder values in URDF must be replaced', styles['tc'])],
    ]
    tbl = Table(tbl_data, colWidths=[5.5*cm, 11.9*cm])
    tbl.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (0,-1), C_NAVY),
        ('BACKGROUND', (1,0), (1,-1), C_LGREY),
        ('ROWBACKGROUNDS', (1,0), (1,-1), [C_LGREY, colors.white]),
        ('GRID', (0,0), (-1,-1), 0.4, C_MGREY),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('TOPPADDING', (0,0), (-1,-1), 6),
        ('BOTTOMPADDING', (0,0), (-1,-1), 6),
        ('LEFTPADDING', (0,0), (-1,-1), 8),
    ]))
    story.append(tbl)
    story.append(PageBreak())
    return story


# ─────────────────────────────────────────────────────────────────────────────
# Document body
# ─────────────────────────────────────────────────────────────────────────────
def build_story(styles):
    story = []
    story += cover_page(styles)

    # ── Overview ──────────────────────────────────────────────────────────────
    story.append(H('0. Overview — What Calibration Is Needed?', 'h1', styles))
    story.append(hr(styles))
    story.append(P(
        'The RealSense D435i has two distinct calibration requirements that serve '
        'completely different purposes. Understanding which ones you need — and which '
        'are already handled — is essential before touching any config files.', styles))
    story.append(SP())

    # Pipeline diagram
    img_pipeline = _make_pipeline_diagram()
    story.append(RLImage(img_pipeline, width=17.4*cm, height=4.7*cm))
    story.append(SP(8))

    tbl_data = [
        [Paragraph('Calibration Type', styles['th']),
         Paragraph('What it provides', styles['th']),
         Paragraph('Already done?', styles['th']),
         Paragraph('Action required', styles['th'])],
        [Paragraph('Intrinsic\n(per camera)', styles['tc']),
         Paragraph('fx, fy, cx, cy, distortion coefficients', styles['tc']),
         Paragraph('YES — factory calibrated,\nstored in EEPROM', styles['tc']),
         Paragraph('Verify. Recalibrate only if camera takes a hard knock '
                   'or intrinsics look wrong.', styles['tc'])],
        [Paragraph('Stereo baseline\n(IR left ↔ IR right)', styles['tc']),
         Paragraph('50 mm baseline transform between the two IR cameras', styles['tc']),
         Paragraph('YES — factory calibrated', styles['tc']),
         Paragraph('None. Never attempt to re-do stereo extrinsics manually.', styles['tc'])],
        [Paragraph('Camera ↔ LiDAR\nextrinsic', styles['tc']),
         Paragraph('6-DOF transform from camera_link to lidar_link', styles['tc']),
         Paragraph('NO — URDF has placeholder\nvalues (CALIB: markers)', styles['tc']),
         Paragraph('REQUIRED. Measure physically + optionally refine with '
                   'target-based calibration. This is the critical step.', styles['tc'])],
        [Paragraph('Camera ↔ Robot\n(URDF TF)', styles['tc']),
         Paragraph('base_link → camera_link static transform', styles['tc']),
         Paragraph('Partial — placeholder\nxyz="0.20 0.0 0.20"', styles['tc']),
         Paragraph('Update URDF with physically measured or '
                   'cam-lidar-calibrated values.', styles['tc'])],
        [Paragraph('Time\nsynchronisation', styles['tc']),
         Paragraph('Align camera and lidar timestamps', styles['tc']),
         Paragraph('Partial — RealSense driver\nhandles internal sync', styles['tc']),
         Paragraph('Verify clock offset between camera and lidar host '
                   '(chronyd or PTP recommended).', styles['tc'])],
    ]
    tbl = Table(tbl_data, colWidths=[3.4*cm, 4.8*cm, 4.0*cm, 5.2*cm])
    tbl.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), C_NAVY),
        ('ROWBACKGROUNDS', (0,1), (-1,-1), [C_LGREY, colors.white]),
        ('BACKGROUND', (2,3), (2,3), colors.HexColor('#FEE2E2')),
        ('BACKGROUND', (2,4), (2,4), colors.HexColor('#FEF3C7')),
        ('GRID', (0,0), (-1,-1), 0.4, C_MGREY),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('TOPPADDING', (0,0), (-1,-1), 5),
        ('BOTTOMPADDING', (0,0), (-1,-1), 5),
        ('LEFTPADDING', (0,0), (-1,-1), 5),
    ]))
    story += [tbl, SP()]
    story.append(warn_box(
        'The URDF file (src/robot_description/urdf/robot.urdf.xacro) currently has '
        'PLACEHOLDER values for camera_link (xyz="0.20 0.0 0.20"). '
        'These must be replaced with real measured values before the robot is used. '
        'Using wrong extrinsics will cause misaligned sensor fusion and poor localization.', styles))
    story.append(PageBreak())

    # ── Section 1: Understanding Intrinsics ───────────────────────────────────
    story.append(H('1. Understanding Camera Intrinsic Parameters', 'h1', styles))
    story.append(hr(styles))

    img_d435i = _make_d435i_diagram()
    story.append(RLImage(img_d435i, width=17.4*cm, height=5.2*cm))
    story.append(SP(8))

    story.append(H('1.1  What are intrinsic parameters?', 'h2', styles))
    story.append(P(
        'Intrinsic parameters describe the internal optics of a single camera — '
        'they are independent of where the camera is mounted. '
        'They are captured in the 3×3 camera matrix K:', styles))
    story += code_block('Camera intrinsic matrix', """\
K = | fx   0   cx |
    |  0  fy   cy |
    |  0   0    1 |

fx, fy  — focal lengths in pixels (how much the lens magnifies the scene)
cx, cy  — principal point (pixel coordinates of the optical axis intersection)
           Ideally: cx ≈ image_width/2,  cy ≈ image_height/2

Distortion coefficients (Brown-Conrady model):
  [k1, k2, p1, p2, k3]
  k1, k2, k3 — radial distortion  (barrel / pincushion)
  p1, p2      — tangential distortion (lens not perfectly parallel to sensor)""", styles)

    story.append(H('1.2  D435i factory intrinsics — what you already have', 'h2', styles))
    story.append(P(
        'The RealSense D435i stores its calibration data in on-board EEPROM, factory-calibrated '
        'by Intel at sub-pixel precision. The <code>realsense2_camera</code> ROS 2 driver reads '
        'this data at startup and publishes it automatically on the camera_info topics:', styles))
    story += B([
        '<code>/camera/color/camera_info</code> — RGB colour camera intrinsics',
        '<code>/camera/infra1/camera_info</code> — Left IR camera intrinsics',
        '<code>/camera/infra2/camera_info</code> — Right IR camera intrinsics',
        '<code>/camera/depth/camera_info</code> — Depth stream intrinsics',
    ], styles)
    story.append(tip_box(
        'For this competition stack you do NOT need to do intrinsic calibration. '
        'The factory calibration is sufficient for YOLO detection, depth sensing, '
        'and stereo VSLAM. Proceed directly to Section 3 (camera-to-lidar extrinsic).', styles))

    story.append(H('1.3  How to inspect the factory intrinsics', 'h2', styles))
    story.append(P(
        'With the camera plugged in and the RealSense driver running, inspect the '
        'published calibration:', styles))
    story += code_block('Inspect camera intrinsics from ROS 2 topic', """\
# Start the RealSense driver (or the full bringup)
ros2 launch realsense2_camera rs_launch.py

# In another terminal - print the colour camera intrinsics
ros2 topic echo /camera/color/camera_info --once

# Expected output (approximate for D435i @ 1280x720):
# width: 1280
# height: 720
# K: [911.3, 0, 640.5, 0, 911.3, 360.3, 0, 0, 1]
#        fx       cx        fy       cy
# D: [-0.054, 0.068, 0.001, -0.001, -0.022]   (distortion k1..k3, p1, p2)
# distortion_model: plumb_bob""", styles)

    story.append(H('1.4  Typical D435i factory values', 'h2', styles))
    tbl_data = [
        [Paragraph('Stream', styles['th']),
         Paragraph('Resolution', styles['th']),
         Paragraph('fx ≈ fy', styles['th']),
         Paragraph('cx ≈ w/2', styles['th']),
         Paragraph('cy ≈ h/2', styles['th'])],
        [Paragraph('IR (infra1/2)', styles['tc']),
         Paragraph('848 × 480', styles['tc']),
         Paragraph('~430 px', styles['tc']),
         Paragraph('~424 px', styles['tc']),
         Paragraph('~240 px', styles['tc'])],
        [Paragraph('Color (RGB)', styles['tc']),
         Paragraph('1280 × 720', styles['tc']),
         Paragraph('~911 px', styles['tc']),
         Paragraph('~641 px', styles['tc']),
         Paragraph('~360 px', styles['tc'])],
        [Paragraph('Color (RGB)', styles['tc']),
         Paragraph('640 × 480', styles['tc']),
         Paragraph('~606 px', styles['tc']),
         Paragraph('~321 px', styles['tc']),
         Paragraph('~240 px', styles['tc'])],
    ]
    tbl = Table(tbl_data, colWidths=[4.0*cm, 3.8*cm, 3.0*cm, 3.0*cm, 3.0*cm])
    tbl.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), C_NAVY),
        ('ROWBACKGROUNDS', (0,1), (-1,-1), [C_LGREY, colors.white]),
        ('GRID', (0,0), (-1,-1), 0.4, C_MGREY),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('TOPPADDING', (0,0), (-1,-1), 5),
        ('BOTTOMPADDING', (0,0), (-1,-1), 5),
        ('LEFTPADDING', (0,0), (-1,-1), 5),
        ('ALIGN', (2,0), (-1,-1), 'CENTER'),
    ]))
    story += [tbl, SP()]
    story.append(note_box(
        'If the published fx/fy values differ significantly from the table above '
        '(e.g. fx < 300 or > 1100 for RGB), or if cx/cy are far from image centre, '
        'the EEPROM data may be corrupted. In that case proceed to Section 2 for '
        'manual recalibration.', styles))
    story.append(PageBreak())

    # ── Section 2: Manual Intrinsic Calibration ───────────────────────────────
    story.append(H('2. Manual Intrinsic Calibration (Only If Needed)', 'h1', styles))
    story.append(hr(styles))
    story.append(warn_box(
        'Only perform this section if Section 1.3 shows corrupted or clearly wrong '
        'factory intrinsics. The factory calibration is typically more accurate than '
        'a manual checkerboard calibration.', styles))
    story.append(SP())

    story.append(H('2.1  What you need', 'h2', styles))
    story += B([
        '<b>Checkerboard pattern</b>: 9×6 inner corners, 25 mm square size recommended '
        '(print on A3 paper, mount flat on rigid board — no curling)',
        '<b>ROS 2 camera_calibration package</b>: <code>sudo apt install '
        'ros-humble-camera-calibration</code>',
        '<b>Good lighting</b>: bright, diffuse — no shadows on the checkerboard',
        '<b>~50–70 captures</b>: varying distance (0.5–2 m), tilt, and rotation',
    ], styles)

    story.append(H('2.2  Run the calibration', 'h2', styles))
    story += code_block('Install and run camera_calibration for the colour camera', """\
# Install if not present
sudo apt install ros-humble-camera-calibration

# Start the RealSense driver in another terminal
ros2 launch realsense2_camera rs_launch.py  color_width:=640 color_height:=480

# Run the calibrator (9x6 inner corners, 0.025 m square size)
ros2 run camera_calibration cameracalibrator \\
  --size 9x6 \\
  --square 0.025 \\
  --ros-args \\
  --remap image:=/camera/color/image_raw \\
  --remap camera:=/camera/color

# A GUI window opens. Move the checkerboard until all four bars fill:
#   X — horizontal coverage (slide board left and right across full frame)
#   Y — vertical coverage   (slide board up and down across full frame)
#   Size — distance variation (move board closer and farther, 0.5–2 m)
#   Skew — tilt variation   (tilt board ±30° in all axes)
# Once fully green, click CALIBRATE, then SAVE.
# Output saved to /tmp/calibrationdata.tar.gz""", styles)

    story += code_block('For IR cameras (used in stereo VSLAM)', """\
# Left IR camera only (use this if recalibrating for VSLAM)
ros2 run camera_calibration cameracalibrator \\
  --size 9x6 \\
  --square 0.025 \\
  --ros-args \\
  --remap image:=/camera/infra1/image_rect_raw \\
  --remap camera:=/camera/infra1

# NOTE: Disable the IR emitter first to get a clean passive image:
# ros2 param set /realsense2_camera enable_infra_emitter false""", styles)

    story.append(H('2.3  Apply the new calibration', 'h2', styles))
    story += code_block('Extract and apply calibration YAML', """\
# Extract the calibration archive
cd /tmp && tar -xzf calibrationdata.tar.gz
# This creates: ost.yaml  (the new intrinsics)

# The RealSense driver can load a custom calibration file:
ros2 launch realsense2_camera rs_launch.py \\
  color_camera_info_url:=file:///tmp/ost.yaml

# To make it permanent, copy to the config directory:
cp /tmp/ost.yaml ~/ros2_ws/src/DIY-Challenge-Repo/src/challenge_bringup/config/color_camera_info.yaml
# Then update challenge_master.launch.py to pass color_camera_info_url""", styles)

    story.append(note_box(
        'The RealSense driver\'s camera_info_url parameter overrides the EEPROM calibration '
        'for the specified stream. Other streams continue to use factory calibration. '
        'Calibration scripts are provided in scripts/calibrate_camera_intrinsics.sh.', styles))
    story.append(PageBreak())

    # ── Section 3: Camera ↔ LiDAR Extrinsic ──────────────────────────────────
    story.append(H('3. Camera ↔ LiDAR Extrinsic Calibration (Required)', 'h1', styles))
    story.append(hr(styles))
    story.append(P(
        'This is the <b>most important calibration step</b> for this robot. '
        'The extrinsic calibration defines the 6-DOF rigid-body transform between '
        '<code>camera_link</code> and <code>lidar_link</code>. '
        'Without it, any node that fuses camera and lidar data (or uses the camera '
        'relative to the map) will have incorrect spatial reasoning.', styles))
    story.append(SP())
    story.append(warn_box(
        'The current URDF has placeholder values: base_to_camera xyz="0.20 0.0 0.20". '
        'These are a rough estimate only. They must be replaced with real measured values '
        'before competition.', styles))
    story.append(SP())

    story.append(H('3.1  What the extrinsic transform is', 'h2', styles))
    story += code_block('Extrinsic transform — what it means', """\
# The transform T_lidar_camera answers:
#   "Given a 3D point in camera_link frame, where is it in lidar_link frame?"
#
# It is a 4x4 homogeneous matrix:
#   T = | R  t |    R: 3x3 rotation matrix
#       | 0  1 |    t: 3x1 translation vector (x, y, z in metres)
#
# This is what goes into robot.urdf.xacro:
#   <origin xyz="tx ty tz" rpy="roll pitch yaw"/>
# under the base_to_camera joint.
#
# It is also written to the nav2_params.yaml obstacle source transform
# if camera detections are fused into the costmap.""", styles)

    story.append(H('3.2  Method A — Physical measurement (start here)', 'h2', styles))
    story.append(P(
        'For competition purposes, a careful physical measurement is sufficient as a '
        'starting point. Use a ruler or tape measure to determine the 3D offset of the '
        'camera lens centre from the lidar origin.', styles))
    story += code_block('How to physically measure camera-lidar offset', """\
# Reference frames:
#   lidar_link origin  = centre of Hesai QT64 rotating head
#   camera_link origin = centre of the D435i lens array

# Measure in ROBOT body frame (base_link convention: x=forward, y=left, z=up):

# 1. x offset: horizontal distance FORWARD from lidar to camera
#    (positive if camera is in front of lidar)
x = <measure in metres, e.g. 0.12>

# 2. y offset: lateral distance (positive = camera to LEFT of lidar)
#    (0.0 if both are on the robot centreline)
y = <measure in metres, e.g. 0.0>

# 3. z offset: vertical distance UP from lidar to camera
#    (positive if camera is above lidar)
z = <measure in metres, e.g. -0.05>

# 4. rpy: rotation of camera frame relative to lidar frame
#    For a camera mounted level and facing forward, same as lidar:
#    rpy = "0 0 0"  (no rotation)
#    If camera is tilted down 10 degrees to see ground objects:
#    rpy = "0 0.175 0"  (0, 10°, 0 in radians)

# Update robot.urdf.xacro:
# <origin xyz="0.12 0.0 -0.05" rpy="0 0 0"/>  ← under base_to_camera joint""", styles)

    story.append(H('3.3  Method B — Target-based calibration with Kalibr (precision)', 'h2', styles))
    story.append(P(
        'Kalibr is the industry-standard open-source tool for multi-sensor calibration. '
        'It uses an April-Tag target board to jointly solve camera intrinsics, '
        'camera-IMU extrinsics, and multi-camera geometry. '
        'For camera-to-lidar it is used alongside a known flat calibration target '
        'visible to both sensors.', styles))
    story += B([
        'Accuracy: ~1–3 mm translation, ~0.1° rotation (vs ~5–10 mm for manual measure)',
        'Time: ~2 hours for full setup, data collection, and solving',
        'Recommended if VSLAM or precise sensor fusion is required for the competition',
        'Not required if the robot only uses the camera for YOLO detection (coarse alignment sufficient)',
    ], styles)
    story.append(SP())
    story += code_block('Install Kalibr (Ubuntu 22.04 / ROS 2 Humble)', """\
# Kalibr runs best in a Docker container on Ubuntu 22.04
docker pull ethzasl/kalibr:latest

# Or build from source:
cd ~/  &&  git clone https://github.com/ethz-asl/kalibr.git
cd kalibr
# Follow https://github.com/ethz-asl/kalibr/wiki/installation for ROS 2""", styles)

    story += code_block('Camera-to-lidar calibration data collection', """\
# You need a calibration target visible in BOTH the camera AND the lidar.
# Best approach: a flat board with AprilTags + lidar-reflective markers.
#
# Step 1: Record a ROS 2 bag with both sensors:
ros2 bag record /camera/color/image_raw /camera/color/camera_info \\
                /hesai/points \\
                -o ~/calib_cam_lidar_$(date +%Y%m%d_%H%M%S)
#
# Step 2: Move the target board to 15-20 different positions/orientations
#         covering the overlapping FOV of camera and lidar.
#         Keep the robot stationary during each capture.
#
# Step 3: Run the Kalibr camera-lidar solver:
#   See: https://github.com/ethz-asl/kalibr/wiki/camera-imu-calibration
#   (adapt for lidar as the second sensor)""", styles)

    story.append(H('3.4  Method C — scripts/calibrate_cam_lidar.sh helper', 'h2', styles))
    story.append(P(
        'The repo provides a helper script that automates the data collection step. '
        'Run it on the robot, then process the bag offline:', styles))
    story += code_block('Use the calibration helper script', """\
# Ensure both camera and lidar are running, then:
./scripts/calibrate_cam_lidar.sh

# The script:
#   1. Verifies /camera/color/image_raw and /hesai/points are publishing
#   2. Records a ~3-minute bag to calibration/cam_lidar_<timestamp>/
#   3. Prints instructions for processing with Kalibr offline
#
# After processing, update the URDF with the result (Section 4).""", styles)

    story.append(PageBreak())

    # ── Section 4: Updating URDF and configs ─────────────────────────────────
    story.append(H('4. Applying Calibration Results to the Repository', 'h1', styles))
    story.append(hr(styles))

    story.append(H('4.1  Update the URDF camera joint', 'h2', styles))
    story.append(P(
        'Open <code>src/robot_description/urdf/robot.urdf.xacro</code> and '
        'replace the placeholder camera joint origin with your measured values:', styles))
    story += code_block('robot.urdf.xacro — replace placeholder (line ~118)', """\
<!-- BEFORE (placeholder): -->
<joint name="base_to_camera" type="fixed">
  <parent link="base_link"/>
  <child link="camera_link"/>
  <origin xyz="0.20 0.0 0.20" rpy="0.0 0.0 0.0"/>   <!-- ← PLACEHOLDER -->
</joint>

<!-- AFTER (example with measured values): -->
<joint name="base_to_camera" type="fixed">
  <parent link="base_link"/>
  <child link="camera_link"/>
  <origin xyz="0.18 0.0 0.22" rpy="0.0 0.05 0.0"/>  <!-- ← YOUR VALUES -->
</joint>
#
# xyz = translation in metres (x forward, y left, z up)
# rpy = roll pitch yaw in radians
#       e.g. rpy="0 0.05 0" tilts camera slightly downward (pitch 3°)""", styles)

    story.append(H('4.2  Rebuild the robot description package', 'h2', styles))
    story += code_block('Rebuild and verify TF tree', """\
# Rebuild after URDF change
cd ~/ros2_ws && colcon build --packages-select robot_description

# Verify the TF tree is correct
ros2 launch robot_description description.launch.py &
ros2 run tf2_tools view_frames  # saves frames.pdf
# Check that base_link → camera_link transform matches your measurements""", styles)

    story.append(H('4.3  Verify with RViz2', 'h2', styles))
    story += code_block('Visual verification in RViz2', """\
# Launch the full robot stack
ros2 launch challenge_bringup challenge_master.launch.py

# Open RViz2, add:
#   - PointCloud2 display: topic /hesai/points, fixed frame: base_link
#   - Image display: topic /camera/color/image_raw
#   - TF display (shows all coordinate frames)
#
# Place a known object (e.g. a 30cm box) at a measured distance in FRONT of the robot.
# Confirm:
#   - The lidar point cloud shows the box at the correct distance
#   - The camera image shows the box
#   - The TF arrow from base_link to camera_link points to the correct location on the robot""", styles)

    story.append(H('4.4  Verify sensor alignment numerically', 'h2', styles))
    story += code_block('Check timestamp synchronisation and topic rates', """\
# Check that camera and lidar are publishing at expected rates
ros2 topic hz /camera/color/image_raw   # expect ~30 Hz
ros2 topic hz /hesai/points             # expect ~10-20 Hz

# Check timestamp alignment (both should show recent, matching stamps)
ros2 topic echo /camera/color/image_raw --field header.stamp --once
ros2 topic echo /hesai/points --field header.stamp --once

# Large timestamp offset (>100 ms) indicates clock sync issue.
# Fix: ensure both camera and Jetson are NTP/PTP synchronised:
sudo chronyc tracking   # check NTP sync status""", styles)

    story.append(PageBreak())

    # ── Section 5: Time Synchronisation ──────────────────────────────────────
    story.append(H('5. Time Synchronisation Between Camera and LiDAR', 'h1', styles))
    story.append(hr(styles))
    story.append(P(
        'The RealSense D435i uses its own internal clock. The Hesai QT64 lidar has its own '
        'clock. If the Jetson Nano is not time-synchronised, timestamps can diverge by '
        'hundreds of milliseconds, corrupting any time-based sensor fusion.', styles))
    story.append(SP())

    story.append(H('5.1  Check current clock status', 'h2', styles))
    story += code_block('Verify system clock is synchronised', """\
# Check NTP synchronisation status
timedatectl status
# Look for: "System clock synchronized: yes"
# If not synchronized, install chrony:
sudo apt install chrony
sudo systemctl enable --now chrony
sudo chronyc makestep  # force immediate sync""", styles)

    story.append(H('5.2  RealSense hardware timestamp', 'h2', styles))
    story += code_block('Configure RealSense driver to use hardware timestamps', """\
# In challenge_master.launch.py, the realsense2_camera node already runs.
# To use hardware timestamps (lower jitter):
ros2 launch realsense2_camera rs_launch.py \\
  enable_sync:=true \\
  unite_imu_method:=linear_interpolation

# The enable_sync parameter synchronises all RealSense streams to the same
# hardware timestamp, reducing inter-stream offset to <1 ms.""", styles)

    story.append(H('5.3  Lidar-camera timestamp offset check', 'h2', styles))
    story += code_block('Measure actual timestamp offset', """\
# Record one message from each sensor and compare stamps
python3 - <<'EOF'
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
import time

class StampCheck(Node):
    def __init__(self):
        super().__init__('stamp_check')
        self.lidar_stamp = None
        self.camera_stamp = None
        self.create_subscription(PointCloud2, '/hesai/points',
            lambda m: setattr(self, 'lidar_stamp', m.header.stamp), 1)
        self.create_subscription(Image, '/camera/color/image_raw',
            lambda m: setattr(self, 'camera_stamp', m.header.stamp), 1)

rclpy.init()
node = StampCheck()
rclpy.spin_once(node, timeout_sec=2)
rclpy.spin_once(node, timeout_sec=2)
if node.lidar_stamp and node.camera_stamp:
    diff = abs((node.lidar_stamp.sec + node.lidar_stamp.nanosec*1e-9) -
               (node.camera_stamp.sec + node.camera_stamp.nanosec*1e-9))
    print(f"Timestamp offset: {diff*1000:.1f} ms")
    print("OK" if diff < 0.05 else "WARNING: offset > 50ms, check NTP sync")
EOF""", styles)

    story.append(PageBreak())

    # ── Section 6: Full Checklist ─────────────────────────────────────────────
    story.append(H('6. Camera Setup Checklist', 'h1', styles))
    story.append(hr(styles))
    story.append(P(
        'Complete all items in order before the competition. '
        'Items marked REQUIRED must be done. Items marked OPTIONAL improve accuracy.', styles))
    story.append(SP())

    checklist_data = [
        [Paragraph('', styles['th']),
         Paragraph('Task', styles['th']),
         Paragraph('Priority', styles['th']),
         Paragraph('How to verify', styles['th'])],
        [Paragraph('☐', styles['tc']),
         Paragraph('Plugin camera and confirm RealSense driver starts without errors', styles['tc']),
         Paragraph('REQUIRED', styles['tc']),
         Paragraph('ros2 topic list | grep camera  — should show 10+ topics', styles['tc'])],
        [Paragraph('☐', styles['tc']),
         Paragraph('Inspect factory intrinsics (Section 1.3) — check fx/fy are reasonable', styles['tc']),
         Paragraph('REQUIRED', styles['tc']),
         Paragraph('ros2 topic echo /camera/color/camera_info --once', styles['tc'])],
        [Paragraph('☐', styles['tc']),
         Paragraph('Physically measure camera-to-lidar offset (Section 3.2)', styles['tc']),
         Paragraph('REQUIRED', styles['tc']),
         Paragraph('Ruler/tape measure, record x/y/z to nearest 5 mm', styles['tc'])],
        [Paragraph('☐', styles['tc']),
         Paragraph('Update robot.urdf.xacro base_to_camera joint with measured values', styles['tc']),
         Paragraph('REQUIRED', styles['tc']),
         Paragraph('grep "base_to_camera" urdf/robot.urdf.xacro — no CALIB: comment', styles['tc'])],
        [Paragraph('☐', styles['tc']),
         Paragraph('Rebuild robot_description package', styles['tc']),
         Paragraph('REQUIRED', styles['tc']),
         Paragraph('colcon build --packages-select robot_description', styles['tc'])],
        [Paragraph('☐', styles['tc']),
         Paragraph('Verify TF tree in RViz2 (camera_link at expected position)', styles['tc']),
         Paragraph('REQUIRED', styles['tc']),
         Paragraph('ros2 run tf2_tools view_frames  →  inspect frames.pdf', styles['tc'])],
        [Paragraph('☐', styles['tc']),
         Paragraph('Verify NTP clock synchronisation on Jetson', styles['tc']),
         Paragraph('REQUIRED', styles['tc']),
         Paragraph('timedatectl status → "System clock synchronized: yes"', styles['tc'])],
        [Paragraph('☐', styles['tc']),
         Paragraph('Run timestamp offset check (Section 5.3) — offset < 50 ms', styles['tc']),
         Paragraph('REQUIRED', styles['tc']),
         Paragraph('Run python3 stamp check script — prints offset in ms', styles['tc'])],
        [Paragraph('☐', styles['tc']),
         Paragraph('Manual intrinsic recalibration if factory values look wrong', styles['tc']),
         Paragraph('OPTIONAL', styles['tc']),
         Paragraph('scripts/calibrate_camera_intrinsics.sh  (Section 2)', styles['tc'])],
        [Paragraph('☐', styles['tc']),
         Paragraph('Kalibr precision cam-lidar extrinsic calibration', styles['tc']),
         Paragraph('OPTIONAL', styles['tc']),
         Paragraph('scripts/calibrate_cam_lidar.sh + offline Kalibr processing', styles['tc'])],
        [Paragraph('☐', styles['tc']),
         Paragraph('Enable RealSense hardware sync (enable_sync:=true)', styles['tc']),
         Paragraph('OPTIONAL', styles['tc']),
         Paragraph('ros2 topic hz /camera/color/image_raw — stable 30 Hz', styles['tc'])],
    ]
    checklist = Table(checklist_data, colWidths=[0.8*cm, 7.5*cm, 2.5*cm, 6.6*cm])
    checklist.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), C_NAVY),
        ('ROWBACKGROUNDS', (0,1), (-1,-1), [C_LGREY, colors.white]),
        ('BACKGROUND', (2,9), (2,9), colors.HexColor('#DBEAFE')),
        ('BACKGROUND', (2,10), (2,10), colors.HexColor('#DBEAFE')),
        ('BACKGROUND', (2,11), (2,11), colors.HexColor('#DBEAFE')),
        ('GRID', (0,0), (-1,-1), 0.4, C_MGREY),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('TOPPADDING', (0,0), (-1,-1), 5),
        ('BOTTOMPADDING', (0,0), (-1,-1), 5),
        ('LEFTPADDING', (0,0), (-1,-1), 5),
    ]))
    story += [checklist, SP()]

    story.append(H('Quick reference — files to edit', 'h2', styles))
    tbl2_data = [
        [Paragraph('File', styles['th']),
         Paragraph('What to change', styles['th']),
         Paragraph('When', styles['th'])],
        [Paragraph('src/robot_description/urdf/robot.urdf.xacro', styles['tc']),
         Paragraph('base_to_camera joint xyz/rpy — replace placeholder', styles['tc']),
         Paragraph('After physical measurement (required)', styles['tc'])],
        [Paragraph('src/challenge_bringup/config/color_camera_info.yaml\n(create new)', styles['tc']),
         Paragraph('Custom intrinsic calibration YAML from camera_calibration', styles['tc']),
         Paragraph('Only if manual intrinsic recalibration done', styles['tc'])],
        [Paragraph('src/challenge_bringup/launch/challenge_master.launch.py', styles['tc']),
         Paragraph('Add color_camera_info_url param to realsense2 node', styles['tc']),
         Paragraph('Only if custom intrinsics file created above', styles['tc'])],
    ]
    tbl2 = Table(tbl2_data, colWidths=[5.5*cm, 6.5*cm, 5.4*cm])
    tbl2.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), C_NAVY),
        ('ROWBACKGROUNDS', (0,1), (-1,-1), [C_LGREY, colors.white]),
        ('GRID', (0,0), (-1,-1), 0.4, C_MGREY),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('TOPPADDING', (0,0), (-1,-1), 5),
        ('BOTTOMPADDING', (0,0), (-1,-1), 5),
        ('LEFTPADDING', (0,0), (-1,-1), 5),
    ]))
    story += [tbl2, SP()]

    story.append(H('Further reading', 'h2', styles))
    story += B([
        'Intel RealSense ROS 2 wrapper: https://github.com/IntelRealSense/realsense-ros',
        'ROS 2 camera_calibration package: https://github.com/ros-perception/image_pipeline/tree/humble/camera_calibration',
        'Kalibr multi-camera calibration: https://github.com/ethz-asl/kalibr',
        'Intel RealSense SDK calibration tool: rs-calibrate (included in librealsense)',
        'ROS 2 TF2 tutorial: https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html',
    ], styles)

    return story


# ─────────────────────────────────────────────────────────────────────────────
# Page template callbacks
# ─────────────────────────────────────────────────────────────────────────────
def _on_page(canvas, doc, styles):
    canvas.saveState()
    W, H = A4
    canvas.setFillColor(C_NAVY)
    canvas.rect(0, H - 1.2*cm, W, 1.2*cm, fill=True, stroke=False)
    canvas.setFont('Helvetica-Bold', 8)
    canvas.setFillColor(C_WHITE)
    canvas.drawString(1.8*cm, H - 0.8*cm, 'Camera Calibration Guide')
    canvas.setFont('Helvetica', 8)
    canvas.drawRightString(W - 1.8*cm, H - 0.8*cm,
                           'DIY Challenge Repo  ·  RealSense D435i  ·  Hesai QT64')
    canvas.setFont('Helvetica', 7.5)
    canvas.setFillColor(C_MGREY)
    canvas.drawCentredString(W / 2, 0.7*cm, f'Page {doc.page}')
    canvas.restoreState()


def _on_cover(canvas, doc):
    W, H = A4
    canvas.setFillColor(C_NAVY)
    canvas.rect(0, 0, W, H, fill=True, stroke=False)
    canvas.setFillColor(C_RS)
    canvas.rect(0, H * 0.58, W, 0.35*cm, fill=True, stroke=False)


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────
def main():
    styles = make_styles()

    doc = SimpleDocTemplate(
        OUT_PATH,
        pagesize=A4,
        leftMargin=1.8*cm, rightMargin=1.8*cm,
        topMargin=1.8*cm,  bottomMargin=1.6*cm,
        title='Camera Calibration Guide',
        author='DIY Challenge Repo',
    )

    story = build_story(styles)

    doc.build(
        story,
        onFirstPage=_on_cover,
        onLaterPages=lambda c, d: _on_page(c, d, styles),
    )
    print(f'PDF generated: {OUT_PATH}')


if __name__ == '__main__':
    main()
