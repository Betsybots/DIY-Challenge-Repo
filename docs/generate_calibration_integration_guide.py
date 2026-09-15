#!/usr/bin/env python3
"""
generate_calibration_integration_guide.py — Calibration Integration Guide
Produces docs/Calibration_Integration_Guide.pdf using ReportLab.
Run from the repo root:   python3 docs/generate_calibration_integration_guide.py
"""

import os

from reportlab.lib.pagesizes import A4
from reportlab.lib.units import cm
from reportlab.lib import colors
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.enums import TA_LEFT, TA_CENTER, TA_JUSTIFY
from reportlab.platypus import (
    SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle,
    PageBreak, HRFlowable, Preformatted,
)

# ── Output path ───────────────────────────────────────────────────────────────
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_PATH  = os.path.join(REPO_ROOT, 'docs', 'Calibration_Integration_Guide.pdf')

# ── Colour palette ─────────────────────────────────────────────────────────────
C_NAVY   = colors.HexColor('#1A2744')
C_BLUE   = colors.HexColor('#2563EB')
C_TEAL   = colors.HexColor('#0F766E')
C_ORANGE = colors.HexColor('#EA580C')
C_GREEN  = colors.HexColor('#16A34A')
C_RED    = colors.HexColor('#DC2626')
C_LGREY  = colors.HexColor('#F1F5F9')
C_MGREY  = colors.HexColor('#94A3B8')
C_BLACK  = colors.HexColor('#0F172A')
C_WHITE  = colors.white
C_AMBER  = colors.HexColor('#D97706')
C_CALIB  = colors.HexColor('#FEF3C7')  # highlight for CALIB markers


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
    s['h3'] = ParagraphStyle('H3', fontSize=11, leading=15, textColor=C_TEAL,
        spaceBefore=8, spaceAfter=3, fontName='Helvetica-BoldOblique')
    s['body'] = ParagraphStyle('Body', fontSize=10, leading=14, textColor=C_BLACK,
        spaceAfter=6, alignment=TA_JUSTIFY)
    s['bullet'] = ParagraphStyle('Bullet', fontSize=10, leading=13, textColor=C_BLACK,
        spaceAfter=3, leftIndent=14, bulletIndent=4)
    s['code'] = ParagraphStyle('Code', fontName='Courier',
        fontSize=8.5, leading=12, textColor=C_BLACK,
        backColor=C_LGREY, borderPad=6, spaceAfter=6)
    s['code_calib'] = ParagraphStyle('CodeCalib', fontName='Courier',
        fontSize=8.5, leading=12, textColor=colors.HexColor('#92400E'),
        backColor=C_CALIB, borderPad=6, spaceAfter=6)
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
    s['tc_warn'] = ParagraphStyle('TCWarn', fontName='Helvetica-Bold',
        fontSize=9, textColor=C_RED, leading=13)
    return s


def H(text, level, styles): return Paragraph(text, styles[level])
def P(text, styles): return Paragraph(text, styles['body'])
def B(items, styles): return [Paragraph(f'• {item}', styles['bullet']) for item in items]

def code_block(label, text, styles, calib=False):
    out = []
    if label:
        out.append(Paragraph(label, styles['code_label']))
    key = 'code_calib' if calib else 'code'
    out.append(Preformatted(text, styles[key]))
    return out

def hr(styles): return HRFlowable(width='100%', thickness=0.5, color=C_MGREY, spaceAfter=8)
def warn_box(t, styles): return Paragraph('WARNING:  ' + t, styles['warn'])
def note_box(t, styles): return Paragraph('NOTE:  ' + t, styles['note'])
def tip_box(t, styles): return Paragraph('TIP:  ' + t, styles['tip'])
def SP(n=6): return Spacer(1, n)


def calib_map_table(rows, styles):
    """Render the master CALIB location table."""
    header = [
        Paragraph('File', styles['th']),
        Paragraph('Parameter', styles['th']),
        Paragraph('Current (Placeholder)', styles['th']),
        Paragraph('Source of Correct Value', styles['th']),
    ]
    data = [header] + rows
    t = Table(data, colWidths=[4.5*cm, 4*cm, 3.5*cm, 4.5*cm])
    t.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), C_NAVY),
        ('ROWBACKGROUNDS', (0,1), (-1,-1), [C_WHITE, C_CALIB]),
        ('FONTSIZE', (0,0), (-1,-1), 8),
        ('VALIGN', (0,0), (-1,-1), 'TOP'),
        ('GRID', (0,0), (-1,-1), 0.4, C_MGREY),
        ('TOPPADDING', (0,0), (-1,-1), 3),
        ('BOTTOMPADDING', (0,0), (-1,-1), 3),
        ('WORDWRAP', (0,0), (-1,-1), 1),
    ]))
    return t


def _on_cover(canvas, doc):
    w, h = A4
    canvas.setFillColor(C_NAVY)
    canvas.rect(0, h - 7*cm, w, 7*cm, fill=1, stroke=0)
    canvas.setFillColor(C_TEAL)
    canvas.rect(0, h - 7.35*cm, w, 0.35*cm, fill=1, stroke=0)

def _on_page(canvas, doc):
    w, h = A4
    canvas.setFillColor(C_LGREY)
    canvas.rect(0, 0, w, 1.4*cm, fill=1, stroke=0)
    canvas.setFont('Helvetica', 8)
    canvas.setFillColor(C_MGREY)
    canvas.drawString(2*cm, 0.55*cm, 'DIY Challenge — Calibration Integration Guide')
    canvas.drawRightString(w - 2*cm, 0.55*cm, f'Page {doc.page}')


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
    def CB(lbl, txt, calib=False): return code_block(lbl, txt, styles, calib)
    def HR(): return hr(styles)
    def WARN(t): return warn_box(t, styles)
    def NOTE(t): return note_box(t, styles)
    def TIP(t): return tip_box(t, styles)

    story = []

    # ── Cover ──────────────────────────────────────────────────────────────────
    story.append(SP(60))
    story.append(Paragraph('DIY Challenge Robot', styles['title']))
    story.append(SP(4))
    story.append(Paragraph('Calibration Integration Guide', styles['title']))
    story.append(SP(10))
    story.append(Paragraph(
        'Master Map of All CALIB: Markers • IMU Noise Parameters<br/>'
        'Lidar-IMU Extrinsics • Camera-Lidar Extrinsics • GPS Lever Arm<br/>'
        'Where to Put Results &amp; Validation Checklist',
        styles['subtitle']))
    story.append(SP(6))
    story.append(Paragraph('ROS 2 Humble • All config files cross-referenced',
        styles['subtitle']))
    story.append(PageBreak())

    # ── TOC ────────────────────────────────────────────────────────────────────
    toc_data = [
        [Paragraph('Section', styles['th']), Paragraph('Topic', styles['th'])],
        ['1', 'Master Table of All CALIB Placeholders'],
        ['2', 'IMU Noise Calibration → fast_lio_hesai_qt64.yaml'],
        ['3', 'Lidar-IMU Extrinsic → fast_lio_hesai_qt64.yaml + robot.urdf.xacro'],
        ['4', 'Camera-Lidar Extrinsic → robot.urdf.xacro (base_to_camera)'],
        ['5', 'GPS Lever Arm → navsat_transform.yaml'],
        ['6', 'EKF Process Noise Tuning → ekf_local.yaml'],
        ['7', 'Rebuild & Validation Checklist'],
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

    # ── §1 Master CALIB Map ────────────────────────────────────────────────────
    story.append(H1('1  Master Table of All CALIB Placeholders'))
    story.append(Pb(
        'Every CALIB: comment in the repo represents a value that <b>must be replaced</b> '
        'with a measured or calibrated result before autonomous operation. '
        'The table below is the single reference for all placeholder locations.'
    ))
    story.append(WARN(
        'Running FAST-LIO2 or LIO-SAM with placeholder extrinsic values (identity matrix) '
        'will produce severely degraded localization. Replace all CALIB placeholders before '
        'the first mapping session.'
    ))
    story.append(SP(6))

    calib_rows = [
        ['fast_lio_hesai_qt64.yaml\nmapping.acc_cov',
         'acc_cov: 0.1',
         '0.1 (placeholder)',
         'imu_utils Allen-variance output (N_a^2)'],
        ['fast_lio_hesai_qt64.yaml\nmapping.gyr_cov',
         'gyr_cov: 0.1',
         '0.1 (placeholder)',
         'imu_utils Allen-variance output (N_g^2)'],
        ['fast_lio_hesai_qt64.yaml\nmapping.b_acc_cov',
         'b_acc_cov: 0.0001',
         '0.0001 (placeholder)',
         'imu_utils bias instability (B_a^2)'],
        ['fast_lio_hesai_qt64.yaml\nmapping.b_gyr_cov',
         'b_gyr_cov: 0.0001',
         '0.0001 (placeholder)',
         'imu_utils bias instability (B_g^2)'],
        ['fast_lio_hesai_qt64.yaml\nmapping.extrinsic_T',
         'extrinsic_T: [0,0,0]',
         '[0,0,0] (identity)',
         'lidar_imu_calib translation output (metres)'],
        ['fast_lio_hesai_qt64.yaml\nmapping.extrinsic_R',
         'extrinsic_R: [1,0,0, 0,1,0, 0,0,1]',
         'Identity matrix',
         'lidar_imu_calib rotation matrix output (row-major)'],
        ['robot.urdf.xacro\nbase_to_lidar origin',
         'xyz="0.0 0.0 0.38" rpy="0 0 0"',
         'Physical mount approx',
         'lidar_imu_calib result + tape measure'],
        ['robot.urdf.xacro\nbase_to_imu origin',
         'xyz="0.0 0.0 0.25" rpy="0 0 0"',
         'Physical mount approx',
         'Tape measure from base_link origin'],
        ['robot.urdf.xacro\nbase_to_camera origin',
         'xyz="0.20 0.0 0.20" rpy="0 0 0"',
         'PLACEHOLDER — tape measure only',
         'calibrate_cam_lidar.sh extrinsic result'],
        ['robot.urdf.xacro\nbase_to_gps origin',
         'xyz="0.0 0.0 0.42" rpy="0 0 0"',
         'Physical mount approx',
         'Tape measure from base_link to antenna phase centre'],
        ['navsat_transform.yaml\nmagnetic_declination_radians',
         '0.0',
         '0.0 (will cause heading error)',
         'NOAA magnetic declination for competition site'],
        ['navsat_transform.yaml\nyaw_offset',
         '1.5708 rad (π/2)',
         'Assumes robot faces East',
         'Measured: angle from robot +X to magnetic North (CW)'],
        ['ekf_local.yaml\nprocess_noise_covariance',
         'Diagonal 0.05–0.06',
         'Tunable starting point',
         'AprilTag calibration sessions (diy-sad.html §6.3)'],
    ]

    # Convert to Paragraph cells
    para_rows = []
    for row in calib_rows:
        para_rows.append([Paragraph(cell, styles['tc']) for cell in row])

    story.append(calib_map_table(para_rows, styles))
    story.append(SP(8))
    story.append(NOTE(
        'The highlighted rows (amber background) are CALIB placeholders. '
        'After calibrating, delete the CALIB: comment from the YAML/URDF '
        'and commit the updated values.'
    ))
    story.append(PageBreak())

    # ── §2 IMU Noise Calibration ───────────────────────────────────────────────
    story.append(H1('2  IMU Noise Calibration → fast_lio_hesai_qt64.yaml'))
    story.append(Pb(
        'IMU noise parameters describe the spectral density of white noise (N_a, N_g) '
        'and the random-walk bias instability (B_a, B_g) of the accelerometer and gyroscope. '
        'They are measured from a 2-hour stationary dataset using Allan variance analysis '
        'via the <tt>imu_utils</tt> package.'
    ))

    story.append(H2('2.1  Collect the IMU Dataset'))
    story += CB('Record a 2-hour stationary IMU bag:', '''\
# Place robot on a vibration-free surface (foam pad)
# Must be stationary for the full recording
source scripts/env.sh jetson
ros2 bag record -o ~/imu_calib_bag /imu/data
# Record for at least 2 hours (7200 seconds)
# Stop with Ctrl+C
''')
    story.append(NOTE(
        'QT64 lidar spinning creates vibration that affects the IMU. '
        'Turn off the lidar (disconnect 12V rail) during the IMU calibration recording.'
    ))
    story.append(SP(4))

    story.append(H2('2.2  Run imu_utils Allan Variance'))
    story += CB('Run calibration (imu_utils must be built in third_party_ws):', '''\
source third_party_ws/install/setup.bash
bash scripts/calibrate_imu.sh ~/imu_calib_bag
# Runs imu_utils on the bag; outputs imu_param.yaml
# Output file location: ~/imu_utils_output/imu_param.yaml
''')
    story.append(H2('2.3  Read the Output Values'))
    story += CB('Example imu_utils output (imu_param.yaml):', '''\
Gyr:
  unit: "rad/s"
  avg-axis:
    gyr_n:  0.0034    # ← this is N_g (noise density rad/s/√Hz)
    gyr_w:  0.000085  # ← this is B_g (bias instability rad/s/√Hz)
Acc:
  unit: "m/s2"
  avg-axis:
    acc_n:  0.018     # ← this is N_a (noise density m/s²/√Hz)
    acc_w:  0.00031   # ← this is B_a (bias instability m/s²/√Hz)
''')
    story.append(H2('2.4  Write Values into fast_lio_hesai_qt64.yaml'))
    story += CB('src/localization/config/fast_lio_hesai_qt64.yaml — mapping section:', '''\
# Replace these placeholder values with imu_utils output:
acc_cov:   0.018      # <- acc_n^2 from imu_param.yaml Acc.avg-axis.acc_n
gyr_cov:   0.0034     # <- gyr_n^2 from imu_param.yaml Gyr.avg-axis.gyr_n
b_acc_cov: 0.00031    # <- acc_w from imu_param.yaml Acc.avg-axis.acc_w
b_gyr_cov: 0.000085   # <- gyr_w from imu_param.yaml Gyr.avg-axis.gyr_w
''', calib=True)
    story.append(WARN(
        'The values above are examples. Always use the actual numbers from your imu_param.yaml. '
        'Do not share IMU calibration values between different IMU units — even same-model '
        'units have unit-to-unit variation.'
    ))
    story.append(PageBreak())

    # ── §3 Lidar-IMU Extrinsic ─────────────────────────────────────────────────
    story.append(H1('3  Lidar-IMU Extrinsic → fast_lio_hesai_qt64.yaml + robot.urdf.xacro'))
    story.append(Pb(
        'The lidar-IMU extrinsic is the rigid-body transform from the IMU frame '
        'to the lidar frame. FAST-LIO2 uses it to motion-compensate each lidar scan. '
        'It is measured using <tt>lidar_imu_calib</tt> (target-free, figure-8 motion).'
    ))

    story.append(H2('3.1  Run the Extrinsic Calibration'))
    story += CB('Execute calibrate_extrinsics.sh:', '''\
source scripts/env.sh jetson
bash scripts/calibrate_extrinsics.sh
# Starts lidar_imu_calib
# Drives a figure-8 manoeuvre while recording
# Outputs: extrinsic_T (3-vector, metres)
#          extrinsic_R (3x3 rotation matrix, row-major)
''')
    story += Bl([
        'Drive a smooth figure-8 at moderate speed (~0.3 m/s) in a 4 m × 4 m space.',
        'The figure-8 must excite all 6 degrees of freedom for good observability.',
        'Run the calibration at least 3 times; keep the result with lowest residual.',
        'Target residual: < 0.5 cm translation, < 0.5° rotation.',
    ])
    story.append(SP(4))

    story.append(H2('3.2  Write Extrinsics into FAST-LIO2 Config'))
    story += CB('src/localization/config/fast_lio_hesai_qt64.yaml — mapping section:', '''\
# Replace placeholder identity with lidar_imu_calib output:
extrinsic_T: [ 0.012, -0.003, 0.131 ]   # <- T from calibration (example)
extrinsic_R: [ 0.9998,  0.0021, -0.0198,  # <- R row-major (example)
               -0.0018, 0.9999,  0.0134,
                0.0198, -0.0134, 0.9997 ]
extrinsic_est_en: false   # disable online estimation after calibration
''', calib=True)
    story.append(SP(4))

    story.append(H2('3.3  Write Extrinsics into URDF'))
    story += CB('src/robot_description/urdf/robot.urdf.xacro — base_to_lidar joint:', '''\
<!-- Replace with lidar_imu_calib result.
     xyz = extrinsic_T (metres)
     rpy = rotation matrix to Euler ZYX (radians) -->
<joint name="base_to_lidar" type="fixed">
  <parent link="base_link"/>
  <child link="lidar_link"/>
  <origin xyz="0.012 -0.003 0.131" rpy="0.0 0.0 0.0"/>  <!-- CALIB: example only -->
</joint>
''', calib=True)
    story.append(NOTE(
        'The URDF values and the FAST-LIO2 YAML values must be consistent — '
        'both encode the same lidar→IMU transform. The URDF is used by the TF tree '
        '(RViz visualization, Nav2 footprint), not by FAST-LIO2 directly.'
    ))
    story.append(SP(4))

    story.append(H2('3.4  Timestamp Offset (Optional)'))
    story += CB('scripts/calibrate_extrinsics.sh also outputs a time offset:', '''\
# Time offset between lidar and IMU timestamps (seconds)
# Write into fast_lio_hesai_qt64.yaml:
common:
  time_offset_lidar_to_imu: 0.0005   # <- from spin test; target < 0.001 s
''', calib=True)
    story.append(PageBreak())

    # ── §4 Camera-Lidar Extrinsic ──────────────────────────────────────────────
    story.append(H1('4  Camera-Lidar Extrinsic → robot.urdf.xacro (base_to_camera)'))
    story.append(Pb(
        'The camera-lidar extrinsic locates the RealSense D435i in the robot frame. '
        'It is currently a rough tape-measure placeholder. It is used by Nav2 for '
        'obstacle detection (if depth is enabled) and by RViz for visualization. '
        'For SLAM the extrinsic is less critical than the lidar-IMU extrinsic.'
    ))

    story.append(H2('4.1  Current Placeholder'))
    story += CB('robot.urdf.xacro line 124 (current placeholder):', '''\
<origin xyz="0.20 0.0 0.20" rpy="0.0 0.0 0.0"/>
# xyz: 20 cm forward, 0 lateral, 20 cm above base_link
# rpy: camera faces +X (forward) — approximate
''', calib=True)
    story.append(H2('4.2  Tape-Measure Method (Sufficient for Most Uses)'))
    story += CB('Measure from base_link origin (center of wheel axle, floor level):', '''\
# Required measurements (mm):
#  base_link_x → camera lens centre (along robot forward)
#  base_link_y → camera lens centre (left-right, 0 if centred)
#  base_link_z → camera lens centre (vertical height)
# Convert to metres and enter in robot.urdf.xacro base_to_camera joint:
<origin xyz="0.205 0.0 0.195" rpy="0.0 0.0 0.0"/>   # example
''', calib=True)
    story.append(H2('4.3  Full Extrinsic Calibration (Optional — Use if Depth Obstacles Required)'))
    story += CB('Run calibrate_cam_lidar.sh for precise extrinsic (checkerboard method):', '''\
# Requires checkerboard pattern and stationary lidar+camera running
bash scripts/calibrate_cam_lidar.sh

# Output: 4x4 transformation matrix T_cam_in_lidar
# Convert to base_link frame:
# T_cam_in_base = T_lidar_in_base * T_cam_in_lidar

# Write result into robot.urdf.xacro base_to_camera joint
''')
    story.append(NOTE(
        'The RealSense D435i has factory-calibrated intrinsics stored in EEPROM. '
        'The camera intrinsic calibration (focal length, distortion) does not need to be '
        'repeated unless the camera was disassembled or physically damaged.'
    ))
    story.append(PageBreak())

    # ── §5 GPS Lever Arm ──────────────────────────────────────────────────────
    story.append(H1('5  GPS Lever Arm → navsat_transform.yaml'))
    story.append(Pb(
        'The GPS lever arm is the offset from the robot base_link origin to the '
        'GPS antenna phase centre. It is needed by navsat_transform to correctly '
        'project GPS lat/lon into the robot-frame odometry.'
    ))

    story.append(H2('5.1  Current URDF Value'))
    story += CB('robot.urdf.xacro base_to_gps joint (current):', '''\
<origin xyz="0.0 0.0 0.42" rpy="0.0 0.0 0.0"/>
# GPS antenna 42 cm above base_link — physical measurement
# Verify this matches the actual antenna mounting height
''')
    story.append(H2('5.2  Measure the Lever Arm'))
    story += Bl([
        'Measure in <b>all three axes</b> from the base_link origin (wheel axle centre, '
        'floor level) to the antenna phase centre.',
        'Phase centre is not the top of the antenna connector — it is typically '
        '5–15 mm above the antenna body. Check the antenna datasheet.',
        'Enter the result in robot.urdf.xacro <tt>base_to_gps</tt> and rebuild.',
    ])
    story.append(SP(4))

    story.append(H2('5.3  Magnetic Declination'))
    story += CB('src/localization/config/navsat_transform.yaml:', '''\
# CALIB: Look up the competition site magnetic declination from NOAA:
# https://ngdc.noaa.gov/geomag/calculators/magcalc.shtml
# Enter: competition venue lat/lon, current date; copy "Declination" in radians

magnetic_declination_radians: 0.0    # <- replace with NOAA value (example: -0.2153)
''', calib=True)

    story.append(H2('5.4  yaw_offset'))
    story += CB('Measure the GPS-to-robot heading offset:', '''\
# yaw_offset = angle from robot +X axis to magnetic North, measured clockwise
# Default: π/2 (robot +X faces East)
#
# Simple field measurement:
#  1. Place robot facing a known compass bearing (e.g. due East = 0 deg from North + 90 deg)
#  2. Read GPS heading from /gps/fix or /imu/data
#  3. yaw_offset = GPS heading - robot compass bearing (in radians)

yaw_offset: 1.5707963267948966       # <- replace with measured value
''', calib=True)
    story.append(NOTE(
        'A wrong yaw_offset causes navsat_transform to produce a GPS trajectory that is '
        'rotated relative to the lidar map. The error manifests as the GPS track curving '
        'away from the driven path in RViz. Adjust yaw_offset by the angular discrepancy observed.'
    ))
    story.append(PageBreak())

    # ── §6 EKF Noise Tuning ───────────────────────────────────────────────────
    story.append(H1('6  EKF Process Noise Tuning → ekf_local.yaml'))
    story.append(Pb(
        'The EKF1 (odom frame) process noise covariance matrix Q encodes how much '
        'uncertainty the filter adds to its state estimate each time step. '
        'Too small = filter over-trusts predictions, ignores measurements. '
        'Too large = filter over-trusts measurements, becomes jumpy.'
    ))
    story.append(H2('6.1  Current Values (Starting Point)'))
    story += CB('src/localization/config/ekf_local.yaml — key diagonal values:', '''\
# Q matrix diagonal (position, orientation, linear velocity)
# x=0.05, y=0.05, z=0.06   — position uncertainty per cycle
# yaw=0.06                  — heading uncertainty per cycle
# vx=0.025, vy=0.025        — velocity uncertainty per cycle
''')

    story.append(H2('6.2  Tuning Procedure (AprilTag Calibration)'))
    story.append(Pb(
        'The recommended method from diy-sad.html §6.3 uses AprilTag ground-truth '
        'poses recorded while FAST-LIO2 is running. Drive a known path past AprilTags, '
        'compare EKF output to tag ground truth, and adjust Q values to minimize RMS error.'
    ))
    story += Bl([
        'Mount an AprilTag at a known position in the test area.',
        'Drive the robot at 0.3 m/s past the tag at < 3 m range.',
        'Record a bag: /odometry/filtered, /camera/color/image_raw, /tf',
        'Post-process: compare EKF output to known tag position → compute residual.',
        'Increase diagonal Q value for noisy axes; decrease for under-correcting axes.',
        'Iterate until RMS error < 5 cm over a 10 m run.',
    ])
    story.append(H2('6.3  Quick Sanity Check (No AprilTag)'))
    story += CB('Static test to check EKF covariance convergence:', '''\
# Start localization, keep robot stationary for 30 seconds
# Watch covariance in /odometry/filtered:
ros2 topic echo /odometry/filtered | grep -A 36 "pose.*covariance"

# Good result: x[0], y[7] diagonal entries converge to < 0.01 (static)
# If they grow over time: lidar odometry is drifting; check FAST-LIO2 params
''')
    story.append(PageBreak())

    # ── §7 Rebuild & Validation Checklist ────────────────────────────────────
    story.append(H1('7  Rebuild &amp; Validation Checklist'))
    story.append(Pb(
        'After updating any calibration parameter, follow this checklist to verify '
        'the change is correctly integrated before the next mapping or competition session.'
    ))

    story.append(H2('7.1  After Any YAML Change'))
    checks_yaml = [
        'Confirm YAML syntax is valid: python3 -c "import yaml; yaml.safe_load(open(\'config/fast_lio_hesai_qt64.yaml\'))"',
        'Rebuild localization package: colcon build --packages-select localization',
        'Re-source ROS overlay: source install/setup.bash',
        'Launch localization with use_rviz:=true and confirm no error messages at startup',
        'Confirm FAST-LIO2 prints convergence message (not "IMU not ready")',
    ]
    for item in checks_yaml:
        story.append(Paragraph(f'☐  {item}', styles['bullet']))
    story.append(SP(6))

    story.append(H2('7.2  After URDF Change'))
    checks_urdf = [
        'Rebuild robot_description: colcon build --packages-select robot_description',
        'Confirm robot model in RViz has all links in correct position',
        'Verify TF tree: ros2 run tf2_tools view_frames — all expected frames present',
        'Check base_to_lidar transform: ros2 run tf2_ros tf2_echo base_link lidar_link',
        'Check base_to_camera transform: ros2 run tf2_ros tf2_echo base_link camera_link',
    ]
    for item in checks_urdf:
        story.append(Paragraph(f'☐  {item}', styles['bullet']))
    story.append(SP(6))

    story.append(H2('7.3  After Lidar-IMU Extrinsic Change'))
    checks_ext = [
        'Run a short 2-minute mapping session with offline_mapping.launch.py',
        'Verify map in RViz: walls should be straight verticals, no fan-shaped spread',
        'Drive the figure-8 calibration path again and confirm map closes without gaps',
        'Check that extrinsic_est_en: false in FAST-LIO2 config (disable online estimation)',
        'Compare RMS residual with previous calibration — new value should be lower',
    ]
    for item in checks_ext:
        story.append(Paragraph(f'☐  {item}', styles['bullet']))
    story.append(SP(6))

    story.append(H2('7.4  After GPS Calibration Change'))
    checks_gps = [
        'Drive robot in a straight line East for 20 m on open ground',
        'In RViz: /gps/filtered points should overlap with /odometry/filtered path',
        'Offset < 0.5 m for DGNSS, < 0.05 m for RTK fixed',
        'Check navsat_transform output: /odometry/gps publishes at 10 Hz',
        'Confirm EKF2 /odometry/global stays within 1 m of /odometry/filtered after 60 s',
    ]
    for item in checks_gps:
        story.append(Paragraph(f'☐  {item}', styles['bullet']))
    story.append(SP(6))

    story.append(H2('7.5  Final Pre-Competition CALIB Audit'))
    story += CB('Search for remaining placeholder markers in all config files:', '''\
# Check fast_lio config for placeholder values (0.1 is placeholder for acc/gyr_cov)
grep -n "CALIB" src/localization/config/fast_lio_hesai_qt64.yaml
grep -n "CALIB" src/localization/config/navsat_transform.yaml
grep -n "CALIB" src/robot_description/urdf/robot.urdf.xacro

# Check for identity extrinsics (still placeholder):
grep -n "extrinsic_T: \\[ 0.0, 0.0, 0.0" src/localization/config/fast_lio_hesai_qt64.yaml

# Expected: zero remaining CALIB lines with 0.0 placeholder values
''')
    story.append(WARN(
        'If any grep above returns a line, that calibration step is incomplete. '
        'Do not proceed to competition with placeholder values.'
    ))
    story.append(HR())
    story.append(Pb(
        '<i>End of Calibration Integration Guide. '
        'For the calibration scripts themselves see scripts/calibrate_*.sh. '
        'For detailed camera calibration procedures see the Camera Calibration Guide. '
        'For nav2 parameter tuning after localization is working see the Nav2 Tuning Guide.</i>'
    ))

    doc.build(story, onFirstPage=_on_cover, onLaterPages=_on_page)
    print(f'Written: {OUT_PATH}')


if __name__ == '__main__':
    build_pdf()
