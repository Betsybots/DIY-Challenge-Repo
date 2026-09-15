#!/usr/bin/env python3
"""
generate_apriltag_calibration_guide.py — AprilTag EKF Calibration Guide
Produces docs/AprilTag_Calibration_Guide.pdf using ReportLab.

Run from the repo root:
    python3 docs/generate_apriltag_calibration_guide.py
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
OUT_PATH  = os.path.join(REPO_ROOT, 'docs', 'AprilTag_Calibration_Guide.pdf')

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
C_YELLOW = colors.HexColor('#FEF3C7')
C_PURPLE = colors.HexColor('#7C3AED')


# ─────────────────────────────────────────────────────────────────────────────
def make_styles():
    base = getSampleStyleSheet()
    s = {}
    s['title'] = ParagraphStyle('DocTitle', parent=base['Title'],
        fontSize=28, leading=34, textColor=C_WHITE, spaceAfter=6, alignment=TA_CENTER)
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
    s['bullet2'] = ParagraphStyle('Bullet2', fontSize=9.5, leading=13, textColor=C_BLACK,
        spaceAfter=2, leftIndent=28, bulletIndent=18)
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
    s['crit'] = ParagraphStyle('Crit', fontSize=9.5, leading=13,
        textColor=colors.HexColor('#7F1D1D'),
        backColor=colors.HexColor('#FEE2E2'),
        borderPad=6, spaceAfter=6, fontName='Helvetica-Bold')
    s['th'] = ParagraphStyle('TH', fontName='Helvetica-Bold',
        fontSize=9, textColor=C_WHITE, alignment=TA_CENTER)
    s['tc'] = ParagraphStyle('TC', fontName='Helvetica',
        fontSize=9, textColor=C_BLACK, leading=13)
    s['tc_mono'] = ParagraphStyle('TCMono', fontName='Courier',
        fontSize=8, textColor=C_BLACK, leading=12)
    s['tc_green'] = ParagraphStyle('TCGreen', fontName='Helvetica-Bold',
        fontSize=9, textColor=C_GREEN, leading=13)
    s['tc_red'] = ParagraphStyle('TCRed', fontName='Helvetica-Bold',
        fontSize=9, textColor=C_RED, leading=13)
    s['step_num'] = ParagraphStyle('StepNum', fontName='Helvetica-Bold',
        fontSize=14, textColor=C_WHITE, alignment=TA_CENTER)
    s['step_title'] = ParagraphStyle('StepTitle', fontName='Helvetica-Bold',
        fontSize=12, textColor=C_NAVY, leading=15)
    s['math'] = ParagraphStyle('Math', fontName='Courier',
        fontSize=9.5, leading=14, textColor=colors.HexColor('#1E3A5F'),
        backColor=colors.HexColor('#EFF6FF'),
        borderPad=8, spaceAfter=6)
    return s


def H(text, level, styles):   return Paragraph(text, styles[level])
def P(text, styles):           return Paragraph(text, styles['body'])
def B(items, styles):          return [Paragraph(f'• {item}', styles['bullet']) for item in items]
def B2(items, styles):         return [Paragraph(f'◦ {item}', styles['bullet2']) for item in items]
def code_block(label, text, styles):
    out = []
    if label:
        out.append(Paragraph(label, styles['code_label']))
    out.append(Preformatted(text, styles['code']))
    return out
def hr(styles):    return HRFlowable(width='100%', thickness=0.5, color=C_MGREY, spaceAfter=8)
def WARN(t, s):    return Paragraph('⚠  WARNING:  ' + t, s['warn'])
def NOTE(t, s):    return Paragraph('ℹ  NOTE:  ' + t, s['note'])
def TIP(t, s):     return Paragraph('✔  TIP:  ' + t, s['tip'])
def CRIT(t, s):    return Paragraph('🛑  CRITICAL:  ' + t, s['crit'])
def SP(n=6):       return Spacer(1, n)
def TC(t, s):      return Paragraph(t, s['tc'])
def TCM(t, s):     return Paragraph(t, s['tc_mono'])


def step_table(number, title, description, styles):
    """Renders a numbered step banner row."""
    data = [[
        Paragraph(str(number), styles['step_num']),
        [Paragraph(title, styles['step_title']), SP(2), Paragraph(description, styles['tc'])],
    ]]
    t = Table(data, colWidths=[1.2*cm, 14.3*cm])
    t.setStyle(TableStyle([
        ('BACKGROUND',    (0, 0), (0, 0), C_TEAL),
        ('BACKGROUND',    (1, 0), (1, 0), colors.HexColor('#F0FDFA')),
        ('VALIGN',        (0, 0), (-1, -1), 'MIDDLE'),
        ('TOPPADDING',    (0, 0), (-1, -1), 6),
        ('BOTTOMPADDING', (0, 0), (-1, -1), 6),
        ('LEFTPADDING',   (1, 0), (1, 0), 10),
        ('BOX',           (0, 0), (-1, -1), 0.8, C_TEAL),
        ('ROUNDEDCORNERS', [4]),
    ]))
    return t


def _on_cover(canvas, doc):
    w, h = A4
    canvas.setFillColor(C_NAVY)
    canvas.rect(0, h - 8*cm, w, 8*cm, fill=1, stroke=0)
    canvas.setFillColor(C_TEAL)
    canvas.rect(0, h - 8.4*cm, w, 0.4*cm, fill=1, stroke=0)


def _on_page(canvas, doc):
    w, h = A4
    canvas.setFillColor(C_LGREY)
    canvas.rect(0, 0, w, 1.4*cm, fill=1, stroke=0)
    canvas.setFont('Helvetica', 8)
    canvas.setFillColor(C_MGREY)
    canvas.drawString(2*cm, 0.55*cm, 'DIY Challenge 2026 — AprilTag EKF Calibration Guide')
    canvas.drawRightString(w - 2*cm, 0.55*cm, f'Page {doc.page}')


def simple_table(headers, rows, col_widths, styles):
    header_row = [Paragraph(h, styles['th']) for h in headers]
    data = [header_row]
    for row in rows:
        data.append([Paragraph(cell, styles['tc']) if isinstance(cell, str) else cell for cell in row])
    t = Table(data, colWidths=col_widths)
    t.setStyle(TableStyle([
        ('BACKGROUND',    (0, 0), (-1, 0), C_NAVY),
        ('ROWBACKGROUNDS',(0, 1), (-1, -1), [C_WHITE, C_LGREY]),
        ('GRID',          (0, 0), (-1, -1), 0.4, C_MGREY),
        ('VALIGN',        (0, 0), (-1, -1), 'TOP'),
        ('TOPPADDING',    (0, 0), (-1, -1), 4),
        ('BOTTOMPADDING', (0, 0), (-1, -1), 4),
        ('LEFTPADDING',   (0, 0), (-1, -1), 6),
    ]))
    return t


# =============================================================================
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
    def Bl2(items): return B2(items, styles)
    def CB(lbl, txt): return code_block(lbl, txt, styles)
    def HR(): return hr(styles)
    def ST(n, title, desc): return step_table(n, title, desc, styles)

    story = []

    # ── COVER ─────────────────────────────────────────────────────────────────
    story.append(SP(70))
    story.append(Paragraph('DIY Challenge Robot 2026', styles['title']))
    story.append(SP(6))
    story.append(Paragraph('AprilTag EKF Calibration Guide', styles['title']))
    story.append(SP(12))
    story.append(Paragraph(
        'Equipment Required • Tag Printing &amp; Surveying<br/>'
        'Software Setup • Data Collection Procedure<br/>'
        'Post-Processing &amp; EKF Q/R Matrix Tuning<br/>'
        'Validation &amp; Results',
        styles['subtitle']))
    story.append(SP(8))
    story.append(Paragraph(
        'ROS 2 Humble • FAST-LIO2 • robot_localization EKF • apriltag_ros',
        styles['subtitle']))
    story.append(PageBreak())

    # ── SECTION 1 — Overview ──────────────────────────────────────────────────
    story.append(H1('1  Overview &amp; Purpose'))
    story.append(HR())
    story.append(Pb(
        'AprilTag calibration is a <b>pre-competition, offline tuning session</b> run '
        'in your own test environment — not on the competition course. Its purpose is to '
        'provide centimetre-level ground-truth pose measurements so that the EKF '
        '(robot_localization) Q and R noise matrices can be tuned to match the real-world '
        'sensor behaviour of your specific robot.'
    ))
    story.append(Pb(
        'The RealSense D435i camera detects printed AprilTag markers at known surveyed positions. '
        'The difference between the EKF-predicted pose and the tag-measured ground-truth pose '
        '(the <i>innovation sequence</i>) contains all information needed to tune the filter '
        'parameters automatically. Tighter, more accurate EKF fusion reduces localization drift '
        'per lap from a typical 10–20 cm to under 5 cm — critical for the 2-inch clearance '
        'narrow passages and movable-obstacle avoidance in the competition.'
    ))

    story.append(H2('1.1  What This Session Gives You'))
    story.append(simple_table(
        ['Output', 'How It Helps Competition Performance'],
        [
            ['Per-section drift profile of FAST-LIO2',
             'Reveals where odometry drifts most — tunnel exit, tight turns, gravel'],
            ['Heading accuracy at low speed',
             'RTK single-antenna heading is poor below ~0.3 m/s — tag data quantifies this'],
            ['IMU contribution isolated by speed',
             'Compare fast vs slow laps to separate IMU drift from scan-matching errors'],
            ['Systematic error patterns',
             'e.g. "always drifts left after a right turn" — fix in URDF or EKF config'],
            ['Optimised EKF Q and R matrices',
             'Saved to competition.yaml — 30–60 % drift reduction typical'],
        ],
        [5.5*cm, 10*cm], styles
    ))
    story.append(SP(8))

    story.append(NOTE(
        'AprilTags are NOT present on the competition course. This is purely a '
        'pre-competition calibration tool used in your own test environment.',
        styles
    ))
    story.append(SP(4))

    # ── SECTION 2 — Equipment ─────────────────────────────────────────────────
    story.append(PageBreak())
    story.append(H1('2  Equipment Required'))
    story.append(HR())

    story.append(H2('2.1  AprilTag Hardware'))
    story.append(simple_table(
        ['Item', 'Specification', 'Where to Get', 'Notes'],
        [
            ['Printed AprilTag targets',
             '8–10 tags, Tag36h11 family, IDs 0–9',
             'Print free from april.eecs.umich.edu or generate with apriltag Python lib',
             'Print on plain paper, mount on flat rigid board (foam board, cardboard). '
             'Minimum 20 × 20 cm per tag.'],
            ['Rigid mounting boards',
             '~40 × 40 cm, flat, non-reflective surface',
             'Foam board / foam core from stationery shop',
             'Prevents tag warping. Warp causes pose estimation error.'],
            ['Ground stakes or stands',
             'Anything that holds boards vertical and stable',
             'Camera tripods, tent stakes, zip-tied to fence posts',
             'Tags must face the robot path. Keep them at camera height (0.3–1.0 m).'],
            ['RTK rover / base station',
             'Your existing RTK GNSS setup',
             'Already in hardware inventory',
             'Used to survey exact (x, y, z) position of each tag centre to < 2 cm accuracy.'],
            ['Measuring tape',
             '5 m minimum',
             'Hardware store',
             'Backup measurement to cross-check RTK survey.'],
        ],
        [3*cm, 3.5*cm, 4*cm, 5*cm], styles
    ))

    story.append(H2('2.2  Robot Equipment'))
    story += Bl([
        '<b>RealSense D435i</b> — already on robot, used for tag detection via RGB stream',
        '<b>Jetson Nano</b> — running full ROS 2 Humble stack',
        '<b>Hesai QT64</b> — running FAST-LIO2 for odometry during laps',
        '<b>RTK GNSS</b> — running for EKF2 global frame during laps',
        '<b>Joystick</b> — for driving laps (JOYSTICK mode, not autonomous)',
        '<b>Large SSD / USB drive</b> — rosbag from 10+ laps at full rate is 5–15 GB',
    ])
    story.append(SP(8))

    story.append(WARN(
        'Do NOT use the D435i built-in IMU during this session. Use only the dedicated '
        'external IMU. USB latency jitter from the D435i causes timestamp drift that '
        'corrupts FAST-LIO2 fusion.',
        styles
    ))

    # ── SECTION 3 — Software Setup ────────────────────────────────────────────
    story.append(PageBreak())
    story.append(H1('3  Software Setup'))
    story.append(HR())

    story.append(H2('3.1  Install apriltag_ros'))
    story.append(Pb(
        'The repo references <b>christianrauch/apriltag_ros</b> — the standard ROS 2 Humble '
        'compatible package. Install via apt:'
    ))
    story += CB('Install apriltag_ros',
        'sudo apt update\n'
        'sudo apt install ros-humble-apriltag-ros\n\n'
        '# Verify\n'
        'ros2 pkg prefix apriltag_ros'
    )

    story.append(H2('3.2  Camera Intrinsics Requirement'))
    story.append(Pb(
        'AprilTag pose estimation requires accurate camera intrinsics. If you have '
        'already run <b>scripts/calibrate_camera_intrinsics.sh</b> and your D435i '
        'intrinsics are stored, you are ready. If not, run camera intrinsics calibration '
        'first — see Camera_Calibration_Guide.pdf.'
    ))
    story += CB('Check camera info topic is publishing intrinsics',
        'ros2 topic echo /camera/color/camera_info --once'
    )

    story.append(H2('3.3  Tag Configuration File'))
    story.append(Pb(
        'Create a tag configuration YAML that lists the tag family and sizes. '
        'Save as <b>src/challenge_bringup/config/apriltag_tags.yaml</b>:'
    ))
    story += CB('src/challenge_bringup/config/apriltag_tags.yaml',
        'tag_family: 36h11\n'
        'tag_threads: 2\n'
        'tag_decimate: 1.0\n'
        'tag_blur: 0.0\n'
        'tag_refine_edges: 1\n'
        'tag_debug: 0\n'
        'max_hamming_dist: 0\n\n'
        'standalone_tags:\n'
        '  - {id: 0, size: 0.20}   # tag physical size in metres — adjust to your printout\n'
        '  - {id: 1, size: 0.20}\n'
        '  - {id: 2, size: 0.20}\n'
        '  - {id: 3, size: 0.20}\n'
        '  - {id: 4, size: 0.20}\n'
        '  - {id: 5, size: 0.20}\n'
        '  - {id: 6, size: 0.20}\n'
        '  - {id: 7, size: 0.20}\n'
        '  - {id: 8, size: 0.20}\n'
        '  - {id: 9, size: 0.20}'
    )
    story.append(NOTE(
        'The "size" field is the length of one side of the black tag border in metres. '
        'Measure your printed tag — not the paper size. Incorrect size causes direct '
        'proportional error in the distance estimate.',
        styles
    ))

    story.append(H2('3.4  Launch apriltag_ros with D435i'))
    story.append(Pb(
        'Run the apriltag_ros node pointed at the D435i colour stream. '
        'This can be done in a separate terminal alongside the main stack:'
    ))
    story += CB('Terminal — run apriltag detection',
        '# Source ROS and your workspace\n'
        'source /opt/ros/humble/setup.bash\n'
        'source install/setup.bash\n\n'
        'ros2 run apriltag_ros apriltag_node --ros-args \\\n'
        '  -r image_rect:=/camera/color/image_raw \\\n'
        '  -r camera_info:=/camera/color/camera_info \\\n'
        '  --params-file src/challenge_bringup/config/apriltag_tags.yaml'
    )
    story.append(Pb('Detections publish to: <b>/detections</b> (apriltag_ros/msg/AprilTagDetectionArray)'))

    # ── SECTION 4 — Physical Setup ────────────────────────────────────────────
    story.append(PageBreak())
    story.append(H1('4  Physical Tag Placement &amp; Surveying'))
    story.append(HR())

    story.append(H2('4.1  Placement Rules'))
    story += Bl([
        'Use <b>8–10 tags</b> spread around the entire test track perimeter',
        'Place tags so <b>at least 2 are visible</b> from any position on the track',
        'Face tags toward the inside of the track (toward the robot path)',
        'Height: tag centre at <b>0.3 m to 0.8 m</b> above ground — within D435i field of view',
        'Spacing: roughly every 3–5 m around the track',
        'Avoid placing tags in deep shadow or directly facing the sun',
        'Secure boards firmly — any movement between laps invalidates the ground truth',
    ])

    story.append(H2('4.2  Surveying Tag Positions with RTK'))
    story.append(Pb(
        'Each tag must have its <b>centre position measured in the same world frame</b> '
        'that your EKF uses (typically UTM or local ENU origin set at robot start). '
        'Survey each tag with the RTK rover and record the values in a JSON file:'
    ))
    story += CB('calibration/tag_survey.json  (create manually after surveying)',
        '{\n'
        '  "frame_id": "map",\n'
        '  "tags": [\n'
        '    {"id": 0, "x":  0.00, "y":  0.00, "z": 0.50, "yaw_deg": 0.0},\n'
        '    {"id": 1, "x":  5.20, "y":  0.15, "z": 0.50, "yaw_deg": 90.0},\n'
        '    {"id": 2, "x": 10.45, "y":  0.10, "z": 0.50, "yaw_deg": 90.0},\n'
        '    {"id": 3, "x": 10.50, "y":  8.30, "z": 0.50, "yaw_deg": 180.0},\n'
        '    {"id": 4, "x":  5.10, "y":  8.25, "z": 0.50, "yaw_deg": 270.0},\n'
        '    {"id": 5, "x":  0.05, "y":  8.20, "z": 0.50, "yaw_deg": 270.0},\n'
        '    {"id": 6, "x":  2.50, "y":  4.10, "z": 0.50, "yaw_deg": 0.0},\n'
        '    {"id": 7, "x":  7.80, "y":  4.00, "z": 0.50, "yaw_deg": 180.0},\n'
        '    {"id": 8, "x":  1.20, "y":  2.50, "z": 0.50, "yaw_deg": 45.0},\n'
        '    {"id": 9, "x":  9.10, "y":  6.50, "z": 0.50, "yaw_deg": 135.0}\n'
        '  ]\n'
        '}'
    )
    story.append(NOTE(
        'yaw_deg is the direction the tag FACES (i.e. the direction the robot must be '
        'approaching from to see it). 0° = facing +X axis (East). This is used only '
        'for documentation — apriltag_ros computes the full 6DOF pose from the image.',
        styles
    ))

    # ── SECTION 5 — Data Collection ───────────────────────────────────────────
    story.append(PageBreak())
    story.append(H1('5  Data Collection Procedure'))
    story.append(HR())

    story.append(H2('5.1  Pre-session Checklist'))
    story.append(simple_table(
        ['Check', 'Pass Condition'],
        [
            ['All tags firmly mounted and surveyed', 'tag_survey.json complete with all IDs'],
            ['D435i publishing /camera/color/image_raw', 'ros2 topic hz shows ~30 Hz'],
            ['apriltag_ros node running, /detections publishing', 'ros2 topic echo /detections shows tag IDs as you walk past'],
            ['FAST-LIO2 running and converged', '/odometry/fast_lio topic publishing, no drift warnings'],
            ['RTK fix acquired', '/fix topic shows status=2 (RTK fixed)'],
            ['EKF1 and EKF2 running', 'ros2 node list shows ekf_local and ekf_global nodes'],
            ['Rosbag recording started', 'See step 5.2'],
            ['SSD has >20 GB free', 'df -h'],
        ],
        [7*cm, 8.5*cm], styles
    ))

    story.append(H2('5.2  Recording the Calibration Bag'))
    story.append(Pb(
        'Record a bag containing all topics needed for post-processing. '
        'Drive <b>10 or more laps</b> at competition speed with the joystick. '
        'Vary speed slightly between laps — slow laps isolate IMU drift, fast laps test '
        'scan-matching limits.'
    ))
    story += CB('Record calibration bag',
        'LABEL="apriltag_calib_$(date +%Y%m%d_%H%M%S)"\n'
        'mkdir -p calibration\n\n'
        'ros2 bag record \\\n'
        '  /tf \\\n'
        '  /tf_static \\\n'
        '  /odometry/fast_lio \\\n'
        '  /odometry/filtered \\\n'
        '  /odometry/global \\\n'
        '  /fix \\\n'
        '  /imu/data \\\n'
        '  /detections \\\n'
        '  /camera/color/camera_info \\\n'
        '  --output calibration/${LABEL}'
    )
    story.append(NOTE(
        'Record /detections in addition to /tf — this is the raw apriltag_ros output '
        'containing the 6DOF pose of each detected tag relative to the camera.',
        styles
    ))

    story.append(H2('5.3  During the Session'))
    story += Bl([
        'Drive at <b>normal competition speed</b> (0.4–0.6 m/s) for the first 5 laps',
        'Drive 3 laps at <b>slow speed</b> (~0.15 m/s) to isolate IMU vs scan-matching drift',
        'Drive 2 laps at <b>fast speed</b> (~0.7 m/s) to test high-speed performance',
        'Do <b>not stop</b> mid-lap — continuous motion improves FAST-LIO2 accuracy',
        'Ensure each tag is within camera view range (<b>max ~3 m</b> for 20 cm tags) '
        'at some point each lap',
        'After all laps: stop recording, note the bag filename',
    ])

    # ── SECTION 6 — Post-Processing ───────────────────────────────────────────
    story.append(PageBreak())
    story.append(H1('6  Post-Processing &amp; EKF Tuning'))
    story.append(HR())

    story.append(H2('6.1  What the Post-Processing Script Does'))
    story.append(Pb(
        'A Python script reads the recorded rosbag and tag_survey.json, then:'
    ))
    story += Bl([
        'For each /detections message: looks up the EKF-estimated robot pose at that timestamp from /tf',
        'Computes the expected tag pose in map frame using the surveyed ground truth',
        'Computes the <b>innovation</b> = (observed tag pose in map) − (EKF-predicted tag pose in map)',
        'Accumulates all innovations across all laps and all tags',
        'Computes optimal R (measurement noise) and Q (process noise) matrices from the innovation statistics',
        'Writes updated ekf_local.yaml and ekf_global.yaml with new covariance values',
    ])

    story.append(H2('6.2  The Math (Innovation-Based Adaptive EKF)'))
    story.append(Paragraph(
        'For each tag observation k, the innovation vector is:',
        styles['body']
    ))
    story.append(Paragraph(
        '    ν_k  =  z_k  −  H · x̂_k\n\n'
        'where:\n'
        '    z_k   = observed tag pose in map frame (from apriltag + surveyed position)\n'
        '    x̂_k  = EKF-predicted robot pose at time k  (from /odometry/filtered)\n'
        '    H     = observation matrix (identity for pose observations)\n\n'
        'Optimal measurement noise covariance:\n'
        '    R_optimal  =  mean( ν_k · ν_k^T )  over all observations\n\n'
        'Residual process noise estimate:\n'
        '    Q_optimal  =  estimated from residual between filter output and ground truth\n'
        '                  using the Sage-Husa adaptive method',
        styles['math']
    ))

    story.append(H2('6.3  Running the Post-Processing Script'))
    story.append(Pb(
        'Run the provided post-processing script from the repo root. '
        'Pass the path to your recorded bag and the survey JSON:'
    ))
    story += CB('Post-process bag and compute optimal EKF parameters',
        'python3 scripts/tune_ekf_apriltag.py \\\n'
        '  --bag    calibration/apriltag_calib_20261005_143021 \\\n'
        '  --survey calibration/tag_survey.json \\\n'
        '  --output calibration/ekf_tuning_results.yaml\n\n'
        '# The script prints per-tag drift statistics and writes:\n'
        '#   calibration/ekf_tuning_results.yaml  — new Q/R values\n'
        '#   calibration/drift_plot.png            — drift profile per section'
    )
    story.append(WARN(
        'scripts/tune_ekf_apriltag.py does not yet exist in the repo — it needs to '
        'be written. Section 7 describes exactly what it should do. This is the '
        'recommended task for one teammate to own.',
        styles
    ))

    story.append(H2('6.4  Applying Results to EKF Config'))
    story.append(Pb(
        'Once the script outputs ekf_tuning_results.yaml, copy the values into '
        '<b>src/localization/config/ekf_local.yaml</b>. '
        'The relevant parameters are the process_noise_covariance and '
        'observation_model_type entries:'
    ))
    story += CB('src/localization/config/ekf_local.yaml  (update these values)',
        '# ── EKF Process Noise (Q matrix diagonal) ───────────────────────────\n'
        '# Update with values from calibration/ekf_tuning_results.yaml\n'
        'process_noise_covariance: [\n'
        '  0.05,  0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,\n'
        '  0.0,   0.05,  0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0,\n'
        '  ...    # 15x15 matrix — paste from tuning output\n'
        ']'
    )
    story.append(TIP(
        'Save the pre-tuning ekf_local.yaml as ekf_local_default.yaml before '
        'overwriting, so you can revert if the tuned values perform worse.',
        styles
    ))

    # ── SECTION 7 — Script Spec ───────────────────────────────────────────────
    story.append(PageBreak())
    story.append(H1('7  tune_ekf_apriltag.py — Implementation Specification'))
    story.append(HR())
    story.append(Pb(
        'This section is the complete specification for the post-processing script '
        'that needs to be written. One teammate should own this task.'
    ))

    story.append(H2('7.1  Dependencies'))
    story += CB('pip install dependencies',
        'pip3 install rosbags numpy scipy matplotlib pyyaml transforms3d'
    )
    story.append(Pb(
        '<b>rosbags</b> is used instead of rclpy bag playback — it reads bags offline '
        'without requiring a running ROS 2 node, which makes the script faster and '
        'runnable on any machine (laptop, not just Jetson).'
    ))

    story.append(H2('7.2  Algorithm Steps'))
    steps = [
        ('Load tag_survey.json',
         'Parse tag ground-truth positions into a dict: {tag_id: (x, y, z)} in map frame.'),
        ('Open rosbag with rosbags library',
         'Read messages from /detections, /tf, /odometry/filtered topics.'),
        ('Build TF tree from /tf and /tf_static',
         'Use the rosbags TF buffer to resolve transforms between frames at any timestamp.'),
        ('For each /detections message',
         'Extract timestamp t, tag ID, and pose of tag relative to camera_link frame. '
         'Lookup robot pose (map → base_link) at time t from TF buffer. '
         'Transform tag detection into map frame using robot pose + camera_link transform. '
         'Compare to surveyed ground-truth position → compute innovation vector ν_k.'),
        ('Accumulate innovations per tag and per track section',
         'Group innovations by tag ID and by lap number. '
         'Track section = which tag was nearest when the observation was made.'),
        ('Compute R_optimal',
         'R_optimal = mean(ν_k · ν_k^T) over all observations. '
         'Extract x, y, heading components for ekf_local.yaml.'),
        ('Compute Q estimate',
         'Use Sage-Husa approximation or simple residual variance between '
         '/odometry/filtered and ground truth interpolated between tag observations.'),
        ('Write output YAML',
         'Write calibration/ekf_tuning_results.yaml with new Q diagonal and R diagonal values, '
         'plus per-tag drift statistics (mean, std, max).'),
        ('Plot drift profile',
         'Matplotlib figure: x-axis = track position (m along path), '
         'y-axis = EKF error (cm). Save as calibration/drift_plot.png.'),
    ]
    for i, (title, desc) in enumerate(steps, 1):
        story.append(ST(i, title, desc))
        story.append(SP(4))

    story.append(H2('7.3  Script Skeleton'))
    story += CB('scripts/tune_ekf_apriltag.py  (skeleton to implement)',
        '#!/usr/bin/env python3\n'
        '"""\n'
        'tune_ekf_apriltag.py — Post-process AprilTag calibration bag.\n'
        'Usage:\n'
        '    python3 scripts/tune_ekf_apriltag.py \\\n'
        '        --bag  calibration/apriltag_calib_XXX \\\n'
        '        --survey calibration/tag_survey.json \\\n'
        '        --output calibration/ekf_tuning_results.yaml\n'
        '"""\n'
        'import argparse, json, yaml, numpy as np, matplotlib.pyplot as plt\n'
        'from pathlib import Path\n'
        'from rosbags.rosbag2 import Reader\n'
        'from rosbags.typesys import get_types_from_msg, register_types\n'
        '\n'
        'def load_survey(path):\n'
        '    """Return dict {tag_id: np.array([x, y, z])}"""\n'
        '    with open(path) as f:\n'
        '        data = json.load(f)\n'
        '    return {t["id"]: np.array([t["x"], t["y"], t["z"]]) for t in data["tags"]}\n'
        '\n'
        'def main():\n'
        '    ap = argparse.ArgumentParser()\n'
        '    ap.add_argument("--bag",    required=True)\n'
        '    ap.add_argument("--survey", required=True)\n'
        '    ap.add_argument("--output", default="calibration/ekf_tuning_results.yaml")\n'
        '    args = ap.parse_args()\n'
        '\n'
        '    survey = load_survey(args.survey)\n'
        '    innovations = []  # list of np.array([dx, dy, dheading])\n'
        '\n'
        '    with Reader(args.bag) as reader:\n'
        '        # TODO: iterate messages, build TF buffer,\n'
        '        #       extract detections, compute innovations\n'
        '        pass\n'
        '\n'
        '    innovations = np.array(innovations)   # shape (N, 3)\n'
        '    R_optimal = np.mean(innovations[:, :, None] * innovations[:, None, :], axis=0)\n'
        '\n'
        '    results = {\n'
        '        "R_diagonal_xy_heading": R_optimal.diagonal().tolist(),\n'
        '        "num_observations": len(innovations),\n'
        '        "mean_error_m": float(np.mean(np.linalg.norm(innovations[:, :2], axis=1))),\n'
        '        "max_error_m":  float(np.max (np.linalg.norm(innovations[:, :2], axis=1))),\n'
        '    }\n'
        '    with open(args.output, "w") as f:\n'
        '        yaml.dump(results, f)\n'
        '    print(f"Results written to {args.output}")\n'
        '    print(f"Mean position error: {results[\'mean_error_m\']*100:.1f} cm")\n'
        '\n'
        'if __name__ == "__main__":\n'
        '    main()'
    )

    # ── SECTION 8 — Validation ────────────────────────────────────────────────
    story.append(PageBreak())
    story.append(H1('8  Validation After Tuning'))
    story.append(HR())

    story.append(H2('8.1  Validation Procedure'))
    story.append(Pb(
        'After applying the tuned EKF parameters, run <b>5 fresh validation laps</b> '
        '(not used in the tuning data) and re-run the post-processing script. '
        'The mean error should decrease by 30–60 % compared to the pre-tuning baseline.'
    ))
    story.append(simple_table(
        ['Metric', 'Target (Pre-Tuning Typical)', 'Target (Post-Tuning)'],
        [
            ['Mean position error per lap',  '10–20 cm', '< 5 cm'],
            ['Max position error per lap',   '30–50 cm', '< 15 cm'],
            ['Heading error at tag',         '2–5°',     '< 1.5°'],
            ['Lap-2 drift vs Lap-1',         '5–15 cm',  '< 5 cm'],
            ['Narrow passage lateral error', '4–8 cm',   '< 4 cm (2" clearance budget)'],
        ],
        [5.5*cm, 5*cm, 5*cm], styles
    ))

    story.append(H2('8.2  If Results Are Worse After Tuning'))
    story += Bl([
        'Check that the tag survey measurements are accurate — re-measure with RTK',
        'Check the tag_survey.json frame_id matches the EKF map frame origin',
        'Ensure apriltag_ros is using the correct tag size (measure your printout physically)',
        'Revert to ekf_local_default.yaml and try tuning with only 5 laps (reduce overfitting)',
        'Check camera intrinsics are calibrated — bad intrinsics = bad tag pose estimates',
    ])

    story.append(H2('8.3  Save Results'))
    story += CB('Archive tuning results with git',
        'git add calibration/tag_survey.json \\\n'
        '         calibration/ekf_tuning_results.yaml \\\n'
        '         src/localization/config/ekf_local.yaml\n'
        'git commit -m "calib: AprilTag EKF tuning session YYYYMMDD"'
    )
    story.append(NOTE(
        'Do NOT commit the rosbag to git — it is too large. '
        'Store it on an external SSD and reference it in calibration/ekf_tuning_results.yaml '
        'under a "source_bag" key.',
        styles
    ))

    # ── SECTION 9 — Quick Reference ───────────────────────────────────────────
    story.append(PageBreak())
    story.append(H1('9  Quick Reference — Day-of Checklist'))
    story.append(HR())

    story.append(simple_table(
        ['#', 'Action', 'Command / Notes'],
        [
            ['1', 'Print and mount 8–10 Tag36h11 tags',
             'Min 20×20 cm, flat board, camera-height'],
            ['2', 'Survey each tag centre with RTK',
             'Record to calibration/tag_survey.json'],
            ['3', 'Source workspace',
             'source /opt/ros/humble/setup.bash && source install/setup.bash'],
            ['4', 'Launch full robot stack',
             'bash scripts/run_robot.sh laptop'],
            ['5', 'Launch apriltag_ros',
             'ros2 run apriltag_ros apriltag_node (see §3.4)'],
            ['6', 'Verify /detections',
             'ros2 topic echo /detections  — walk past a tag to confirm'],
            ['7', 'Start bag recording',
             'ros2 bag record /tf /tf_static /odometry/fast_lio /odometry/filtered /detections /fix'],
            ['8', 'Drive 10+ laps',
             'Mix of normal, slow, fast speed laps'],
            ['9', 'Stop recording, note bag name', ''],
            ['10', 'Run post-processing',
             'python3 scripts/tune_ekf_apriltag.py --bag ... --survey ... (§6.3)'],
            ['11', 'Apply results to ekf_local.yaml', 'See §6.4'],
            ['12', 'Run 5 validation laps + re-process', 'Confirm improvement — see §8.1'],
            ['13', 'Commit results to git', 'See §8.3'],
        ],
        [0.8*cm, 4.2*cm, 10.5*cm], styles
    ))

    # ── BUILD ─────────────────────────────────────────────────────────────────
    doc.build(story, onFirstPage=_on_cover, onLaterPages=_on_page)
    print(f'[OK] PDF written to: {OUT_PATH}')


if __name__ == '__main__':
    build_pdf()
