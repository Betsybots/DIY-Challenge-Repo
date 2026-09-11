#!/usr/bin/env python3
"""
generate_jetson_bringup_guide.py — Jetson Bring-Up Guide
Team Juggernauts · DIY Robot Challenge 2026

Produces docs/Jetson_Bringup_Guide.pdf using ReportLab.
Source content: docs/jetson_bringup_guide.md (kept in sync manually).
Run from repo root:   python3 docs/generate_jetson_bringup_guide.py
"""

import os
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import cm
from reportlab.lib import colors
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.enums import TA_CENTER, TA_JUSTIFY
from reportlab.platypus import (
    SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle,
    PageBreak, HRFlowable, Preformatted,
)

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_PATH = os.path.join(REPO_ROOT, 'docs', 'Jetson_Bringup_Guide.pdf')

# ── Colour palette (matches the rest of this repo's generated guides) ─────────
C_NAVY = colors.HexColor('#1A2744')
C_BLUE = colors.HexColor('#2563EB')
C_TEAL = colors.HexColor('#0F766E')
C_ORANGE = colors.HexColor('#EA580C')
C_GREEN = colors.HexColor('#16A34A')
C_RED = colors.HexColor('#DC2626')
C_LGREY = colors.HexColor('#F1F5F9')
C_MGREY = colors.HexColor('#94A3B8')
C_BLACK = colors.HexColor('#0F172A')
C_WHITE = colors.white
C_AMBER = colors.HexColor('#D97706')
C_PURPLE = colors.HexColor('#7C3AED')


def make_styles():
    base = getSampleStyleSheet()
    s = {}
    s['title'] = ParagraphStyle('DocTitle', parent=base['Title'],
        fontSize=27, leading=33, textColor=C_WHITE, spaceAfter=6, alignment=TA_CENTER)
    s['subtitle'] = ParagraphStyle('DocSubtitle',
        fontSize=12, leading=17, textColor=colors.HexColor('#CBD5E1'),
        spaceAfter=4, alignment=TA_CENTER)
    s['h1'] = ParagraphStyle('H1', fontSize=16, leading=20, textColor=C_NAVY,
        spaceBefore=20, spaceAfter=6, fontName='Helvetica-Bold')
    s['h2'] = ParagraphStyle('H2', fontSize=13, leading=17, textColor=C_BLUE,
        spaceBefore=14, spaceAfter=4, fontName='Helvetica-Bold')
    s['h3'] = ParagraphStyle('H3', fontSize=11, leading=15, textColor=C_TEAL,
        spaceBefore=8, spaceAfter=3, fontName='Helvetica-BoldOblique')
    s['body'] = ParagraphStyle('Body', fontSize=10, leading=14, textColor=C_BLACK,
        spaceAfter=6, alignment=TA_JUSTIFY)
    s['cover_body'] = ParagraphStyle('CoverBody', fontSize=10.5, leading=15,
        textColor=colors.HexColor('#E2E8F0'),
        spaceAfter=6, alignment=TA_CENTER)
    s['bullet'] = ParagraphStyle('Bullet', fontSize=10, leading=13, textColor=C_BLACK,
        spaceAfter=3, leftIndent=14, bulletIndent=4)
    s['step'] = ParagraphStyle('Step', fontSize=10, leading=14, textColor=C_BLACK,
        spaceAfter=5, leftIndent=20, bulletIndent=4)
    s['code'] = ParagraphStyle('Code', fontName='Courier',
        fontSize=8.5, leading=12, textColor=C_BLACK,
        backColor=C_LGREY, borderPad=6, spaceAfter=6)
    s['code_label'] = ParagraphStyle('CodeLabel', fontName='Helvetica-Bold',
        fontSize=8, textColor=C_MGREY, spaceAfter=1)
    s['warn'] = ParagraphStyle('Warn', fontSize=9.5, leading=13,
        textColor=colors.HexColor('#7C2D12'),
        backColor=colors.HexColor('#FEF3C7'),
        borderPad=6, spaceAfter=6, fontName='Helvetica')
    s['danger'] = ParagraphStyle('Danger', fontSize=9.5, leading=13,
        textColor=colors.HexColor('#7F1D1D'),
        backColor=colors.HexColor('#FEE2E2'),
        borderPad=6, spaceAfter=6, fontName='Helvetica-Bold')
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
    s['tc_mono'] = ParagraphStyle('TC_mono', fontName='Courier',
        fontSize=8, textColor=C_BLACK, leading=12)
    return s


# ── Helper builders ─────────────────────────────────────────────────────────────

def H(text, level, s): return Paragraph(text, s[level])
def P(text, s): return Paragraph(text, s['body'])
def B(items, s): return [Paragraph(f'&bull;  {item}', s['bullet']) for item in items]
def SP(n=6): return Spacer(1, n)


def numbered_steps(items, s):
    return [Paragraph(f'<b>{i + 1}.</b>&nbsp;&nbsp;{item}', s['step']) for i, item in enumerate(items)]


def code_block(label, text, s):
    out = []
    if label:
        out.append(Paragraph(label, s['code_label']))
    out.append(Preformatted(text, s['code']))
    return out


def hr(s): return HRFlowable(width='100%', thickness=0.5, color=C_MGREY, spaceAfter=8)
def warn_box(t, s): return Paragraph('<b>WARNING &mdash;</b>&nbsp; ' + t, s['warn'])
def danger_box(t, s): return Paragraph('<b>CRITICAL &mdash;</b>&nbsp; ' + t, s['danger'])
def note_box(t, s): return Paragraph('<b>NOTE &mdash;</b>&nbsp; ' + t, s['note'])
def tip_box(t, s): return Paragraph('<b>VERIFIED &mdash;</b>&nbsp; ' + t, s['tip'])


def simple_table(header, rows, col_widths, s, header_color=None):
    hc = header_color or C_NAVY
    data = [[Paragraph(h, s['th']) for h in header]]
    for row in rows:
        data.append([Paragraph(str(c), s['tc']) for c in row])
    tbl = Table(data, colWidths=col_widths)
    tbl.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, 0), hc),
        ('ROWBACKGROUNDS', (0, 1), (-1, -1), [C_WHITE, C_LGREY]),
        ('GRID', (0, 0), (-1, -1), 0.4, C_MGREY),
        ('VALIGN', (0, 0), (-1, -1), 'TOP'),
        ('TOPPADDING', (0, 0), (-1, -1), 4),
        ('BOTTOMPADDING', (0, 0), (-1, -1), 4),
        ('LEFTPADDING', (0, 0), (-1, -1), 6),
        ('RIGHTPADDING', (0, 0), (-1, -1), 6),
    ]))
    return tbl


def mono_table(header, rows, col_widths, s, header_color=None):
    hc = header_color or C_NAVY
    data = [[Paragraph(h, s['th']) for h in header]]
    for row in rows:
        data.append([Paragraph(str(c), s['tc_mono']) for c in row])
    tbl = Table(data, colWidths=col_widths)
    tbl.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, 0), hc),
        ('ROWBACKGROUNDS', (0, 1), (-1, -1), [C_WHITE, C_LGREY]),
        ('GRID', (0, 0), (-1, -1), 0.4, C_MGREY),
        ('VALIGN', (0, 0), (-1, -1), 'TOP'),
        ('TOPPADDING', (0, 0), (-1, -1), 4),
        ('BOTTOMPADDING', (0, 0), (-1, -1), 4),
        ('LEFTPADDING', (0, 0), (-1, -1), 6),
        ('RIGHTPADDING', (0, 0), (-1, -1), 6),
    ]))
    return tbl


def checklist_table(items, s):
    rows = [[Paragraph('&#9744;', s['tc']), Paragraph(item, s['tc'])] for item in items]
    tbl = Table(rows, colWidths=[0.7 * cm, 14.3 * cm])
    tbl.setStyle(TableStyle([
        ('ROWBACKGROUNDS', (0, 0), (-1, -1), [C_WHITE, C_LGREY]),
        ('GRID', (0, 0), (-1, -1), 0.3, C_MGREY),
        ('TOPPADDING', (0, 0), (-1, -1), 3),
        ('BOTTOMPADDING', (0, 0), (-1, -1), 3),
        ('LEFTPADDING', (0, 0), (-1, -1), 6),
    ]))
    return tbl


# ── Cover page ──────────────────────────────────────────────────────────────────

def cover_page(s):
    story = []
    story.append(SP(60))
    story.append(Paragraph('Team Juggernauts', s['subtitle']))
    story.append(SP(4))
    story.append(Paragraph('Jetson Bring-Up Guide', s['title']))
    story.append(SP(8))
    story.append(Paragraph('Fresh Clone &rarr; Running Localization', s['subtitle']))
    story.append(SP(4))
    story.append(Paragraph(
        'ROS 2 Humble &middot; Jetson Orin Nano &middot; FAST-LIO2 &middot; map_localizer (VGICP)',
        s['subtitle']))
    story.append(SP(40))
    story.append(HRFlowable(width='100%', thickness=1.5, color=C_BLUE, spaceAfter=12))
    story.append(Paragraph(
        'This guide covers: what must already be installed &middot; cloning and building '
        'on a fresh Jetson &middot; launching the localization pipeline (and the full '
        'stack) &middot; exactly what output reaches the controller &middot; how to verify '
        'it is working &middot; known open items to check before trusting results.',
        s['cover_body']))
    story.append(SP(20))
    story.append(Paragraph(
        'Companion references: <b>docs/testing_guide.md</b> (full script/launch-file/'
        'profile reference) and <b>docs/reuse_plan_step1.md</b> (the running decision '
        'log — why things are built the way they are).', s['cover_body']))
    story.append(PageBreak())
    return story


# ── Section 1 : Prerequisites ────────────────────────────────────────────────────

def section_prereqs(s):
    story = []
    story.append(H('1&nbsp;&middot;&nbsp;What Has To Already Be True Before You Start', 'h1', s))
    story.append(hr(s))
    story.append(P(
        'This repo does <b>not</b> install the items below — confirm each one '
        'separately before attempting a launch.', s))

    rows = [
        ['ROS 2 Humble installed', 'Everything depends on it',
         'ls /opt/ros/humble/setup.bash'],
        ['Real Hesai lidar driver<br/>(hesai_ros_driver)',
         'challenge_master.launch.py launches it by package name — it must '
         'already be on the Jetson ROS overlay, not vendored in this repo',
         'ros2 pkg prefix hesai_ros_driver'],
        ['Real ZED SDK + zed_wrapper<br/>(Stereolabs zed-ros2-wrapper)',
         'Same as above — camera driver is not vendored here',
         'ros2 pkg prefix zed_wrapper'],
        ['ACEINNA IMU driver running<br/><b>on the RPi</b> (separately)',
         'FAST-LIO2/EKF need /imu/data; this repo has no IMU package '
         'anymore — it runs independently on the RPi',
         'Confirm /imu/data appears once bridged'],
        ['Zenoh bridge running on<br/><b>both</b> Jetson and RPi',
         'Without it /imu/data and /wheel_odom never reach the Jetson, '
         'and /cmd_vel_nav never reaches the RPi motor driver',
         'Lives entirely outside this repo — restart manually on both '
         'sides after any reboot'],
        ['libyaml-cpp-dev, PCL,<br/>Eigen3 (system libs)',
         'map_localizer / fast_gicp hard-require these at build time',
         'rosdep install (part of setup.sh) pulls these in automatically'],
    ]
    story.append(simple_table(
        ['Requirement', 'Why', 'How to Check'], rows,
        [4.6 * cm, 7.2 * cm, 5.2 * cm], s))

    story.append(SP(10))
    story.append(H('Hardware Location', 'h2', s))
    story.append(P(
        '<b>On the Jetson:</b> Hesai QT64 lidar (UDP, default '
        '192.168.1.201:2368 &mdash; see profiles/jetson.env), ZED2i camera (USB3).', s))
    story.append(P(
        '<b>NOT on the Jetson:</b> the IMU and the motor/CAN bus &mdash; both live '
        'on the Raspberry Pi.', s))
    story.append(PageBreak())
    return story


# ── Section 2 : Clone and build ──────────────────────────────────────────────────

def section_clone_build(s):
    story = []
    story.append(H('2&nbsp;&middot;&nbsp;Clone and Build', 'h1', s))
    story.append(hr(s))

    story.append(H('2.1&nbsp;&nbsp;Clone at the Recommended Path', 'h2', s))
    story.append(note_box(
        'profiles/jetson.env&rsquo;s DIY_ROS_WS defaults to ${HOME}/ros2_ws, and '
        'diy_localization&rsquo;s map_pcd_path launch argument defaults to '
        '$DIY_ROS_WS/src/DIY-Challenge-Repo/maps/refined_map.pcd. Clone elsewhere '
        'and you must pass map_pcd_path explicitly every time you launch.', s))
    story += code_block('Clone with submodules, then build', '''\
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone --recurse-submodules \\
    https://github.com/Betsybots/DIY-Challenge-Repo.git
cd DIY-Challenge-Repo

# Builds third_party_ws (fast_gicp, ndt_omp_ros2, lidar_imu_calib,
# LIO-SAM, imu_utils_ros2_humble) + first-party DIY packages, in order.
bash setup.sh jetson''', s)

    story.append(H('2.2&nbsp;&nbsp;If setup.sh Fails Partway Through', 'h2', s))
    story.append(P(
        'A few known, unrelated-to-this-repo sandbox/system gaps may or may not '
        'apply to your specific Jetson image:', s))
    story += B([
        '<b>code_utils / imu_utils</b> failing on <font face="Courier">elfutils/'
        'libdw.h</font> &mdash; missing <font face="Courier">libdw-dev</font>; only '
        'affects IMU Allan-variance calibration tooling, not the main pipeline.',
        '<b>lio_sam</b> failing on missing <b>GTSAM</b> &mdash; only affects offline '
        'map generation (offline_mapping.launch.py), not runtime localization.',
    ], s)
    story.append(tip_box(
        'Both are safe to ignore if you are not using those specific tools right now.', s))

    story.append(H('2.3&nbsp;&nbsp;Sourcing Order Matters', 'h2', s))
    story += code_block('Manual sourcing (order is important)', '''\
source /opt/ros/humble/setup.bash
source third_party_ws/install/setup.bash   # MUST come before the next line
source install/setup.bash''', s)
    story.append(warn_box(
        'diy_ndt_localization (and anything linking third_party_ws packages like '
        'ndt_omp_ros2) will fail to even be found by ros2 launch / colcon build if '
        'third_party_ws/install/setup.bash is not sourced first.', s))
    story += code_block('Preferred: use the env script (does this in the right order)', '''\
source scripts/env.sh jetson''', s)

    story.append(PageBreak())
    return story


# ── Section 3 : Launching ────────────────────────────────────────────────────────

def section_launching(s):
    story = []
    story.append(H('3&nbsp;&middot;&nbsp;Launching', 'h1', s))
    story.append(hr(s))

    story.append(H('Option A &mdash; The Full Stack (Normal Operation)', 'h2', s))
    story += code_block('Run on the Jetson', 'scripts/run_robot.sh jetson', s)
    story.append(P(
        'Runs challenge_master.launch.py with every DIY_USE_* flag from '
        'profiles/jetson.env mapped to a launch argument &mdash; lidar driver, camera, '
        'FAST-LIO2, the single EKF, map_localizer, Nav2, zone_nav.', s))
    story.append(warn_box(
        'Must be run alongside  scripts/run_robot.sh raspi  on the RPi (owns the '
        'motor driver, cmd_vel_mux, and the IMU) with the Zenoh bridge up on both '
        'sides. Never run the same profile on both devices &mdash; see '
        'testing_guide.md &sect;2 / caveat #5.', s))

    story.append(H('Option B &mdash; Localization Only (Recommended First Test)', 'h2', s))
    story.append(P(
        'Isolates FAST-LIO2 + EKF + map_localizer without Nav2 / zone_nav / motor '
        'control &mdash; much easier to debug on a first run.', s))
    story += code_block('Run directly', '''\
ros2 launch diy_localization localization.launch.py \\
    mode:=runtime \\
    use_rviz:=true \\
    map_pcd_path:=/absolute/path/to/your/refined_map.pcd''', s)
    story.append(note_box(
        'map_pcd_path only needs to be passed explicitly if you did not clone at '
        '~/ros2_ws/src/DIY-Challenge-Repo, or want to use a different map than '
        'maps/refined_map.pcd.', s))
    story.append(SP(4))
    story.append(note_box(
        'This is a separate, unrelated map from the one the custom A*/PD '
        'controller stack uses. map_pcd_path here is a 3D point cloud (.pcd) '
        'for map_localizer&rsquo;s map&rarr;odom TF. The controller&rsquo;s map is a 2D '
        'occupancy grid (.pgm/.yaml) configured via map_yaml/DIY_MAP_YAML '
        '&mdash; see custom_nav_stack_design.md &sect;5. A .pgm/.yaml pair cannot be '
        'used as a map_pcd_path value or vice versa.', s))

    story.append(H('What This Starts, In Order', 'h2', s))
    story += numbered_steps([
        '<b>fastlio_mapping</b> (FAST-LIO2) &mdash; needs /lidar_points and '
        '/imu/data immediately.',
        '<b>ekf_filter_node_odom</b> (the single EKF) &mdash; needs /wheel_odom '
        '(from the RPi, over Zenoh), /imu/data, and FAST-LIO2&rsquo;s gated output.',
        '<b>map_localizer_node</b> &mdash; starts immediately but does nothing until '
        'step 4.',
        '<b>trigger_map_relocalize.py</b> &mdash; waits for map_localizer_node&rsquo;s '
        '/relocalize service, calls it once with map_pcd_path + the initial pose '
        '(defaults to map origin), then <b>exits</b> (by design &mdash; not a '
        'long-running node).',
    ], s)
    story.append(tip_box(
        'Watch for the log line:  /relocalize succeeded: relocalize success  &mdash; '
        'this confirms the map loaded. If instead you see it hang on &ldquo;Waiting '
        'up to 30s for /relocalize service&rdquo;, map_localizer_node isn&rsquo;t up '
        'yet or crashed &mdash; check its own log for the real error.', s))

    story.append(PageBreak())
    return story


# ── Section 4 : What reaches the controller ──────────────────────────────────────

def section_controller_output(s):
    story = []
    story.append(H('4&nbsp;&middot;&nbsp;What Actually Reaches the Controller', 'h1', s))
    story.append(hr(s))
    story.append(P(
        'Regardless of whether the controller is Nav2 or a custom one &mdash; there is '
        '<b>no controller-specific wiring</b> on the localization side. Two things are '
        'published, and any controller consumes them the standard ROS 2 way.', s))

    story.append(H('1. TF Tree:  map &rarr; odom &rarr; base_link', 'h2', s))
    story += B([
        '<b>map &rarr; odom</b>: broadcast by map_localizer_node (only after step 4 '
        'in &sect;3 succeeds &mdash; before that, this transform does not exist at all).',
        '<b>odom &rarr; base_link</b>: broadcast by the EKF (ekf_filter_node_odom).',
    ], s)
    story.append(danger_box(
        'There is NO /pose or /ndt_pose-style topic for global position. '
        'map_localizer only publishes TF + a debug /map_cloud + its two services. '
        'A controller that needs the robot&rsquo;s pose in the map frame MUST do a '
        'standard TF lookup.', s))

    story += code_block('C++ (rclcpp) — same pattern Nav2 itself uses internally', '''\
geometry_msgs::msg::TransformStamped map_to_base =
    tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);''', s)
    story += code_block('Python (rclpy)', '''\
from tf2_ros import Buffer, TransformListener
tf_buffer = Buffer()
TransformListener(tf_buffer, node)
map_to_base = tf_buffer.lookup_transform(
    "map", "base_link", rclpy.time.Time())''', s)

    story.append(H('2. /odometry/filtered  (nav_msgs/Odometry, odom frame, 50 Hz)', 'h2', s))
    story.append(P(
        'Published by the EKF. Most controllers subscribe to this directly for '
        'local velocity/pose feedback (smooth, continuous, no TF-tree traversal '
        'needed) rather than doing a TF lookup on every control-loop tick.', s))

    story.append(H('Which One Should the Controller Use?', 'h2', s))
    rows = [
        ['Local smooth velocity feedback (closed-loop control)', '/odometry/filtered topic'],
        ['Global position for waypoint / path following against the map', 'map &rarr; base_link TF lookup'],
    ]
    story.append(simple_table(['Need', 'Use This'], rows, [10 * cm, 7 * cm], s))

    story.append(PageBreak())
    return story


# ── Section 5 : Verifying it works ───────────────────────────────────────────────

def section_verify(s):
    story = []
    story.append(H('5&nbsp;&middot;&nbsp;Verifying It Is Actually Working', 'h1', s))
    story.append(hr(s))

    story += code_block('Topic rates you would expect', '''\
ros2 topic hz /lidar_points          # real Hesai QT64 lidar topic
ros2 topic hz /imu/data              # from the RPi, over Zenoh
ros2 topic hz /lidar_odometry        # FAST-LIO2 output
ros2 topic hz /odometry/filtered     # EKF output, should be ~50 Hz''', s)

    story += code_block('Confirm the map actually loaded', '''\
ros2 service call /relocalize_check slam_interfaces/srv/IsValid \\
    "{code: 1}"''', s)

    story += code_block('Confirm the full TF chain resolves', '''\
ros2 run tf2_ros tf2_echo map base_link''', s)

    story += code_block('Full pre-flight checklist (nodes, rates, TF, e-stop, Nav2)', '''\
scripts/health_check.sh jetson''', s)

    story.append(warn_box(
        'If map &rarr; base_link never resolves: check  ros2 node list  for '
        'map_localizer_node and ekf_filter_node_odom both present, then check '
        'ros2 topic list  for /lidar_odometry and /cloud_registered_body actually '
        'publishing (both required before map_localizer&rsquo;s synced callback ever '
        'fires &mdash; see &sect;3).', s))

    story.append(PageBreak())
    return story


# ── Section 6 : Known open items ─────────────────────────────────────────────────

def section_open_items(s):
    story = []
    story.append(H('6&nbsp;&middot;&nbsp;Known Open Items &mdash; Check Before Trusting Results', 'h1', s))
    story.append(hr(s))
    story.append(P(
        'These affect the localization pipeline specifically and are not yet '
        'resolved as of this guide &mdash; see reuse_plan_step1.md for full detail on '
        'each.', s))

    story.append(note_box(
        '<b>RESOLVED (2026-09-11):</b> hesai_ros_driver&rsquo;s real output '
        'topic is confirmed on hardware to be <font face="Courier">/lidar_points'
        '</font>. All code/config in this repo now uses that name consistently '
        '(challenge_master.launch.py previously assumed /hesai/points &mdash; '
        'that assumption has been corrected; see reuse_plan_step1.md Step 25).', s))
    story.append(SP(4))
    story.append(warn_box(
        '<b>extrinsic_R was just fixed to a valid rotation (identity) but not yet '
        're-verified on hardware</b> &mdash; the new value does not match this '
        'repo&rsquo;s own from-scratch Rx(-90&deg;) derivation from an earlier '
        'accelerometer test. Re-verify with a live accelerometer reading if results '
        'look physically wrong (e.g. yaw/roll swapped).', s))
    story.append(SP(4))
    story.append(warn_box(
        '<b>Whether maps/refined_map.pcd is genuinely this course&rsquo;s map</b> '
        '(vs. a bench/test map) is unconfirmed. If localization looks completely '
        'wrong from the start, this is the first thing to check.', s))
    story.append(SP(4))
    story.append(warn_box(
        '<b>VGICP alignment accuracy has only been verified with synthetic data '
        'in a sandbox</b> &mdash; never against real lidar scans. Watch '
        'map_localizer&rsquo;s own log output and /map_cloud in RViz for obviously '
        'bad alignment.', s))
    story.append(SP(4))
    story.append(warn_box(
        '<b>base_link&rarr;camera_link has NO publisher at all right now.</b> '
        'The URDF (robot.urdf.xacro) does not define camera_link (only '
        'lidar_link/imu_link are real &mdash; verified by actually running xacro '
        'and checking the generated output), and zed_wrapper&rsquo;s own publish_tf '
        'was deliberately disabled based on the false assumption that the URDF '
        'already covers it. Anything projecting ZED2i image/depth data into the '
        'robot frame via TF will fail until this is fixed.', s))
    story.append(SP(4))
    story.append(note_box(
        '<b>This robot has no GPS hardware at all</b> (confirmed) &mdash; '
        'DIY_USE_GPS is false on every profile; no /gps/fix topic will ever '
        'appear, by design, not because anything is broken.', s))

    story.append(PageBreak())
    return story


# ── Section 7 : Quick reference ──────────────────────────────────────────────────

def section_quick_ref(s):
    story = []
    story.append(H('7&nbsp;&middot;&nbsp;Quick Reference &mdash; Commands Used Most Often', 'h1', s))
    story.append(hr(s))

    story += code_block('One-time setup', '''\
git clone --recurse-submodules <repo-url> \\
    ~/ros2_ws/src/DIY-Challenge-Repo
cd ~/ros2_ws/src/DIY-Challenge-Repo
bash setup.sh jetson''', s)

    story += code_block('Every new terminal', 'source scripts/env.sh jetson', s)

    story += code_block('Test localization in isolation', '''\
ros2 launch diy_localization localization.launch.py \\
    mode:=runtime use_rviz:=true''', s)

    story += code_block('Full stack (after localization checks out)', 'scripts/run_robot.sh jetson', s)

    story += code_block('Pre-flight check', 'scripts/health_check.sh jetson', s)

    story.append(SP(20))
    story.append(hr(s))
    story.append(P(
        '<i>This guide is a focused walkthrough. For the full script / launch-file / '
        'profile reference, see docs/testing_guide.md. For why things are built the '
        'way they are, see docs/reuse_plan_step1.md.</i>', s))
    return story


# ── Page decoration (fixes the invisible-white-title-on-white-background bug
#    present in this repo's other generate_*.py scripts — cover_bg was defined
#    but never actually painted onto the canvas there) ─────────────────────────

PAGE_W, PAGE_H = A4


def draw_cover_background(canvas, doc):
    canvas.saveState()
    canvas.setFillColor(C_NAVY)
    canvas.rect(0, 0, PAGE_W, PAGE_H, fill=1, stroke=0)
    # A thin accent band near the top for a bit of colour variation
    canvas.setFillColor(C_BLUE)
    canvas.rect(0, PAGE_H - 0.4 * cm, PAGE_W, 0.4 * cm, fill=1, stroke=0)
    canvas.restoreState()


def draw_content_page(canvas, doc):
    canvas.saveState()
    # Thin colour band at the very top of every content page
    canvas.setFillColor(C_NAVY)
    canvas.rect(0, PAGE_H - 0.3 * cm, PAGE_W, 0.3 * cm, fill=1, stroke=0)
    # Footer: doc title (left) + page number (right)
    canvas.setFont('Helvetica', 8)
    canvas.setFillColor(C_MGREY)
    canvas.drawString(2 * cm, 1.3 * cm, 'Jetson Bring-Up Guide \u2014 Team Juggernauts 2026')
    canvas.drawRightString(PAGE_W - 2 * cm, 1.3 * cm, f'Page {doc.page - 1}')
    canvas.setStrokeColor(C_MGREY)
    canvas.setLineWidth(0.4)
    canvas.line(2 * cm, 1.7 * cm, PAGE_W - 2 * cm, 1.7 * cm)
    canvas.restoreState()


# ── Main ──────────────────────────────────────────────────────────────────────

def build_pdf():
    doc = SimpleDocTemplate(
        OUT_PATH,
        pagesize=A4,
        leftMargin=2 * cm, rightMargin=2 * cm,
        topMargin=2.2 * cm, bottomMargin=2.2 * cm,
        title='Jetson Bring-Up Guide — Team Juggernauts 2026',
        author='Team Juggernauts',
        subject='DIY Robot Challenge 2026 — Fresh Clone to Running Localization',
    )

    s = make_styles()
    story = []
    story += cover_page(s)
    story += section_prereqs(s)
    story += section_clone_build(s)
    story += section_launching(s)
    story += section_controller_output(s)
    story += section_verify(s)
    story += section_open_items(s)
    story += section_quick_ref(s)

    doc.build(story, onFirstPage=draw_cover_background, onLaterPages=draw_content_page)
    size_kb = os.path.getsize(OUT_PATH) // 1024
    print(f"Generated: {OUT_PATH}  ({size_kb} KB)")


if __name__ == '__main__':
    build_pdf()
