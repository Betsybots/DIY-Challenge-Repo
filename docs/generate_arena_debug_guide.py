#!/usr/bin/env python3
"""
generate_arena_debug_guide.py — Arena Runs & Debug Guide
Team Juggernauts · DIY Robot Challenge 2026

Produces docs/Arena_Debug_Guide.pdf using ReportLab.
Run from repo root:   python3 docs/generate_arena_debug_guide.py
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

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_PATH  = os.path.join(REPO_ROOT, 'docs', 'Arena_Debug_Guide.pdf')

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
C_PURPLE = colors.HexColor('#7C3AED')


def make_styles():
    base = getSampleStyleSheet()
    s = {}
    s['title'] = ParagraphStyle('DocTitle', parent=base['Title'],
        fontSize=28, leading=34, textColor=C_WHITE, spaceAfter=6, alignment=TA_CENTER)
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
    s['tc_sm'] = ParagraphStyle('TC_sm', fontName='Helvetica',
        fontSize=8, textColor=C_BLACK, leading=11)
    return s


# ── Helper builders ────────────────────────────────────────────────────────────

def H(text, level, s): return Paragraph(text, s[level])
def P(text, s):        return Paragraph(text, s['body'])
def B(items, s):       return [Paragraph(f'• {item}', s['bullet']) for item in items]
def SP(n=6):           return Spacer(1, n)

def numbered_steps(items, s):
    return [Paragraph(f'<b>{i+1}.</b>  {item}', s['step']) for i, item in enumerate(items)]

def code_block(label, text, s):
    out = []
    if label:
        out.append(Paragraph(label, s['code_label']))
    out.append(Preformatted(text, s['code']))
    return out

def hr(s): return HRFlowable(width='100%', thickness=0.5, color=C_MGREY, spaceAfter=8)
def warn_box(t, s):   return Paragraph('⚠  WARNING:  ' + t, s['warn'])
def danger_box(t, s): return Paragraph('🔴  CRITICAL:  ' + t, s['danger'])
def note_box(t, s):   return Paragraph('ℹ  NOTE:  ' + t, s['note'])
def tip_box(t, s):    return Paragraph('✅  TIP:  ' + t, s['tip'])

def simple_table(header, rows, col_widths, s, header_color=None):
    hc = header_color or C_NAVY
    data = [[Paragraph(h, s['th']) for h in header]]
    for row in rows:
        data.append([Paragraph(str(c), s['tc']) for c in row])
    tbl = Table(data, colWidths=col_widths)
    tbl.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), hc),
        ('ROWBACKGROUNDS', (0,1), (-1,-1), [C_WHITE, C_LGREY]),
        ('GRID', (0,0), (-1,-1), 0.4, C_MGREY),
        ('VALIGN', (0,0), (-1,-1), 'TOP'),
        ('TOPPADDING', (0,0), (-1,-1), 4),
        ('BOTTOMPADDING', (0,0), (-1,-1), 4),
        ('LEFTPADDING', (0,0), (-1,-1), 6),
        ('RIGHTPADDING', (0,0), (-1,-1), 6),
    ]))
    return tbl

def mono_table(header, rows, col_widths, s, header_color=None):
    """Table where data cells use monospace font."""
    hc = header_color or C_NAVY
    data = [[Paragraph(h, s['th']) for h in header]]
    for row in rows:
        data.append([Paragraph(str(c), s['tc_mono']) for c in row])
    tbl = Table(data, colWidths=col_widths)
    tbl.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), hc),
        ('ROWBACKGROUNDS', (0,1), (-1,-1), [C_WHITE, C_LGREY]),
        ('GRID', (0,0), (-1,-1), 0.4, C_MGREY),
        ('VALIGN', (0,0), (-1,-1), 'TOP'),
        ('TOPPADDING', (0,0), (-1,-1), 4),
        ('BOTTOMPADDING', (0,0), (-1,-1), 4),
        ('LEFTPADDING', (0,0), (-1,-1), 6),
        ('RIGHTPADDING', (0,0), (-1,-1), 6),
    ]))
    return tbl

def checklist_table(items, s):
    rows = [[Paragraph('☐', s['tc']), Paragraph(item, s['tc'])] for item in items]
    tbl = Table(rows, colWidths=[0.5*cm, 14.5*cm])
    tbl.setStyle(TableStyle([
        ('ROWBACKGROUNDS', (0,0), (-1,-1), [C_WHITE, C_LGREY]),
        ('GRID', (0,0), (-1,-1), 0.3, C_MGREY),
        ('TOPPADDING', (0,0), (-1,-1), 3),
        ('BOTTOMPADDING', (0,0), (-1,-1), 3),
        ('LEFTPADDING', (0,0), (-1,-1), 6),
    ]))
    return tbl


# ── Cover page ────────────────────────────────────────────────────────────────

def cover_page(s):
    story = []
    cover_bg = colors.HexColor('#1A2744')
    story.append(SP(60))
    story.append(Paragraph('Team Juggernauts', s['subtitle']))
    story.append(SP(4))
    story.append(Paragraph('Arena Runs &amp; Debug Guide', s['title']))
    story.append(SP(8))
    story.append(Paragraph('DIY Robot Challenge 2026', s['subtitle']))
    story.append(SP(4))
    story.append(Paragraph('ROS 2 Humble · Jetson Orin Nano · Zone Nav Stack', s['subtitle']))
    story.append(SP(40))
    story.append(HRFlowable(width='100%', thickness=1.5, color=C_BLUE, spaceAfter=12))
    story.append(Paragraph(
        'This guide covers: workspace build · launching the competition stack · '
        'pre-run health checks · live monitoring · bag recording · CSV logging · '
        'bag replay · debug tools · field troubleshooting decision tree.',
        s['body']))
    story.append(PageBreak())
    return story


# ── Section 1 : Workspace Build ───────────────────────────────────────────────

def section_build(s):
    story = []
    story.append(H('1 · Workspace Build (Local Dev Machine)', 'h1', s))
    story.append(hr(s))
    story.append(P(
        'The repo has two workspaces: the main workspace in the repo root (zone nav, '
        'bringup, mux) and a third-party workspace for SLAM libraries. '
        'phoenix6 (CTRE motor vendor) is robot-only — skip it for local builds.', s))

    story.append(H('1.1  One-time Setup', 'h2', s))
    story += code_block('Source ROS 2 Humble', '''\
source /opt/ros/humble/setup.bash
# Add to ~/.bashrc to avoid repeating every session:
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc''', s)

    story.append(H('1.2  Build Zone Nav Stack (recommended for debugging)', 'h2', s))
    story += code_block('scripts/env.sh must be sourced first', '''\
cd /path/to/DIY-Challenge-Repo
source /opt/ros/humble/setup.bash

colcon build --symlink-install \\
  --packages-select \\
    diy_zone_nav \\
    diy_cmd_vel_mux \\
    diy_estop_controller \\
    challenge_bringup \\
    diy_robot_description

source install/setup.bash''', s)
    story.append(note_box(
        'diy_motor_control_legacy requires phoenix6 (CTRE vendor lib — robot only). '
        'Always skip it on dev machines.', s))

    story.append(H('1.3  Build Third-Party SLAM Stack (if needed for replay)', 'h2', s))
    story += code_block('Build separately in third_party_ws/', '''\
cd /path/to/DIY-Challenge-Repo/third_party_ws
colcon build --symlink-install
source install/setup.bash''', s)

    story.append(H('1.4  Full On-Robot Build (Jetson)', 'h2', s))
    story += code_block('On Jetson Orin — builds everything including motor driver', '''\
cd /path/to/DIY-Challenge-Repo
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash''', s)

    story.append(warn_box(
        'Always run  source install/setup.bash  after every build. '
        'Missing this step is the most common cause of "package not found" errors.', s))

    story.append(PageBreak())
    return story


# ── Section 2 : Environment & Profiles ───────────────────────────────────────

def section_env(s):
    story = []
    story.append(H('2 · Environment & Profiles', 'h1', s))
    story.append(hr(s))
    story.append(P(
        'scripts/env.sh sets all DIY_* environment variables from the selected profile '
        'before launching. Profiles live in profiles/ and define which hardware and '
        'algorithms are enabled for that machine.', s))

    story += code_block('Source environment for a profile', '''\
source scripts/env.sh jetson       # full hardware stack (Jetson Orin)
source scripts/env.sh laptop       # software-only / replay mode
source scripts/env.sh sim          # Gazebo simulation''', s)

    story.append(H('Key DIY_* Variables', 'h2', s))
    rows = [
        ['DIY_ROBOT_PROFILE',    'Active profile name (jetson/laptop/sim)'],
        ['DIY_USE_NAV2',         'true/false — enable Nav2 navigation stack'],
        ['DIY_USE_ZONE_NAV',     'true/false — enable zone nav state machine'],
        ['DIY_USE_LOCALIZATION', 'true/false — enable FAST-LIO2 + NDT pipeline'],
        ['DIY_USE_HESAI',        'true/false — enable Hesai QT64 lidar driver'],
        ['DIY_USE_MOTOR_DRIVER', 'true/false — enable CTRE motor driver'],
        ['DIY_MUX_MODE',         'Initial mux mode: JOYSTICK/AUTONOMOUS'],
        ['DIY_FASTLIO_CONFIG',   'FAST-LIO2 config YAML filename'],
    ]
    story.append(simple_table(
        ['Variable', 'Description'], rows,
        [5.5*cm, 10.5*cm], s))
    story.append(SP(8))
    story.append(PageBreak())
    return story


# ── Section 3 : Launching the Competition Stack ───────────────────────────────

def section_launch(s):
    story = []
    story.append(H('3 · Launching the Competition Stack', 'h1', s))
    story.append(hr(s))

    story.append(H('3.1  Full Competition Launch (run_robot.sh)', 'h2', s))
    story.append(P(
        'This is the primary launch script for arena runs. It sources the environment, '
        'then calls challenge_master.launch.py with all arguments resolved from the profile.', s))
    story += code_block('scripts/run_robot.sh', '''\
# On Jetson — full hardware stack
./scripts/run_robot.sh jetson

# On laptop — software-only (requires bag file set in profile)
./scripts/run_robot.sh laptop''', s)

    story.append(H('3.2  Direct Launch via challenge_master.launch.py', 'h2', s))
    story.append(P(
        'Use this when you need to override individual arguments '
        'without changing the profile file.', s))
    story += code_block('ros2 launch — manual argument override', '''\
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch challenge_bringup challenge_master.launch.py \\
    use_joystick:=false \\
    use_nav2:=true \\
    use_zone_nav:=true \\
    use_localization:=true \\
    use_hesai:=true \\
    use_motor_driver:=true \\
    use_realsense:=false \\
    use_micro_ros:=true \\
    mux_mode:=AUTONOMOUS \\
    fastlio_config:=fast_lio_hesai_qt64.yaml''', s)

    story.append(H('3.3  Zone Nav Only (without full bringup)', 'h2', s))
    story += code_block('Launch zone nav subsystem in isolation', '''\
ros2 launch diy_zone_nav zone_nav.launch.py \\
    use_localization:=true \\
    launch_gate:=true''', s)
    story.append(note_box(
        'When launched from challenge_master, zone_nav.launch.py receives '
        'launch_gate:=false to avoid duplicating the lidar_odom_gate_node '
        'which the master already starts.', s))

    story.append(H('3.4  Launch Arguments Reference', 'h2', s))
    rows = [
        ['use_joystick',     'true/false', 'Enable joystick driver node'],
        ['use_nav2',         'true/false', 'Enable Nav2 navigation stack'],
        ['use_zone_nav',     'true/false', 'Enable zone nav state machine'],
        ['use_localization', 'true/false', 'Enable FAST-LIO2 + NDT + lidar gate'],
        ['use_hesai',        'true/false', 'Enable Hesai QT64 lidar driver'],
        ['use_motor_driver', 'true/false', 'Enable CTRE TalonFX motor driver'],
        ['use_realsense',    'true/false', 'Enable RealSense depth camera'],
        ['use_micro_ros',    'true/false', 'Enable micro-ROS (IMU, wheel enc.)'],
        ['mux_mode',         'string',     'Initial mux mode (JOYSTICK / AUTONOMOUS)'],
        ['fastlio_config',   'string',     'FAST-LIO2 config YAML filename'],
        ['launch_gate',      'true/false', 'Launch lidar_odom_gate_node here (default true)'],
        ['use_rviz',         'true/false', 'Launch RViz2 for visualization'],
    ]
    story.append(simple_table(
        ['Argument', 'Type', 'Description'], rows,
        [4.5*cm, 2*cm, 9.5*cm], s))

    story.append(PageBreak())
    return story


# ── Section 4 : Pre-Run Checklist ────────────────────────────────────────────

def section_prerun(s):
    story = []
    story.append(H('4 · Pre-Run Checklist (Before Every Arena Run)', 'h1', s))
    story.append(hr(s))

    story.append(H('4.1  Automated Health Check', 'h2', s))
    story.append(P(
        'Run health_check_zone_nav.sh after launching the stack. '
        'All 11 checks must PASS before the robot moves.', s))
    story += code_block('scripts/health_check_zone_nav.sh', '''\
bash scripts/health_check_zone_nav.sh

# Expected output:
#   [PASS] /nav_mode is publishing
#   [PASS] /mux_mode is publishing
#   [PASS] /odometry/filtered is publishing
#   [PASS] /ndt_fitness_score is publishing
#   [PASS] /cmd_vel_safe is publishing
#   [PASS] Service /global_costmap/global_costmap/set_parameters is available
#   [PASS] Service /local_costmap/local_costmap/set_parameters is available
#   [PASS] Service /cmd_vel_mux_node/set_parameters is available
#   [PASS] /nav_mode value readable
#   [PASS] /mux_mode value readable
#   [PASS] /estop_active is False (safe to run)
#   Passed: 11  Failed: 0''', s)
    story.append(danger_box(
        'Do NOT start a run if any check FAILS. '
        'A silent topic means a crashed node — find and fix it first.', s))

    story.append(H('4.2  Manual Pre-Run Checklist', 'h2', s))
    items = [
        'E-stop button is released (LED off)',
        'Joystick is connected and responsive',
        'Mux mode is set to JOYSTICK for manual verification, then switch to AUTONOMOUS',
        'Robot is placed at start line, pointing toward first waypoint',
        'zone_waypoints.yaml coordinates match the actual arena layout',
        'ndt_fitness_score < 0.5 (good localization before start)',
        'Recording tools are started (record_zone_nav.sh + zone_nav_logger.py)',
        'Battery is fully charged',
        'All team members clear of the robot path',
    ]
    story.append(checklist_table(items, s))
    story.append(PageBreak())
    return story


# ── Section 5 : Live Monitoring Tools ────────────────────────────────────────

def section_monitoring(s):
    story = []
    story.append(H('5 · Live Monitoring Tools', 'h1', s))
    story.append(hr(s))

    story.append(H('5.1  Zone Nav Dashboard (inspect_zone_nav.sh)', 'h2', s))
    story.append(P(
        'Displays a live terminal dashboard refreshing every 1 second. '
        'Shows nav_mode, mux_mode, estop, NDT fitness, odom position, and cmd_vel.', s))
    story += code_block('scripts/inspect_zone_nav.sh', '''\
bash scripts/inspect_zone_nav.sh

# Run in a separate terminal from the robot launch.
# Press Ctrl+C to exit.''', s)

    story.append(H('5.2  rqt Tools', 'h2', s))
    rows = [
        ['rqt_graph',   'rqt_graph',   'Node/topic connection graph — spot disconnected nodes'],
        ['rqt_console', 'rqt_console', 'Filtered log viewer — filter by node name or severity'],
        ['rqt_plot',    'rqt_plot',    'Real-time topic plotting (basic — use PlotJuggler for serious analysis)'],
        ['rqt_tf_tree', 'rqt &gt; Plugins &gt; TF Tree', 'Verify TF frames are publishing'],
    ]
    story.append(simple_table(
        ['Tool', 'Command', 'Use'], rows,
        [3*cm, 4.5*cm, 8.5*cm], s))

    story.append(SP(6))
    story.append(H('5.3  RViz2 Key Displays', 'h2', s))
    rows = [
        ['Map',                '/map',                          'Occupancy grid (pre-built map for NDT)'],
        ['Path',               '/plan',                         'Nav2 planned path to goal'],
        ['Pose',               '/odometry/filtered',            'Robot position estimate (EKF output)'],
        ['Costmap (global)',   '/global_costmap/costmap',       'Global cost layer'],
        ['Costmap (local)',    '/local_costmap/costmap',        'Local obstacle layer (VoxelLayer)'],
        ['PointCloud2',        '/hesai/points',                 'Raw lidar scan (heavy — disable if slow)'],
        ['TF',                 '—',                             'Visualize all active TF frames'],
    ]
    story.append(simple_table(
        ['Display Type', 'Topic', 'Purpose'], rows,
        [3.5*cm, 5.5*cm, 7*cm], s))

    story += code_block('Launch RViz2 standalone', '''\
rviz2 -d src/diy_robot_description/rviz/default.rviz''', s)

    story.append(H('5.4  Quick Topic Checks (One-liners)', 'h2', s))
    story += code_block('Useful ros2 topic commands during a run', '''\
# Check nav state machine mode
ros2 topic echo --once /nav_mode

# Check velocity arbitration mode
ros2 topic echo --once /mux_mode

# Check NDT localization quality (lower = better; >0.8 = degraded)
ros2 topic echo --once /ndt_fitness_score

# Check what velocity is being commanded to motors
ros2 topic echo --once /cmd_vel_safe

# Check active speed limit
ros2 topic echo --once /speed_limit

# Monitor topic publish rates (4-second window)
ros2 topic hz /nav_mode --window 4
ros2 topic hz /odometry/filtered --window 4
ros2 topic hz /ndt_fitness_score --window 4''', s)

    story.append(H('5.5  PlotJuggler (install once)', 'h2', s))
    story += code_block('Install and launch', '''\
sudo apt install ros-humble-plotjuggler-ros
plotjuggler''', s)
    story.append(P(
        'In PlotJuggler: File → Start ROS2 Topic Subscriber, then drag topics '
        'onto the plot canvas. Drag /ndt_fitness_score, /cmd_vel_safe/linear/x, '
        'and /nav_mode onto the same time axis to correlate mode changes with '
        'speed commands during a run.', s))

    story.append(PageBreak())
    return story


# ── Section 6 : Recording Tools ──────────────────────────────────────────────

def section_recording(s):
    story = []
    story.append(H('6 · Recording Tools', 'h1', s))
    story.append(hr(s))
    story.append(P(
        'Always record during arena runs. If something goes wrong, '
        'the recording lets you replay and debug the exact sequence of events '
        'without needing the robot to be present.', s))

    story.append(H('6.1  Bag Recorder — record_zone_nav.sh', 'h2', s))
    story.append(P(
        'Records 20 zone nav state topics (no lidar point clouds, no camera). '
        'A full run fits in ~5 MB vs. GBs for a full bag. '
        'Output: ~/bags/zone_nav_&lt;label&gt;_&lt;timestamp&gt;/', s))
    story += code_block('scripts/record_zone_nav.sh', '''\
# Basic usage — label defaults to "run"
bash scripts/record_zone_nav.sh

# With a descriptive label
bash scripts/record_zone_nav.sh --label arena_run_01

# Run in background while monitoring in another terminal
bash scripts/record_zone_nav.sh --label arena_run_01 &

# Stop recording
Ctrl+C  (or kill the background PID)

# Output location
ls ~/bags/zone_nav_arena_run_01_*/''', s)

    story.append(H('Recorded Topics', 'h3', s))
    rows = [
        ['/nav_mode',              'Zone nav state machine mode (INIT/NORMAL_NAV/SLOW_NAV/BLIND_DRIVE/...)'],
        ['/green_light',           'Race start trigger'],
        ['/mux_mode',              'Velocity arbitration mode (JOYSTICK/AUTONOMOUS/BLIND_DRIVE/ESTOP_LOCK)'],
        ['/cmd_vel_zone_nav',      'Zone nav velocity command'],
        ['/cmd_vel_nav',           'Nav2 velocity command'],
        ['/cmd_vel_joy',           'Joystick velocity command'],
        ['/cmd_vel_safe',          'Final velocity to motors (mux output)'],
        ['/speed_limit',           'Active speed cap (nav2_msgs/SpeedLimit)'],
        ['/estop_active',          'Hardware e-stop state'],
        ['/ndt_fitness_score',     'NDT match quality (Float32, lower=better)'],
        ['/odometry/filtered',     'EKF fused position estimate'],
        ['/lidar_odometry',        'FAST-LIO2 raw odometry'],
        ['/lidar_odometry_gated',  'Lidar odom gated off during BLIND_DRIVE'],
        ['/tf + /tf_static',       'Transform frames for rviz2 replay'],
        ['/diagnostics',           'Node diagnostic messages'],
        ['/rosout',                'All ROS log output'],
    ]
    story.append(simple_table(
        ['Topic', 'Content'], rows,
        [5.5*cm, 10.5*cm], s))

    story.append(H('6.2  CSV Flight Data Logger — zone_nav_logger.py', 'h2', s))
    story.append(P(
        'A standalone Python node that writes a timestamped 13-column CSV file '
        'for post-run analysis. Writes immediately on any state transition '
        '(nav_mode/mux_mode/estop changes); throttles high-frequency topics to 5 Hz. '
        'Output: ~/bags/zone_nav_log_&lt;timestamp&gt;.csv', s))
    story += code_block('scripts/zone_nav_logger.py', '''\
# Run in a separate terminal alongside the stack
python3 scripts/zone_nav_logger.py

# Output location
ls ~/bags/zone_nav_log_*.csv''', s)

    story.append(H('CSV Column Reference', 'h3', s))
    rows = [
        ['wall_time_s',       'Unix wall-clock timestamp (seconds)'],
        ['ros_time_s',        'ROS clock timestamp (seconds)'],
        ['nav_mode',          'Current zone nav state machine mode string'],
        ['mux_mode',          'Current velocity mux mode string'],
        ['estop_active',      'True/False — hardware e-stop state'],
        ['ndt_fitness',       'NDT match fitness score (Float32, lower=better)'],
        ['odom_x',            'EKF position X in map frame (meters)'],
        ['odom_y',            'EKF position Y in map frame (meters)'],
        ['odom_yaw_deg',      'EKF heading angle (degrees)'],
        ['speed_limit_ms',    'Active speed cap (m/s, 0.0 = no limit)'],
        ['cmd_vel_linear_x',  'Final commanded forward velocity (m/s)'],
        ['cmd_vel_angular_z', 'Final commanded angular velocity (rad/s)'],
        ['event',             'Optional label — state transition trigger description'],
    ]
    story.append(simple_table(
        ['Column', 'Description'], rows,
        [4.5*cm, 11.5*cm], s))

    story.append(H('Analyse in Python / pandas', 'h3', s))
    story += code_block('Post-run analysis snippet', '''\
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("~/bags/zone_nav_log_20260801_143022.csv")

# Plot speed vs time, coloured by nav_mode
fig, ax = plt.subplots(figsize=(14, 4))
for mode, grp in df.groupby("nav_mode"):
    ax.plot(grp["wall_time_s"], grp["cmd_vel_linear_x"], label=mode)
ax.set_xlabel("Time (s)")
ax.set_ylabel("Linear velocity (m/s)")
ax.legend()
plt.tight_layout()
plt.show()

# Find all BLIND_DRIVE events
blind = df[df["nav_mode"] == "BLIND_DRIVE"]
print(f"BLIND_DRIVE entries: {len(blind)}")
print(blind[["wall_time_s", "ndt_fitness", "odom_x", "odom_y"]].head(20))''', s)

    story.append(H('Open in PlotJuggler', 'h3', s))
    story += code_block('Load CSV in PlotJuggler', '''\
plotjuggler
# File → Load Data File → select the .csv
# Drag columns onto plot canvas
# Suggested layout:
#   Row 1: cmd_vel_linear_x, speed_limit_ms
#   Row 2: ndt_fitness
#   Row 3: odom_x, odom_y  (as XY scatter for path trace)''', s)

    story.append(PageBreak())
    return story


# ── Section 7 : Bag Replay ───────────────────────────────────────────────────

def section_replay(s):
    story = []
    story.append(H('7 · Bag Replay (Offline Debugging)', 'h1', s))
    story.append(hr(s))
    story.append(P(
        'Bag replay lets you re-run a recorded session through the full software '
        'stack without the robot. Use this to reproduce bugs, tune parameters, '
        'or verify fixes before the next arena session.', s))

    story.append(H('7.1  replay_bag.sh', 'h2', s))
    story += code_block('scripts/replay_bag.sh', '''\
# Full replay at real speed
bash scripts/replay_bag.sh ~/bags/zone_nav_arena_run_01_20260801_143022 laptop

# Slow-motion replay (0.5x speed — easier to follow state transitions)
bash scripts/replay_bag.sh ~/bags/zone_nav_arena_run_01_* laptop --rate 0.5

# Replay without Nav2 (just localization + zone nav — faster startup)
bash scripts/replay_bag.sh ~/bags/zone_nav_arena_run_01_* laptop --no-nav2''', s)

    story.append(H('7.2  Manual Bag Replay', 'h2', s))
    story += code_block('Play a bag directly (stack must be running separately)', '''\
# In terminal 1 — start the stack (no hardware drivers)
ros2 launch challenge_bringup challenge_master.launch.py \\
    use_hesai:=false use_motor_driver:=false \\
    use_micro_ros:=false use_rviz:=true

# In terminal 2 — play the bag
ros2 bag play ~/bags/zone_nav_arena_run_01_*/ --clock

# Slow it down
ros2 bag play ~/bags/zone_nav_arena_run_01_*/ --clock --rate 0.3

# Loop it
ros2 bag play ~/bags/zone_nav_arena_run_01_*/ --clock --loop''', s)

    story.append(H('7.3  Inspect Bag Contents', 'h2', s))
    story += code_block('Check what is in a bag before replaying', '''\
ros2 bag info ~/bags/zone_nav_arena_run_01_*/

# List topics and message counts
# Look for: duration, message count per topic, missing topics''', s)

    story.append(warn_box(
        'Always use --clock when replaying bags. Without it, nodes use wall-clock '
        'time and time-based logic (watchdogs, timeouts) will behave incorrectly.', s))
    story.append(PageBreak())
    return story


# ── Section 8 : Injection & Control Tools ────────────────────────────────────

def section_injection(s):
    story = []
    story.append(H('8 · Injection & Control Tools', 'h1', s))
    story.append(hr(s))
    story.append(P(
        'These tools let you manually trigger events and change modes at runtime '
        'without physical hardware — essential for bench testing and verifying '
        'state transitions before arena runs.', s))

    story.append(H('8.1  Set Mux Mode — set_mux_mode.sh', 'h2', s))
    story += code_block('scripts/set_mux_mode.sh', '''\
# Switch to autonomous (Nav2 controls robot)
bash scripts/set_mux_mode.sh AUTONOMOUS

# Switch to joystick control
bash scripts/set_mux_mode.sh JOYSTICK

# Switch to blind drive mode (lidar gated, fixed velocity)
bash scripts/set_mux_mode.sh BLIND_DRIVE

# Show current mode (no argument)
bash scripts/set_mux_mode.sh

# NOTE: ESTOP_LOCK cannot be set manually — hardware e-stop button only''', s)

    story.append(H('8.2  Simulate Green Light — trigger_green_light.sh', 'h2', s))
    story.append(P(
        'Publishes a single Bool:true message on /green_light, causing the zone '
        'nav state machine to transition from INIT → NORMAL_NAV. '
        'Use this for bench testing without the physical start gate signal.', s))
    story += code_block('scripts/trigger_green_light.sh', '''\
bash scripts/trigger_green_light.sh

# Expected result:
# /nav_mode transitions from INIT → NORMAL_NAV
# /mux_mode transitions from JOYSTICK → AUTONOMOUS
# Robot begins following the Nav2 plan''', s)

    story.append(H('8.3  Manual Topic Publishing', 'h2', s))
    story += code_block('Useful one-liners for testing', '''\
# Manually publish green light
ros2 topic pub --once /green_light std_msgs/msg/Bool "data: true"

# Manually set a speed limit (m/s)
ros2 topic pub --once /speed_limit nav2_msgs/msg/SpeedLimit \\
    "{header: {frame_id: map}, speed_limit: 0.5, percentage: false}"

# Manually trigger e-stop (simulated)
ros2 topic pub --once /estop_active std_msgs/msg/Bool "data: true"

# Release simulated e-stop
ros2 topic pub --once /estop_active std_msgs/msg/Bool "data: false"''', s)

    story.append(PageBreak())
    return story


# ── Section 9 : Test Scripts (Step-by-step Validation) ───────────────────────

def section_test_scripts(s):
    story = []
    story.append(H('9 · Step-by-Step Validation Scripts', 'h1', s))
    story.append(hr(s))
    story.append(P(
        'Use these scripts to validate each layer of the stack in isolation '
        'before attempting a full arena run. Run them in order — each builds '
        'on the previous layer.', s))

    rows = [
        ['test_step1_wheel_odom.sh',  'Step 1',
         'Validate wheel odometry from micro-ROS encoders.\nChecks /wheel_odom topic is publishing at ~20 Hz.'],
        ['test_step2_fused_odom.sh',  'Step 2',
         'Validate EKF fused odometry.\nChecks /odometry/filtered is running and covariance is reasonable.'],
        ['test_step3_fastlio.sh',     'Step 3',
         'Validate FAST-LIO2 lidar-inertial odometry.\nChecks /lidar_odometry and confirms point cloud is arriving.'],
        ['test_step4_motion_plan.sh', 'Step 4',
         'Validate Nav2 motion planning.\nSends a test goal and checks that a plan is computed and the robot starts moving.'],
    ]
    for script, step, desc in rows:
        story.append(H(f'{step}: {script}', 'h3', s))
        story.append(P(desc.replace('\n', ' — '), s))
        story += code_block('', f'bash scripts/{script}', s)

    story.append(tip_box(
        'Run each test script in a separate terminal from the running stack. '
        'The stack must already be launched before running any test script.', s))
    story.append(PageBreak())
    return story


# ── Section 10 : Field Troubleshooting Decision Tree ─────────────────────────

def section_troubleshoot(s):
    story = []
    story.append(H('10 · Field Troubleshooting Decision Tree', 'h1', s))
    story.append(hr(s))
    story.append(P(
        'Use this decision tree when something goes wrong during an arena run. '
        'Start at the top and work down.', s))

    # --- 10.1 Robot not moving
    story.append(H('10.1  Robot Not Moving at All', 'h2', s))
    steps = [
        '<b>Check /estop_active</b> — if True, release the hardware e-stop button',
        '<b>Check /mux_mode</b> — if ESTOP_LOCK, release e-stop; if JOYSTICK, switch to AUTONOMOUS',
        '<b>Check /nav_mode</b> — if INIT, green light was not received; run trigger_green_light.sh',
        '<b>Check /cmd_vel_safe</b> — if all zeros but /cmd_vel_nav has values, mux is blocking',
        '<b>Check motor driver node</b> — run: ros2 node list | grep motor',
        '<b>Run health_check_zone_nav.sh</b> — any FAIL indicates a crashed node',
    ]
    story += numbered_steps(steps, s)

    # --- 10.2 Robot moving wrong direction / wrong speed
    story.append(H('10.2  Robot Moving Wrong Direction or Wrong Speed', 'h2', s))
    steps = [
        '<b>Check /speed_limit</b> — a very low speed_limit will cap velocity unexpectedly',
        '<b>Check /nav_mode</b> — SLOW_NAV mode halves speed; check if robot entered wrong zone',
        '<b>Check zone waypoints</b> — zone_waypoints.yaml coordinates may be wrong for this arena',
        '<b>Check /odometry/filtered</b> — drifted position makes Nav2 plan wrong paths',
        '<b>Check /ndt_fitness_score</b> — if > 0.8, localization is degraded; BLIND_DRIVE may activate',
    ]
    story += numbered_steps(steps, s)

    # --- 10.3 Localization / NDT degraded
    story.append(H('10.3  NDT Localization Degraded (fitness > 0.5)', 'h2', s))
    steps = [
        '<b>Check lidar is publishing</b> — ros2 topic hz /hesai/points --window 4',
        '<b>Check /lidar_odometry</b> — FAST-LIO2 may have crashed; check node list',
        '<b>Check /lidar_odometry_gated</b> — if stuck in BLIND_DRIVE, gate is closed; check /nav_mode',
        '<b>Check map file</b> — map must match the actual arena; wrong map = bad fitness everywhere',
        '<b>Check lidar mounting</b> — any physical shift in the lidar breaks the extrinsic calibration',
    ]
    story += numbered_steps(steps, s)

    # --- 10.4 Zone nav stuck in wrong mode
    story.append(H('10.4  Zone Nav Stuck in Wrong Mode', 'h2', s))
    steps = [
        '<b>Check /nav_mode topic</b> — confirm the current mode string',
        '<b>BLIND_DRIVE stuck</b>: fitness must drop below blind_drive_exit_fitness_threshold (0.8) to exit; check NDT',
        '<b>SLOW_NAV stuck</b>: robot must leave the slow zone radius; check zone_waypoints.yaml coordinates',
        '<b>INIT stuck</b>: check /green_light topic — may never have received true; run trigger_green_light.sh',
        '<b>Check zone_nav_manager_node logs</b> — ros2 topic echo /rosout | grep zone_nav',
    ]
    story += numbered_steps(steps, s)

    # --- 10.5 Bag replay not working
    story.append(H('10.5  Bag Replay Not Working', 'h2', s))
    steps = [
        'Ensure the stack is running with use_hesai:=false use_motor_driver:=false',
        'Always include --clock in ros2 bag play command',
        'Check bag info — ros2 bag info &lt;bag_path&gt; — confirm expected topics exist',
        'Check TF frame timestamps — if TF is stale the robot will not appear in RViz2',
        'Reduce --rate to 0.3 if system cannot process messages fast enough',
    ]
    story += numbered_steps(steps, s)

    story.append(H('10.6  Quick Diagnosis Commands', 'h2', s))
    story += code_block('Copy-paste these at the arena when something looks wrong', '''\
# Which nodes are running?
ros2 node list

# Which topics are publishing?
ros2 topic list

# What is zone nav currently doing?
ros2 topic echo --once /nav_mode && \\
ros2 topic echo --once /mux_mode && \\
ros2 topic echo --once /estop_active && \\
ros2 topic echo --once /ndt_fitness_score

# Are all expected topics publishing?
bash scripts/health_check_zone_nav.sh

# Check for error/warning logs
ros2 topic echo /rosout | grep -E "WARN|ERROR|FATAL"

# Zone nav node logs (last 50 lines)
ros2 topic echo /rosout | grep zone_nav_manager''', s)

    story.append(PageBreak())
    return story


# ── Section 11 : Full Terminal Layout for Arena Runs ─────────────────────────

def section_terminal_layout(s):
    story = []
    story.append(H('11 · Recommended Terminal Layout for Arena Runs', 'h1', s))
    story.append(hr(s))
    story.append(P(
        'Use a terminal multiplexer (tmux or byobu) to run all tools in one view. '
        'Suggested 4-pane layout:', s))

    story += code_block('tmux 4-pane setup', '''\
# Create new session
tmux new-session -s arena

# Split into 4 panes:
# Ctrl+B, "  (horizontal split)
# Ctrl+B, %  (vertical split)

# Pane 1 (top-left): Main stack launch
./scripts/run_robot.sh jetson

# Pane 2 (top-right): Live dashboard
bash scripts/inspect_zone_nav.sh

# Pane 3 (bottom-left): Bag + CSV recorder
bash scripts/record_zone_nav.sh --label arena_run_01
# (new sub-pane below)
python3 scripts/zone_nav_logger.py

# Pane 4 (bottom-right): Health check + ad-hoc commands
bash scripts/health_check_zone_nav.sh
# then use for ros2 topic echo / set_mux_mode / trigger_green_light''', s)

    story.append(tip_box(
        'Start recorders BEFORE the stack reaches INIT mode. '
        'Starting them late means you miss the green light and first zone transition events.', s))

    story.append(H('Recommended Tool Order', 'h2', s))
    rows = [
        ['1', 'Build',        'colcon build --packages-select ...', 'One-time or after code change'],
        ['2', 'Launch',       './scripts/run_robot.sh jetson',       'Start full competition stack'],
        ['3', 'Record bag',   'bash scripts/record_zone_nav.sh --label run01', 'Immediate after launch'],
        ['4', 'CSV log',      'python3 scripts/zone_nav_logger.py',  'Immediate after launch'],
        ['5', 'Health check', 'bash scripts/health_check_zone_nav.sh', 'All PASS before moving'],
        ['6', 'Dashboard',    'bash scripts/inspect_zone_nav.sh',     'Keep open during entire run'],
        ['7', 'Green light',  'bash scripts/trigger_green_light.sh', 'Start the run'],
        ['8', 'Post-run',     'plotjuggler / pandas CSV',             'Analyse recording after run'],
    ]
    story.append(simple_table(
        ['#', 'Step', 'Command', 'When'], rows,
        [0.8*cm, 2.5*cm, 6.5*cm, 6*cm], s))

    story.append(PageBreak())
    return story


# ── Section 12 : Key Topics & Parameters Quick Reference ─────────────────────

def section_reference(s):
    story = []
    story.append(H('12 · Key Topics & Parameters Quick Reference', 'h1', s))
    story.append(hr(s))

    story.append(H('12.1  Key Topics', 'h2', s))
    rows = [
        ['/nav_mode',             'std_msgs/String',        'Zone nav state (INIT/NORMAL_NAV/SLOW_NAV/BLIND_DRIVE/DONE)'],
        ['/mux_mode',             'std_msgs/String',        'Mux mode (JOYSTICK/AUTONOMOUS/BLIND_DRIVE/ESTOP_LOCK)'],
        ['/green_light',          'std_msgs/Bool',          'Race start trigger'],
        ['/estop_active',         'std_msgs/Bool',          'Hardware e-stop state'],
        ['/ndt_fitness_score',    'std_msgs/Float32',       'NDT match quality (lower=better; >0.8=degraded)'],
        ['/speed_limit',          'nav2_msgs/SpeedLimit',   'Active speed cap (percentage=false, 0.0=no limit)'],
        ['/cmd_vel_zone_nav',     'geometry_msgs/Twist',    'Zone nav velocity output'],
        ['/cmd_vel_nav',          'geometry_msgs/Twist',    'Nav2 MPPI velocity output'],
        ['/cmd_vel_joy',          'geometry_msgs/Twist',    'Joystick velocity'],
        ['/cmd_vel_safe',         'geometry_msgs/Twist',    'Final velocity to motors (mux output)'],
        ['/odometry/filtered',    'nav_msgs/Odometry',      'EKF2 fused position estimate'],
        ['/lidar_odometry',       'nav_msgs/Odometry',      'FAST-LIO2 raw odometry'],
        ['/lidar_odometry_gated', 'nav_msgs/Odometry',      'Lidar odom (off during BLIND_DRIVE)'],
    ]
    story.append(simple_table(
        ['Topic', 'Type', 'Description'], rows,
        [4.5*cm, 4*cm, 7.5*cm], s))

    story.append(H('12.2  Key Parameters (Runtime Tunable)', 'h2', s))
    rows = [
        ['/zone_nav_manager_node', 'ndt_fitness_gate',                  '0.5',  'Warn if fitness exceeds this'],
        ['/zone_nav_manager_node', 'blind_drive_exit_fitness_threshold', '0.8',  'Exit BLIND_DRIVE when fitness drops below this'],
        ['/zone_nav_manager_node', 'zone_entry_radius_m',               '3.0',  'Distance to trigger zone entry (m)'],
        ['/zone_nav_manager_node', 'slow_nav_speed_limit_ms',           '0.5',  'Speed cap in SLOW_NAV zones (m/s)'],
        ['/cmd_vel_mux_node',      'mode',                              'AUTO', 'Override mux mode at runtime'],
        ['/global_costmap/global_costmap', 'inflation_layer.inflation_radius', '0.35', 'Global costmap inflation (m)'],
        ['/local_costmap/local_costmap',   'voxel_layer.enabled',              'true', 'Enable/disable obstacle avoidance'],
    ]
    story.append(simple_table(
        ['Node', 'Parameter', 'Default', 'Notes'], rows,
        [5*cm, 5.5*cm, 1.8*cm, 3.7*cm], s))

    story += code_block('Set a parameter at runtime', '''\
ros2 param set /zone_nav_manager_node ndt_fitness_gate 0.6
ros2 param set /zone_nav_manager_node slow_nav_speed_limit_ms 0.3
ros2 param set /cmd_vel_mux_node mode AUTONOMOUS''', s)

    story.append(PageBreak())
    return story


# ── Main ──────────────────────────────────────────────────────────────────────

def build_pdf():
    doc = SimpleDocTemplate(
        OUT_PATH,
        pagesize=A4,
        leftMargin=2*cm, rightMargin=2*cm,
        topMargin=2.2*cm, bottomMargin=2.2*cm,
        title='Arena Runs & Debug Guide — Team Juggernauts 2026',
        author='Team Juggernauts',
        subject='DIY Robot Challenge 2026 — ROS 2 Zone Nav Debug Guide',
    )

    s = make_styles()
    story = []
    story += cover_page(s)
    story += section_build(s)
    story += section_env(s)
    story += section_launch(s)
    story += section_prerun(s)
    story += section_monitoring(s)
    story += section_recording(s)
    story += section_replay(s)
    story += section_injection(s)
    story += section_test_scripts(s)
    story += section_troubleshoot(s)
    story += section_terminal_layout(s)
    story += section_reference(s)

    doc.build(story)
    size_kb = os.path.getsize(OUT_PATH) // 1024
    print(f"✅  Generated: {OUT_PATH}  ({size_kb} KB)")


if __name__ == '__main__':
    build_pdf()
