#!/usr/bin/env python3
"""
generate_competition_playbook.py — Competition Day Playbook
Produces docs/Competition_Day_Playbook.pdf using ReportLab.
Run from the repo root:   python3 docs/generate_competition_playbook.py
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
OUT_PATH  = os.path.join(REPO_ROOT, 'docs', 'Competition_Day_Playbook.pdf')

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
    s['step'] = ParagraphStyle('Step', fontSize=10, leading=14, textColor=C_BLACK,
        spaceAfter=4, leftIndent=18, bulletIndent=4, fontName='Helvetica')
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
    s['tc_sm'] = ParagraphStyle('TC_sm', fontName='Helvetica',
        fontSize=8, textColor=C_BLACK, leading=11)
    s['check_label'] = ParagraphStyle('CheckLabel', fontName='Helvetica-Bold',
        fontSize=11, textColor=C_NAVY, spaceBefore=10, spaceAfter=4)
    return s


def H(text, level, styles): return Paragraph(text, styles[level])
def P(text, styles): return Paragraph(text, styles['body'])
def B(items, styles): return [Paragraph(f'• {item}', styles['bullet']) for item in items]

def numbered_steps(items, styles):
    return [Paragraph(f'<b>{i+1}.</b>  {item}', styles['step']) for i, item in enumerate(items)]

def code_block(label, text, styles):
    out = []
    if label:
        out.append(Paragraph(label, styles['code_label']))
    out.append(Preformatted(text, styles['code']))
    return out

def hr(styles): return HRFlowable(width='100%', thickness=0.5, color=C_MGREY, spaceAfter=8)
def warn_box(t, styles): return Paragraph('WARNING:  ' + t, styles['warn'])
def danger_box(t, styles): return Paragraph('CRITICAL:  ' + t, styles['danger'])
def note_box(t, styles): return Paragraph('NOTE:  ' + t, styles['note'])
def tip_box(t, styles): return Paragraph('TIP:  ' + t, styles['tip'])
def SP(n=6): return Spacer(1, n)


def checklist_table(items, styles):
    """Render a two-column checkbox checklist table."""
    rows = []
    for item in items:
        rows.append([
            Paragraph('☐', styles['tc']),
            Paragraph(item, styles['tc']),
        ])
    t = Table(rows, colWidths=[0.7*cm, 15.8*cm])
    t.setStyle(TableStyle([
        ('FONTSIZE', (0,0), (-1,-1), 10),
        ('VALIGN',   (0,0), (-1,-1), 'TOP'),
        ('TOPPADDING',    (0,0), (-1,-1), 3),
        ('BOTTOMPADDING', (0,0), (-1,-1), 3),
        ('ROWBACKGROUNDS', (0,0), (-1,-1), [C_WHITE, C_LGREY]),
        ('GRID', (0,0), (-1,-1), 0.3, C_MGREY),
    ]))
    return t


def _on_cover(canvas, doc):
    w, h = A4
    canvas.setFillColor(C_NAVY)
    canvas.rect(0, h - 7*cm, w, 7*cm, fill=1, stroke=0)
    canvas.setFillColor(C_ORANGE)
    canvas.rect(0, h - 7.35*cm, w, 0.35*cm, fill=1, stroke=0)

def _on_page(canvas, doc):
    w, h = A4
    canvas.setFillColor(C_LGREY)
    canvas.rect(0, 0, w, 1.4*cm, fill=1, stroke=0)
    canvas.setFont('Helvetica', 8)
    canvas.setFillColor(C_MGREY)
    canvas.drawString(2*cm, 0.55*cm, 'DIY Challenge — Competition Day Playbook')
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
    def NS(items): return numbered_steps(items, styles)
    def CB(lbl, txt): return code_block(lbl, txt, styles)
    def HR(): return hr(styles)
    def WARN(t): return warn_box(t, styles)
    def CRIT(t): return danger_box(t, styles)
    def NOTE(t): return note_box(t, styles)
    def TIP(t): return tip_box(t, styles)
    def CL(items): return checklist_table(items, styles)

    story = []

    # ── Cover ──────────────────────────────────────────────────────────────────
    story.append(SP(60))
    story.append(Paragraph('DIY Challenge Robot', styles['title']))
    story.append(SP(4))
    story.append(Paragraph('Competition Day Playbook', styles['title']))
    story.append(SP(10))
    story.append(Paragraph(
        'Power-On Sequence • Pre-Flight Checks • Start-Line Procedure<br/>'
        'Live Monitoring • Emergency Procedures • Post-Run Recovery',
        styles['subtitle']))
    story.append(SP(6))
    story.append(Paragraph('ROS 2 Humble • Jetson JetPack 5.x', styles['subtitle']))
    story.append(PageBreak())

    # ── TOC ────────────────────────────────────────────────────────────────────
    toc_data = [
        [Paragraph('Section', styles['th']), Paragraph('Topic', styles['th'])],
        ['1', 'The Night Before — Pre-Competition Checklist'],
        ['2', 'Power-On & Boot Sequence'],
        ['3', 'Environment Setup & nvpmodel'],
        ['4', 'Localization Initialization'],
        ['5', 'Start-Line Procedure'],
        ['6', 'During-Run Monitoring'],
        ['7', 'Post-Run: Bag Recording & Data Off-Load'],
        ['8', 'Common Failure Responses'],
        ['9', 'Emergency Stop Procedure'],
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

    # ── §1 Night Before ────────────────────────────────────────────────────────
    story.append(H1('1  The Night Before — Pre-Competition Checklist'))
    story.append(Pb(
        'Complete these tasks the evening before competition day. '
        'Do not leave them to the morning — any failure discovered overnight '
        'gives time to recover.'
    ))

    story.append(H2('1.1  Software &amp; Map Bundle Deploy'))
    story += CB('Deploy the calibration-and-map bundle to the Jetson:', '''\
# From your laptop
bash scripts/deploy_bundle.sh

# What it does:
#  1. rsync src/challenge_bringup/maps/         → jetson:~/robot_ws/maps/
#  2. rsync src/diy_localization/config/*.yaml  → jetson:~/robot_ws/config/
#  3. rsync scripts/                            → jetson:~/robot_ws/scripts/
#  Skips .pcd files >100 MB unless --maps flag passed
''')
    story.append(SP(4))

    story.append(H2('1.2  Night-Before Checklist'))
    story += [SP(4), CL([
        'deploy_bundle.sh completed without errors',
        'static_map.yaml points to correct .pcd files on Jetson',
        'Calibration YAMLs have non-placeholder values (no "CALIB:" entries with 0.0)',
        'USB drive mounted and writable at /mnt/usb — test: touch /mnt/usb/.test',
        'Battery fully charged — check cell voltages, not just SOC leds',
        'All ethernet/USB cables connector firm — wiggle test each',
        'Hesai IP reachable from Jetson: ping 192.168.1.201',
        'STM32 serial port shows up: ls /dev/ttyACM*',
        'GPS antenna clear sky view from mounting position (test fix in open area)',
        'Run health_check.sh — all PASS',
    ]), SP(8)]
    story.append(PageBreak())

    # ── §2 Power-On ────────────────────────────────────────────────────────────
    story.append(H1('2  Power-On &amp; Boot Sequence'))
    story.append(CRIT(
        'Power components in order. Wrong order can brownout the Jetson or '
        'damage the lidar power supply.'
    ))
    story.append(SP(4))
    story.append(H2('2.1  Power-On Order'))
    story += NS([
        '<b>Main battery</b> — connect main LiPo. Check voltage indicator: must be > 22 V (6S).',
        '<b>12 V regulation board</b> — flip the 12 V rail switch. Hesai QT64 will begin spinning '
        '(you may hear the motor start).',
        '<b>Jetson power switch</b> — hold the power button 1 s. Wait for the green LED steady.',
        '<b>STM32/micro-ROS serial</b> — should auto-enumerate on /dev/ttyACM0 once Jetson boots.',
        '<b>GPS antenna</b> — if external power needed, connect now. Expect 30–120 s to first fix.',
        '<b>RealSense D435i</b> — powered via USB3 from Jetson; auto-enumerates on boot.',
    ])
    story.append(SP(4))

    story.append(H2('2.2  Boot Time Expectations'))
    boot_data = [
        [Paragraph('Component', styles['th']), Paragraph('Expected Time', styles['th']),
         Paragraph('Verification', styles['th'])],
        ['Jetson OS', '~30 s', 'Green status LED; ssh jetson@<IP> responds'],
        ['Hesai QT64 spinning', '~10 s after 12V power', 'Audible rotation hum'],
        ['GPS first fix (cold start)', '60–120 s', 'ros2 topic echo /gps/fix: status.status = 0'],
        ['GPS RTK fix', '2–5 min after first fix', 'status.status = 2 (GBAS fix)'],
        ['micro-ROS agent', '5 s after Jetson boot', '/dev/ttyACM0 exists; node appears in ros2 node list'],
    ]
    boot_table = Table(boot_data, colWidths=[4.5*cm, 3.5*cm, 8.5*cm])
    boot_table.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), C_NAVY),
        ('ROWBACKGROUNDS', (0,1), (-1,-1), [C_WHITE, C_LGREY]),
        ('FONTSIZE', (0,0), (-1,-1), 8.5),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('GRID', (0,0), (-1,-1), 0.4, C_MGREY),
        ('TOPPADDING', (0,0), (-1,-1), 4),
        ('BOTTOMPADDING', (0,0), (-1,-1), 4),
    ]))
    story += [SP(4), boot_table, PageBreak()]

    # ── §3 Environment & nvpmodel ──────────────────────────────────────────────
    story.append(H1('3  Environment Setup &amp; nvpmodel'))
    story.append(H2('3.1  Set Jetson Power Mode'))
    story += CB('Set maximum performance mode before sourcing ROS:', '''\
# SSH into the Jetson
ssh jetson@<JETSON_IP>

# Set MAXN mode (all CPU/GPU cores, max frequency)
sudo nvpmodel -m 0
sudo jetson_clocks

# Verify
sudo nvpmodel -q
# Should print: NV Power Mode: MAXN
''')
    story.append(WARN(
        'Run nvpmodel BEFORE starting any ROS nodes. Changing power mode while '
        'nodes are running may cause temporary CPU frequency drops that destabilize '
        'FAST-LIO2 timing.'
    ))
    story.append(SP(4))

    story.append(H2('3.2  Source the Jetson Profile'))
    story += CB('In every new terminal on the Jetson:', '''\
source ~/robot_ws/scripts/env.sh jetson

# Key variables set by jetson.env:
# DIY_HESAI_IP=192.168.1.201
# DIY_MICRO_ROS_SERIAL=/dev/ttyACM0
# DIY_BAG_PATH=/mnt/usb/bags
# DIY_USE_NAV2=true
# DIY_USE_JOYSTICK=false
# DIY_MUX_MODE=AUTONOMOUS
''')
    story.append(H2('3.3  Run Health Check'))
    story += CB('Verify all hardware before proceeding:', '''\
bash ~/robot_ws/scripts/health_check.sh
# Expected: all PASS
# FAIL on hesai/points → check Ethernet cable + IP; ping 192.168.1.201
# FAIL on imu/data     → check /dev/ttyACM0; restart micro_ros_agent
# FAIL on TF tree      → robot_description node may not be running
''')
    story.append(PageBreak())

    # ── §4 Localization Init ──────────────────────────────────────────────────
    story.append(H1('4  Localization Initialization'))
    story.append(Pb(
        'Start the localization stack and allow FAST-LIO2 to lock to the prior map '
        'before entering the start box. At least 60 seconds of static observation '
        'is recommended.'
    ))

    story.append(H2('4.1  Start Full Stack (Challenge Master)'))
    story += CB('Launch everything with challenge_master.launch.py:', '''\
source ~/robot_ws/scripts/env.sh jetson
ros2 launch challenge_bringup challenge_master.launch.py \\
  use_nav2:=true \\
  use_joystick:=false \\
  mux_mode:=AUTONOMOUS
''')
    story += CB('Or start localization only for a pre-run check:', '''\
ros2 launch diy_localization localization.launch.py \\
  mode:=runtime use_gps:=true
''')
    story.append(SP(4))

    story.append(H2('4.2  Localization Lock Checklist'))
    story += [SP(4), CL([
        'FAST-LIO2 prints "IMU Initializing" then "LiDAR point cloud registration" — wait for stability',
        '/odometry/filtered publishing at 50 Hz — check: ros2 topic hz /odometry/filtered',
        'Covariance in /odometry/filtered diagonal entries < 0.05 (stable lock)',
        'TF tree complete: ros2 run tf2_ros tf2_echo map base_link returns pose (not error)',
        'map→odom TF is being published (EKF2 active)',
        'GPS fix acquired (status ≥ 0): ros2 topic echo /gps/fix --once',
        '/odometry/gps and /odometry/filtered agree within 1 m (pre-start sanity)',
    ]), SP(8)]

    story.append(WARN(
        'Do not drive to the start line until localization is locked (covariance stable). '
        'Starting with diverged pose estimate causes Nav2 to plan from wrong position.'
    ))
    story.append(PageBreak())

    # ── §5 Start-Line Procedure ───────────────────────────────────────────────
    story.append(H1('5  Start-Line Procedure'))
    story.append(H2('5.1  Drive to Start Line (Manual Mode)'))
    story += CB('Switch to joystick/teleop mode to position at the start line:', '''\
# Enable joystick from a second terminal
ros2 param set /cmd_vel_mux_node mode JOYSTICK

# Or restart challenge_master with joystick enabled:
ros2 launch challenge_bringup challenge_master.launch.py \\
  use_nav2:=true \\
  use_joystick:=true \\
  mux_mode:=JOYSTICK
''')
    story.append(NOTE(
        'The cmd_vel_mux arbitrates between JOYSTICK and AUTONOMOUS modes. '
        'In JOYSTICK mode, joystick commands pass through even with Nav2 running. '
        'Estop signal from STM32 overrides both modes.'
    ))
    story.append(SP(4))

    story.append(H2('5.2  Start-Line Lock-In Checklist'))
    story += [SP(4), CL([
        'Robot physically positioned at start line, facing the course direction',
        'Localization pose in RViz matches physical robot position (< 0.3 m error)',
        'No obstacles within 1 m of robot (collision monitor will abort navigation otherwise)',
        'Nav2 goal server online: ros2 action list | grep NavigateToPose',
        'Confirm mux_mode = AUTONOMOUS before sending first goal',
        'Record bag started (see §7)',
        'Announce "ready" to competition official',
    ]), SP(8)]

    story.append(H2('5.3  Send the First Navigation Goal'))
    story += CB('Send goal via ros2 action CLI or a pre-defined task script:', '''\
# Example: send goal to GPS waypoint (lat/lon via navigate_through_poses)
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \\
  "{pose: {header: {frame_id: map}, pose: {position: {x: 10.0, y: 5.0, z: 0.0},
   orientation: {w: 1.0}}}}"

# Or use the pre-defined waypoint follower if configured:
ros2 topic pub /waypoint_list nav2_msgs/msg/WaypointArray ...
''')
    story.append(PageBreak())

    # ── §6 During-Run Monitoring ──────────────────────────────────────────────
    story.append(H1('6  During-Run Monitoring'))
    story.append(Pb(
        'One team member watches RViz and the terminal output during the run. '
        'Know the override commands before the run starts.'
    ))

    story.append(H2('6.1  What to Watch in RViz'))
    story += Bl([
        '<b>Robot footprint marker</b> — should track physical robot position smoothly.',
        '<b>Planned path</b> (blue line) — should be a sensible route, not looping.',
        '<b>Local costmap</b> — red zone around the robot reflects real-time obstacles.',
        '<b>Global costmap</b> — should not have phantom obstacles from lidar noise.',
        '<b>Nav2 status panel</b> — watch for "Recovery" actions (spinning, backing up).',
    ])
    story.append(SP(4))

    story.append(H2('6.2  Key Terminal Monitoring Commands'))
    story += CB('Monitor from a second terminal (run simultaneously):', '''\
# Nav2 planner/controller status
ros2 topic echo /navigate_to_pose/_action/status

# Current velocity commands
ros2 topic echo /cmd_vel --once

# CPU and memory on Jetson
jtop  # install: sudo pip3 install jetson-stats

# Estop status
ros2 topic echo /estop_active --once

# Lidar health
ros2 topic hz /hesai/points   # must stay at ~20 Hz
''')
    story.append(SP(4))

    story.append(H2('6.3  Speed Limits'))
    limits_data = [
        [Paragraph('Parameter', styles['th']), Paragraph('Value', styles['th']),
         Paragraph('Where Set', styles['th'])],
        ['Max forward speed (vx_max)', '0.6 m/s', 'nav2_params.yaml — FollowPath.MotionModel'],
        ['Max reverse speed (vx_min)', '-0.1 m/s', 'nav2_params.yaml'],
        ['Max rotation speed (wz_max)', '1.2 rad/s', 'nav2_params.yaml'],
        ['Goal tolerance x/y', '0.20 m', 'nav2_params.yaml — goal_checker'],
        ['Goal tolerance yaw', '0.20 rad', 'nav2_params.yaml — goal_checker'],
        ['Progress timeout', '8 s', 'nav2_params.yaml — progress_checker'],
    ]
    limits_table = Table(limits_data, colWidths=[5.5*cm, 3*cm, 8*cm])
    limits_table.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), C_NAVY),
        ('ROWBACKGROUNDS', (0,1), (-1,-1), [C_WHITE, C_LGREY]),
        ('FONTSIZE', (0,0), (-1,-1), 8.5),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('GRID', (0,0), (-1,-1), 0.4, C_MGREY),
        ('TOPPADDING', (0,0), (-1,-1), 4),
        ('BOTTOMPADDING', (0,0), (-1,-1), 4),
    ]))
    story += [SP(4), limits_table, PageBreak()]

    # ── §7 Post-Run Bags ─────────────────────────────────────────────────────
    story.append(H1('7  Post-Run: Bag Recording &amp; Data Off-Load'))
    story.append(H2('7.1  Start Recording Before the Run'))
    story += CB('Start bag recording before sending the first goal:', '''\
source scripts/env.sh jetson
bash scripts/record_bag.sh
# Saves to: $DIY_BAG_PATH = /mnt/usb/bags/bag_YYYYMMDD_HHMMSS/
# Topics recorded: /hesai/points /imu/data /gps/fix
#   /odometry/filtered /odometry/global /cmd_vel /cmd_vel_safe
#   /tf /tf_static /estop_active /navigate_to_pose/_action/*
''')
    story.append(H2('7.2  Stop Recording and Verify'))
    story += CB('After the run, Ctrl+C the record terminal, then verify:', '''\
# Check bag is not empty
ros2 bag info /mnt/usb/bags/bag_<TIMESTAMP>/
# Should show: duration > 0s, all topics have messages

# Quick replay check (no network required)
bash scripts/replay_bag.sh /mnt/usb/bags/bag_<TIMESTAMP>/
''')
    story.append(SP(4))

    story.append(H2('7.3  Data Off-Load'))
    story += CB('Copy bags to laptop for post-processing:', '''\
# From laptop
rsync -av jetson@<JETSON_IP>:/mnt/usb/bags/ ~/bags/

# Or eject the USB drive and copy directly
umount /mnt/usb      # on Jetson
# physically remove drive, insert in laptop
''')
    story.append(NOTE(
        'Note the run timestamp and any anomalies observed during the run. '
        'This makes post-run analysis faster when reviewing LIO-SAM / EKF logs.'
    ))
    story.append(PageBreak())

    # ── §8 Failure Responses ──────────────────────────────────────────────────
    story.append(H1('8  Common Failure Responses'))
    story.append(Pb(
        'Failures are most likely to occur at start-line initialization or after '
        'a hard turn. Know these responses before competition begins.'
    ))
    tc_sm = styles['tc_sm']
    fail_data = [
        [Paragraph('Failure', styles['th']), Paragraph('Symptom', styles['th']),
         Paragraph('Response', styles['th'])],
        [Paragraph('Nav2 stuck in recovery', tc_sm),
         Paragraph('Robot spins in place or backs up repeatedly', tc_sm),
         Paragraph('Check costmap for phantom obstacle; ros2 topic pub /initialpose — relocalize', tc_sm)],
        [Paragraph('FAST-LIO2 diverged', tc_sm),
         Paragraph('/odometry/filtered pose jumps &gt;2 m; map display goes wrong', tc_sm),
         Paragraph('kill fastlio_mapping, restart localization.launch.py, confirm lidar is clean', tc_sm)],
        [Paragraph('Hesai lidar goes silent', tc_sm),
         Paragraph('/hesai/points drops to 0 Hz; health_check FAIL', tc_sm),
         Paragraph('Ping 192.168.1.201; reconnect Ethernet; power-cycle lidar 12V rail', tc_sm)],
        [Paragraph('GPS fix lost', tc_sm),
         Paragraph('/gps/fix status=-1; /odometry/gps stops', tc_sm),
         Paragraph('Switch use_gps:=false, restart EKF2; robot uses lidar-only localization', tc_sm)],
        [Paragraph('STM32 / micro-ROS crash', tc_sm),
         Paragraph('/cmd_vel_safe not reaching motors; /estop_active stuck true', tc_sm),
         Paragraph('Reconnect USB; ros2 run micro_ros_agent ... serial; reset robot estop', tc_sm)],
        [Paragraph('EKF diverges (covariance bloom)', tc_sm),
         Paragraph('/odometry/filtered covariance diagonal &gt; 1.0', tc_sm),
         Paragraph('Restart ekf_node; feed initial pose with ros2 topic pub /initialpose', tc_sm)],
        [Paragraph('Nav2 refuses to start', tc_sm),
         Paragraph('NavigateToPose action server not found', tc_sm),
         Paragraph('Check map_server has valid map YAML; check bt_navigator logs', tc_sm)],
    ]
    fail_table = Table(fail_data, colWidths=[4*cm, 5*cm, 7.5*cm])
    fail_table.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), C_NAVY),
        ('ROWBACKGROUNDS', (0,1), (-1,-1), [C_WHITE, C_LGREY]),
        ('VALIGN', (0,0), (-1,-1), 'TOP'),
        ('GRID', (0,0), (-1,-1), 0.4, C_MGREY),
        ('TOPPADDING', (0,0), (-1,-1), 4),
        ('BOTTOMPADDING', (0,0), (-1,-1), 4),
    ]))
    story += [SP(4), fail_table, PageBreak()]

    # ── §9 Emergency Stop ────────────────────────────────────────────────────
    story.append(H1('9  Emergency Stop Procedure'))
    story.append(CRIT(
        'Emergency stop cuts motor output immediately. The robot will coast to a stop '
        'with no braking. Ensure the area ahead of the robot is clear before resetting.'
    ))
    story.append(SP(6))

    story.append(H2('9.1  Hardware Estop'))
    story += Bl([
        'Press the physical red estop button on the robot chassis.',
        'STM32 firmware publishes /estop_active = true.',
        'cmd_vel_mux gates all velocity commands — motors stop.',
        'To resume: twist the estop button clockwise to release, then '
        'send /estop_reset service call or press the estop release button.',
    ])
    story.append(SP(4))

    story.append(H2('9.2  Software Estop (Remote)'))
    story += CB('Force estop via topic from any terminal:', '''\
# Immediate software stop
ros2 topic pub /estop_active std_msgs/msg/Bool "data: true" --once

# Resume when safe
ros2 topic pub /estop_active std_msgs/msg/Bool "data: false" --once

# Cancel all Nav2 goals without estop
ros2 action cancel /navigate_to_pose
''')
    story.append(SP(4))

    story.append(H2('9.3  Full Shutdown Sequence'))
    story += NS([
        'Press hardware estop button.',
        'Kill all ROS nodes: <tt>pkill -f ros2</tt> on Jetson.',
        'Stop bag recording if running: Ctrl+C on record terminal.',
        'Shut down Jetson: <tt>sudo shutdown now</tt>.',
        'Switch off 12 V rail (lidar power).',
        'Disconnect main battery.',
        'Do not reconnect battery within 30 s.',
    ])
    story.append(HR())
    story.append(Pb(
        '<i>End of Competition Day Playbook. '
        'For localization bringup detail see the Mapping &amp; Localization Workflow Guide. '
        'For Nav2 tuning see the Nav2 Tuning Guide.</i>'
    ))

    doc.build(story, onFirstPage=_on_cover, onLaterPages=_on_page)
    print(f'Written: {OUT_PATH}')


if __name__ == '__main__':
    build_pdf()
