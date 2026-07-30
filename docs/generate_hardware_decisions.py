#!/usr/bin/env python3
"""
generate_hardware_decisions.py
Produces docs/Hardware_Architecture_Decisions.pdf — captures team discussion
on sensor choices, compute, and localization strategy.
Run:  python3 docs/generate_hardware_decisions.py
"""

import os
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import cm
from reportlab.lib import colors
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.enums import TA_LEFT, TA_CENTER
from reportlab.platypus import (
    SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle,
    PageBreak, HRFlowable,
)

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_PATH  = os.path.join(REPO_ROOT, 'docs', 'Hardware_Architecture_Decisions.pdf')

PW, PH = A4
M = 2.0*cm
CW = PW - 2*M

# ─── Colors ──────────────────────────────────────────────────────────────────
NAVY   = colors.HexColor('#0F172A')
BLUE   = colors.HexColor('#1D4ED8')
TEAL   = colors.HexColor('#0D9488')
GREEN  = colors.HexColor('#16A34A')
PURPLE = colors.HexColor('#7C3AED')
ORANGE = colors.HexColor('#EA580C')
RED    = colors.HexColor('#DC2626')
DGREY  = colors.HexColor('#334155')
MGREY  = colors.HexColor('#64748B')
LGREY  = colors.HexColor('#F8FAFC')
WHITE  = colors.white
AMBER  = colors.HexColor('#D97706')

# ─── Styles ──────────────────────────────────────────────────────────────────
_n = [0]
def st(name, **kw):
    _n[0] += 1
    return ParagraphStyle(f'{name}_{_n[0]}', **kw)

TITLE  = st('T', fontName='Helvetica-Bold', fontSize=22, leading=28, textColor=NAVY, spaceAfter=4)
H1     = st('H1', fontName='Helvetica-Bold', fontSize=16, leading=22, textColor=NAVY, spaceBefore=18, spaceAfter=6)
H2     = st('H2', fontName='Helvetica-Bold', fontSize=13, leading=18, textColor=BLUE, spaceBefore=12, spaceAfter=4)
H3     = st('H3', fontName='Helvetica-Bold', fontSize=11, leading=15, textColor=DGREY, spaceBefore=8, spaceAfter=3)
BODY   = st('BD', fontName='Helvetica', fontSize=10.5, leading=15, textColor=DGREY, spaceAfter=4)
BULLET = st('BU', fontName='Helvetica', fontSize=10.5, leading=15, textColor=DGREY, leftIndent=16, spaceAfter=3)
NOTE   = st('NT', fontName='Helvetica-Oblique', fontSize=10, leading=14, textColor=MGREY, leftIndent=12, spaceAfter=6)
WARN   = st('WN', fontName='Helvetica-Bold', fontSize=10, leading=14, textColor=RED, leftIndent=0, spaceAfter=6)
HDR_W  = st('HW', fontName='Helvetica-Bold', fontSize=10.5, leading=14, textColor=WHITE)
VERDICT = st('VD', fontName='Helvetica-Bold', fontSize=11, leading=16, textColor=GREEN, spaceBefore=6, spaceAfter=4)

def SP(n=8): return Spacer(1, n)
def HR(): return HRFlowable(width='100%', thickness=1, color=BLUE, spaceAfter=8, spaceBefore=4)
def B(t): return Paragraph(f'• {t}', BULLET)
def P(t): return Paragraph(t, BODY)
def V(t): return Paragraph(f'✓ RECOMMENDATION: {t}', VERDICT)
def W(t): return Paragraph(f'⚠ {t}', WARN)


def info_box(title, bullets, bg=BLUE):
    w = CW - 24
    rows = [[Paragraph(title, HDR_W)]]
    for b in bullets:
        rows.append([Paragraph(f'• {b}', st('IB', fontName='Helvetica', fontSize=10,
            leading=14, textColor=WHITE, leftIndent=10, spaceAfter=2))])
    t = Table(rows, colWidths=[w])
    t.setStyle(TableStyle([
        ('BACKGROUND',(0,0),(-1,-1),bg),
        ('TOPPADDING',(0,0),(0,0),10),('TOPPADDING',(0,1),(-1,-1),2),
        ('BOTTOMPADDING',(0,0),(-1,-1),5),
        ('LEFTPADDING',(0,0),(-1,-1),14),('RIGHTPADDING',(0,0),(-1,-1),14),
        ('ROUNDEDCORNERS',[6]),
    ]))
    return t


def comparison_table(headers, rows_data, col_widths=None):
    all_rows = [headers] + rows_data
    if not col_widths:
        col_widths = [CW / len(headers)] * len(headers)
    tbl = Table(all_rows, colWidths=col_widths)
    tbl.setStyle(TableStyle([
        ('BACKGROUND',(0,0),(-1,0),NAVY),
        ('TEXTCOLOR',(0,0),(-1,0),WHITE),
        ('FONTNAME',(0,0),(-1,0),'Helvetica-Bold'),
        ('FONTSIZE',(0,0),(-1,-1),10),
        ('ROWBACKGROUNDS',(0,1),(-1,-1),[WHITE, LGREY]),
        ('TOPPADDING',(0,0),(-1,-1),6),('BOTTOMPADDING',(0,0),(-1,-1),6),
        ('LEFTPADDING',(0,0),(-1,-1),8),('RIGHTPADDING',(0,0),(-1,-1),8),
        ('GRID',(0,0),(-1,-1),0.4,colors.HexColor('#E2E8F0')),
        ('VALIGN',(0,0),(-1,-1),'TOP'),
    ]))
    return tbl


def verdict_box(text, bg=GREEN):
    t = Table([[Paragraph(f'✓  {text}', st('VB', fontName='Helvetica-Bold', fontSize=11,
        textColor=WHITE, leading=15))]], colWidths=[CW])
    t.setStyle(TableStyle([
        ('BACKGROUND',(0,0),(-1,-1),bg),
        ('TOPPADDING',(0,0),(-1,-1),10),('BOTTOMPADDING',(0,0),(-1,-1),10),
        ('LEFTPADDING',(0,0),(-1,-1),14),('RIGHTPADDING',(0,0),(-1,-1),14),
        ('ROUNDEDCORNERS',[6]),
    ]))
    return t


def build_pdf():
    doc = SimpleDocTemplate(OUT_PATH, pagesize=A4,
        leftMargin=M, rightMargin=M, topMargin=M, bottomMargin=M)

    s = []

    # ── Title ─────────────────────────────────────────────────────────────────
    s.append(Paragraph('Hardware & Architecture Decisions', TITLE))
    s.append(Paragraph('Betsybots · DIY Challenge 2026 · Team Discussion Document', st('SUB',
        fontName='Helvetica', fontSize=11, leading=15, textColor=MGREY, spaceAfter=4)))
    s.append(Paragraph('May 21, 2026 — Capturing decisions from team discussion', st('DT',
        fontName='Helvetica-Oblique', fontSize=9, leading=13, textColor=MGREY, spaceAfter=8)))
    s.append(HR())
    s.append(SP(4))

    s.append(P('This document captures the technical discussions and recommendations around sensor choices, '
               'compute architecture, and localization strategy. Each section presents the question raised, '
               'the analysis, and the team recommendation.'))

    # ══════════════════════════════════════════════════════════════════════════
    # SECTION 1: RTK GPS PUSHBACK
    # ══════════════════════════════════════════════════════════════════════════
    s.append(Paragraph('1. RTK GPS — Does It "Defeat the Purpose" of Localization?', H1))

    s.append(Paragraph('The Concern', H2))
    s.append(P('Team member raised: "Using an accurate RTK GPS module with 1 cm accuracy defeats the '
               'purpose of building a localization system. If GPS is that good, why do we need anything else?"'))

    s.append(Paragraph('Why This Concern is Partially Valid but Incomplete', H2))
    s.append(B('<b>GPS gives position only</b> — no rotation/heading. You still need IMU + LiDAR for orientation.'))
    s.append(B('<b>GPS drops out</b> — under trees, near buildings, in tunnels, or if the competition has GPS-denied zones.'))
    s.append(B('<b>GPS rate is slow</b> — 10–20 Hz vs LiDAR odometry at 100+ Hz. Real-time control needs the fast rate.'))
    s.append(B('<b>GPS alone can\'t navigate</b> — you need relative odometry for obstacle avoidance and path tracking.'))
    s.append(SP(4))
    s.append(P('The real architecture: <b>FAST-LIO gives primary odometry, RTK is a drift-correction signal in the EKF</b>. '
               'If GPS drops, the system still works. If GPS is there, it prevents long-term drift accumulation.'))

    s.append(Paragraph('Alternatives Discussed', H2))
    s.append(comparison_table(
        ['Option', 'Pros', 'Cons'],
        [
            ['RTK GPS\n(secondary EKF input)',
             'Drift-free position\nCheap modules ($150–200)\nSelf-contained (just needs sky)',
             'No rotation data\nDrops in some areas\nSlow update rate'],
            ['Dead-wheel encoders',
             'Simple, reliable, fast rate\n$20–30 total cost\nGives distance + heading (differential)',
             'Drifts on rough terrain (slip)\nNeeds calibration\nMechanical wear'],
            ['Visual odometry\n(stereo/mono camera)',
             'Works indoors\nNo external infrastructure',
             'Compute-heavy\nFragile outdoors (lighting)\nNeeds GPU or heavy CPU'],
        ],
        [3.5*cm, 5.5*cm, 5.5*cm]
    ))

    s.append(SP(8))
    s.append(verdict_box('Use RTK as secondary EKF input (not primary odom). Add dead-wheel encoders as cheap insurance. Skip visual odometry.'))

    s.append(SP(6))
    s.append(info_box('Why Dead-Wheel Encoders Are Worth Adding', [
        '$20–30 total cost — trivial compared to other sensors',
        'Give you wheel-slip detection (compare encoder odom vs LiDAR odom)',
        'Fast update rate (1000+ Hz) — helps EKF between LiDAR scans',
        'Easy ROS 2 integration — just publish nav_msgs/Odometry',
        'Works as a fallback if both GPS and LiDAR have issues simultaneously',
    ], TEAL))

    # ══════════════════════════════════════════════════════════════════════════
    # SECTION 2: UWB
    # ══════════════════════════════════════════════════════════════════════════
    s.append(PageBreak())
    s.append(Paragraph('2. UWB (Ultra-Wideband) Positioning', H1))

    s.append(Paragraph('The Question', H2))
    s.append(P('"Can we use UWB for positioning? Is it allowed by the competition rules?"'))

    s.append(Paragraph('Analysis', H2))
    s.append(B('UWB requires <b>pre-placed anchors</b> around the course at known positions'))
    s.append(B('If the competition organizers don\'t provide anchors and you can\'t place your own → not practical'))
    s.append(B('RTK GPS is self-contained — just needs sky view, no external infrastructure'))
    s.append(B('UWB gives ~10 cm accuracy indoors (comparable to RTK outdoors), but adds infrastructure dependency'))
    s.append(SP(4))

    s.append(verdict_box('Check competition rules. If allowed AND you can place your own anchors, it\'s viable. Otherwise, RTK is simpler and self-contained.', AMBER))
    s.append(SP(4))
    s.append(P('<i>Action item: Review the official competition rules document for permitted positioning systems '
               'and any restrictions on pre-placed infrastructure.</i>'))

    # ══════════════════════════════════════════════════════════════════════════
    # SECTION 3: APRILTAG LANDMARKS
    # ══════════════════════════════════════════════════════════════════════════
    s.append(Paragraph('3. AprilTag Landmarks for Online Position Correction', H1))

    s.append(Paragraph('The Idea', H2))
    s.append(P('During offline map generation (without obstacles), place AprilTags at known positions. '
               'During competition, camera detects these tags and provides global position corrections to the EKF.'))

    s.append(Paragraph('How This Works', H2))
    s.append(B('<b>Step 1:</b> During offline mapping run, place AprilTags at key positions on the course'))
    s.append(B('<b>Step 2:</b> Survey each tag\'s global position (using RTK or manual measurement)'))
    s.append(B('<b>Step 3:</b> Record tag positions in the map as known landmarks'))
    s.append(B('<b>Step 4:</b> During competition, camera detects tags → known global position → feeds EKF'))
    s.append(B('<b>Result:</b> Acts like "indoor GPS" — gives global position fixes at specific points'))
    s.append(SP(4))

    s.append(Paragraph('Where to Place Tags', H2))
    s.append(comparison_table(
        ['Location', 'Why', 'Priority'],
        [
            ['Start/finish line', 'Known exact position, visible on every lap', 'High'],
            ['Before obstacle zones', 'Reset position error before critical navigation', 'High'],
            ['After obstacle zones', 'Correct any drift accumulated during obstacle nav', 'Medium'],
            ['At major turns', 'Heading correction at geometry changes', 'Medium'],
            ['Along straightaways', 'Drift correction on long sections', 'Low'],
        ],
        [4.0*cm, 7.0*cm, 2.5*cm]
    ))

    s.append(SP(6))
    s.append(info_box('Advantages of This Approach', [
        'Works even WITHOUT RTK GPS — tags give you absolute position',
        'CPU-lightweight: AprilTag detection runs at ~30 Hz on ARM without GPU',
        'Zero cost: just printed paper tags on rigid boards',
        'Can be combined with RTK — tag corrections fill GPS gaps (near structures, etc.)',
        'Already planned: our AprilTag calibration guide covers the EKF integration',
    ], GREEN))

    s.append(SP(6))
    s.append(verdict_box('YES — implement AprilTag landmarks. Place tags during offline mapping, survey positions with RTK, use for online EKF corrections.'))

    # ══════════════════════════════════════════════════════════════════════════
    # SECTION 4: IMU — MTLT335D
    # ══════════════════════════════════════════════════════════════════════════
    s.append(PageBreak())
    s.append(Paragraph('4. Our IMU: ACEINNA MTLT335D', H1))

    s.append(Paragraph('What We Have', H2))
    s.append(P('The ACEINNA MTLT335D is an industrial-grade 6-DOF IMU — significantly better than consumer-grade '
               'units like the RealSense D435i\'s built-in BMI055 or typical $15 breakout boards.'))
    s.append(SP(4))

    s.append(comparison_table(
        ['Specification', 'MTLT335D (Ours)', 'Typical Consumer IMU'],
        [
            ['DOF', '6 (3-axis accel + 3-axis gyro)', '6 or 9'],
            ['Gyro bias stability', '1.3 °/hr (excellent)', '10–50 °/hr'],
            ['Roll/Pitch accuracy', '0.05° (built-in KF)', '0.5–2°'],
            ['Gyro range', '±400 °/s', '±250–2000 °/s'],
            ['Accel range', '±8 g', '±2–16 g'],
            ['Output rate', '100/200 Hz', '100–1000 Hz'],
            ['Redundancy', 'Triple-redundant with fault detection', 'Single sensor'],
            ['Temp calibration', 'Full range -40°C to +85°C', 'None or limited'],
            ['Interface', 'CAN 2.0 / RS232', 'I2C / SPI / USB'],
            ['Built-in processing', '16-state EKF (pitch/roll)', 'None'],
            ['Housing', 'IP67 rugged sealed', 'Bare PCB'],
        ],
        [4.0*cm, 5.5*cm, 5.0*cm]
    ))

    s.append(SP(8))
    s.append(Paragraph('6-DOF vs 9-DOF — Do We Need a Magnetometer?', H2))
    s.append(P('The MTLT335D is 6-DOF (no magnetometer). This is <b>fine</b> for our application:'))
    s.append(B('FAST-LIO does NOT use magnetometer data — only gyro + accel'))
    s.append(B('Heading comes from LiDAR scan matching (very accurate) + gyro integration (fast)'))
    s.append(B('Outdoor magnetometers are unreliable near motors, batteries, and metal structures'))
    s.append(B('Our robot starts at a known heading; LiDAR-to-map matching gives absolute heading within seconds'))
    s.append(SP(4))
    s.append(verdict_box('6-DOF is correct for this application. Do NOT add a magnetometer.'))

    s.append(SP(8))
    s.append(Paragraph('Integration Requirements', H2))
    s.append(P('The MTLT335D outputs on <b>CAN 2.0 or RS232</b>, not USB. Integration path:'))
    s.append(SP(4))
    s.append(info_box('Integration Steps', [
        'Option A: CAN-to-USB adapter (PEAK PCAN-USB ~$40) connected to Orin Nano',
        'Option B: RS232 output with USB-to-serial adapter',
        'Write a thin ROS 2 driver node that parses MTLT335D protocol → publishes sensor_msgs/Imu',
        'Protocol is documented in ACEINNA user manual (available on their site)',
        'Mount RIGIDLY to LiDAR frame — vibration isolation between them ruins calibration',
    ], BLUE))

    s.append(SP(4))
    s.append(W('Do NOT use the RealSense D435i built-in IMU. The MTLT335D is vastly superior. '
               'The D435i IMU is consumer-grade and poorly positioned relative to the LiDAR.'))

    # ══════════════════════════════════════════════════════════════════════════
    # SECTION 5: CAMERA — D435i vs D555
    # ══════════════════════════════════════════════════════════════════════════
    s.append(PageBreak())
    s.append(Paragraph('5. Camera: RealSense D435i vs D555', H1))

    s.append(Paragraph('Current Role of the Camera', H2))
    s.append(P('The camera is a <b>secondary/helper sensor</b> — not on the critical path. If it dies mid-run, '
               'the robot still navigates. Three use cases:'))
    s.append(SP(4))

    s.append(comparison_table(
        ['Use Case', 'What It Does', 'Priority'],
        [
            ['AprilTag detection', 'EKF position corrections at known landmarks (CPU, RGB only)', 'Medium'],
            ['Visual start signal', 'Watch for "go" signal to avoid 5-sec penalty', 'Low'],
            ['Depth assist (chimes)', 'One specific obstacle where LiDAR has ground-level blind spot', 'Low'],
        ],
        [4.0*cm, 7.0*cm, 2.5*cm]
    ))

    s.append(SP(8))
    s.append(Paragraph('D435i vs D555 Comparison', H2))
    s.append(comparison_table(
        ['Feature', 'D435i (current)', 'D555 (available)'],
        [
            ['Outdoor depth range', '~3 m usable in sunlight', '~6 m in sunlight'],
            ['Ambient light rejection', 'Basic', 'Significantly better'],
            ['RGB quality', 'Good (1080p)', 'Good (1080p)'],
            ['Built-in IMU', 'Yes (BMI055)', 'No'],
            ['ROS 2 driver', 'realsense2_camera', 'Same — drop-in replacement'],
            ['Form factor', 'Small, USB-C', 'Similar, USB-C'],
            ['Cost', 'Already have', 'Already available'],
        ],
        [4.0*cm, 5.0*cm, 5.5*cm]
    ))

    s.append(SP(6))
    s.append(verdict_box('Either camera works. D555 is a free upgrade if available (better outdoor depth). '
                         'Since we use the MTLT335D as our IMU, losing the D435i\'s built-in IMU is irrelevant.'))
    s.append(SP(4))
    s.append(P('<i>Low priority — swap only after core system is working. The camera is the least critical sensor.</i>'))

    # ══════════════════════════════════════════════════════════════════════════
    # SECTION 6: COMPUTE ARCHITECTURE
    # ══════════════════════════════════════════════════════════════════════════
    s.append(Paragraph('6. Compute Architecture — Orin Nano vs Multiple Pis', H1))

    s.append(Paragraph('The Question', H2))
    s.append(P('"Is it better to use multiple Raspberry Pis, or stick with Orin Nano + one Pi?"'))

    s.append(SP(4))
    s.append(Paragraph('Architecture Comparison', H2))
    s.append(comparison_table(
        ['', 'Orin Nano + 1 Pi', '3× Raspberry Pi 5'],
        [
            ['Total CPU cores', '6 (Orin A78AE) + 4 (Pi A76) = 10', '3×4 = 12 (but weaker per-core)'],
            ['Single-core perf', 'Orin ~2× faster per-core than Pi 5', 'Slower per-core'],
            ['RAM', '8 GB + 8 GB = 16 GB (shared bus)', '3×8 = 24 GB (split across machines)'],
            ['GPU', '40 TOPS (available if needed)', 'None'],
            ['Network overhead', '1 hop (DDS between 2 machines)', '2–3 hops, 3-way DDS discovery'],
            ['Time sync', '1 pair (PTP/chrony)', '3-way sync (harder)'],
            ['Power draw', '~15W + ~5W = 20W', '3×8W = 24W'],
            ['Debug complexity', 'Low — one main brain', 'High — which Pi has what?'],
        ],
        [3.5*cm, 5.5*cm, 5.5*cm]
    ))

    s.append(SP(8))
    s.append(Paragraph('Why Multiple Pis Is a Bad Fit', H2))
    s.append(B('FAST-LIO needs sustained single-core performance at 100 Hz — Pi 5 barely keeps up'))
    s.append(B('Nav2 planner/controller is memory-hungry and CPU-intensive — wants Orin\'s fast cores'))
    s.append(B('Splitting ROS 2 nodes across Pis = network latency on /cmd_vel, /scan, /odom'))
    s.append(B('Debugging multi-machine ROS 2: which machine\'s log? which DDS domain? clock drift?'))
    s.append(B('No team member should spend time on distributed systems debugging instead of the actual challenge'))

    s.append(SP(6))
    s.append(Paragraph('When to Add the Single Pi', H2))
    s.append(B('Only if CPU contention is observed during full-stack testing (check with htop)'))
    s.append(B('Offload camera node (realsense + AprilTag) to Pi to free Orin CPU'))
    s.append(B('Offload bag recording (disk I/O on separate device)'))
    s.append(SP(4))

    s.append(verdict_box('Orin Nano runs everything. Add 1 Pi ONLY if htop shows CPU contention during full runs. Never use multiple Pis.'))

    # ══════════════════════════════════════════════════════════════════════════
    # SECTION 7: GPU STRETCH GOALS
    # ══════════════════════════════════════════════════════════════════════════
    s.append(PageBreak())
    s.append(Paragraph('7. GPU Utilization — Stretch Goals (July/August)', H1))

    s.append(P('The Orin Nano has <b>1024 CUDA cores / 40 TOPS</b> sitting completely idle in the current stack. '
               'Everything we run (FAST-LIO, Nav2, EKF) is CPU-only. If we want extra robustness after the '
               'base system works, here\'s what the GPU can do — all from NVIDIA\'s Isaac ROS packages.'))

    s.append(SP(4))
    s.append(Paragraph('Isaac ROS Packages (Pre-built for Jetson + ROS 2 Humble)', H2))
    s.append(comparison_table(
        ['Package', 'What It Does', 'Benefit to Us'],
        [
            ['Isaac ROS cuVSLAM', 'GPU-accelerated stereo visual SLAM\nfrom camera images',
             'Free second odometry source\nRuns on GPU = zero CPU cost\nWorks when LiDAR struggles'],
            ['Isaac ROS Nvblox', 'Real-time 3D voxel map\nfrom depth + LiDAR on GPU',
             'True 3D obstacle awareness\nCatches overhanging obstacles\nBetter than 2D costmap'],
            ['Isaac ROS AprilTag', 'GPU-accelerated tag detection',
             'Faster/more reliable than CPU apriltag_ros\nFrees CPU for other tasks'],
            ['Isaac ROS Freespace\nSegmentation', 'DNN classifies drivable\nsurface vs obstacle',
             'Could replace LiDAR cluster classifier\nBut needs training data — lower priority'],
        ],
        [4.0*cm, 5.0*cm, 5.5*cm]
    ))

    s.append(SP(10))
    s.append(Paragraph('Highest-Value Addition: cuVSLAM', H2))
    s.append(P('GPU-accelerated Visual SLAM from NVIDIA, purpose-built for Jetson. Takes stereo images '
               '(from D435i or D555) and outputs visual odometry at 60+ Hz — entirely on GPU.'))
    s.append(SP(4))
    s.append(B('<b>Runs on GPU</b> → CPU stays free for FAST-LIO + Nav2'))
    s.append(B('<b>Second independent odometry source</b> → feeds EKF as Odom1'))
    s.append(B('<b>Works when LiDAR might struggle</b> — featureless open field, dust, rain'))
    s.append(B('<b>Zero additional hardware</b> — uses the camera already on the robot'))
    s.append(B('<b>Pre-built ROS 2 package</b> — apt install ros-humble-isaac-ros-visual-slam'))
    s.append(B('<b>Integration effort</b> — 2–3 hours (launch file + EKF config addition)'))

    s.append(SP(6))
    s.append(Paragraph('EKF Architecture With cuVSLAM Added', H2))
    s.append(SP(4))

    ekf_gpu = [
        ['Source', 'Sensor', 'Output', 'Runs On', 'Rate'],
        ['FAST-LIO', 'QT64 + MTLT335D', 'Odom0 (position + orientation)', 'CPU', '100 Hz'],
        ['cuVSLAM', 'D435i/D555 stereo', 'Odom1 (visual odometry)', 'GPU', '60 Hz'],
        ['MTLT335D', 'IMU direct', 'IMU0 (angular velocity)', 'CPU', '200 Hz'],
        ['Wheel encoders', 'Dead-wheels', 'Odom2 (wheel odometry)', 'CPU', '100+ Hz'],
        ['RTK GPS', 'GPS module', 'GPS0 (global position)', 'CPU', '10–20 Hz'],
        ['AprilTags', 'Camera RGB', 'Pose corrections (event)', 'GPU*', '30 Hz'],
    ]
    tbl_ekf = Table(ekf_gpu, colWidths=[2.8*cm, 3.5*cm, 4.8*cm, 1.5*cm, 1.9*cm])
    tbl_ekf.setStyle(TableStyle([
        ('BACKGROUND',(0,0),(-1,0),PURPLE),
        ('TEXTCOLOR',(0,0),(-1,0),WHITE),
        ('FONTNAME',(0,0),(-1,0),'Helvetica-Bold'),
        ('FONTSIZE',(0,0),(-1,-1),9.5),
        ('ROWBACKGROUNDS',(0,1),(-1,-1),[WHITE, LGREY]),
        ('TOPPADDING',(0,0),(-1,-1),5),('BOTTOMPADDING',(0,0),(-1,-1),5),
        ('LEFTPADDING',(0,0),(-1,-1),6),('RIGHTPADDING',(0,0),(-1,-1),6),
        ('GRID',(0,0),(-1,-1),0.4,colors.HexColor('#E2E8F0')),
        ('VALIGN',(0,0),(-1,-1),'TOP'),
    ]))
    s.append(tbl_ekf)
    s.append(SP(2))
    s.append(Paragraph('<i>* Isaac ROS AprilTag runs on GPU; standard apriltag_ros runs on CPU.</i>', NOTE))

    s.append(SP(8))
    s.append(P('<b>Key insight:</b> Two independent odometry sources (LiDAR + visual) means if either has a '
               'bad moment, the EKF detects the disagreement and weights the good one higher. This is '
               'significantly more robust than any single-source approach.'))

    s.append(SP(8))
    s.append(Paragraph('Nvblox — 3D Costmap on GPU', H2))
    s.append(P('Nav2\'s default costmap is 2D (single height slice). Nvblox builds a full 3D voxel map on GPU '
               'from depth camera + LiDAR point cloud, then outputs a 2D costmap slice for Nav2.'))
    s.append(SP(4))
    s.append(B('Catches <b>overhanging obstacles</b> (branches, signs) that 2D LiDAR misses'))
    s.append(B('Better for <b>chimes obstacle</b> (thin vertical objects at various heights)'))
    s.append(B('<b>GPU-accelerated</b> → no CPU cost'))
    s.append(B('Outputs standard nav_msgs/OccupancyGrid → plugs directly into Nav2'))

    s.append(SP(8))
    s.append(Paragraph('What NOT to Use the GPU For', H2))
    s.append(B('<b>YOLO / object detection</b> — not needed, LiDAR geometry is sufficient for our obstacles'))
    s.append(B('<b>Semantic segmentation</b> — overkill, we don\'t need pixel-level classification'))
    s.append(B('<b>Any model requiring training data</b> — we don\'t have labelled datasets and can\'t create them in time'))
    s.append(B('<b>Anything foundational</b> — GPU features are stretch goals, not critical path'))

    s.append(SP(8))
    s.append(info_box('Timeline for GPU Integration', [
        'June: DO NOT touch GPU features — focus on calibration, mapping, base stack',
        'July (if base system is stable): Add cuVSLAM as Odom1 in EKF — 2–3 hour task',
        'Aug 1–15 (if cuVSLAM works well): Consider Nvblox for 3D costmap',
        'Aug 15: HARD FEATURE LOCK — 45 days of bug fixes and practice runs until Oct 1',
    ], PURPLE))

    s.append(SP(4))
    s.append(verdict_box('GPU stretch goals are additive, not foundational. Get the CPU-only stack working first. '
                         'cuVSLAM in July is the highest-value GPU addition.'))

    # ══════════════════════════════════════════════════════════════════════════
    # SECTION 8: BUDGET ALLOCATION
    # ══════════════════════════════════════════════════════════════════════════
    s.append(PageBreak())
    s.append(Paragraph('8. $500 Budget Allocation', H1))

    s.append(Paragraph('Recommended Spend', H2))
    s.append(comparison_table(
        ['Item', 'Est. Cost', 'Why'],
        [
            ['RTK GPS module\n(SparkFun / ArduSimple)', '$150–200', 'Drift-free position corrections in EKF'],
            ['Dead-wheel encoder kit', '$20–30', 'Cheap odom backup, slip detection'],
            ['CAN-to-USB adapter\n(for MTLT335D)', '$40', 'Connect IMU to Orin Nano'],
            ['Extra LiPo batteries', '$60–80', 'Extended field testing sessions'],
            ['Connectors, cables,\nmounting hardware', '$40–50', 'Rigid sensor mounting, wiring'],
            ['AprilTag boards\n(rigid printing)', '$15–20', 'Printed tags on foam/aluminum board'],
            ['Buffer for breakage', '$100–150', 'Spare parts, replacement cables, etc.'],
        ],
        [4.5*cm, 2.5*cm, 7.5*cm]
    ))
    s.append(SP(4))
    s.append(P('<b>Total estimated: $425–530</b> — within budget with buffer for unexpected needs.'))

    s.append(SP(8))
    s.append(Paragraph('What NOT to Spend On', H2))
    s.append(B('<b>Additional compute boards</b> — Orin Nano is sufficient'))
    s.append(B('<b>Better camera</b> — D555 already available, D435i already sufficient'))
    s.append(B('<b>Better IMU</b> — MTLT335D is industrial-grade, more than enough'))
    s.append(B('<b>Magnetometer</b> — not needed, unreliable near motors'))
    s.append(B('<b>UWB system</b> — needs infrastructure, uncertain if allowed'))
    s.append(B('<b>GPU inference hardware</b> — we\'re not doing camera-based detection'))

    # ══════════════════════════════════════════════════════════════════════════
    # SECTION 9: FINAL SENSOR ARCHITECTURE
    # ══════════════════════════════════════════════════════════════════════════
    s.append(Paragraph('9. Final Sensor Architecture Summary', H1))
    s.append(SP(4))

    s.append(comparison_table(
        ['Sensor', 'Role', 'EKF Layer', 'Priority'],
        [
            ['Hesai QT64 LiDAR', 'Primary odometry (via FAST-LIO)\nObstacle detection (point cloud)', 'Odom0 (100 Hz)', 'CRITICAL'],
            ['ACEINNA MTLT335D', 'Gyro + accel for FAST-LIO\nAngular velocity for EKF', 'IMU0 (200 Hz)', 'CRITICAL'],
            ['RTK GPS (new)', 'Global position correction\nPrevents long-term drift', 'GPS0 (10–20 Hz)', 'HIGH'],
            ['Dead-wheel encoders\n(new)', 'Wheel odometry\nSlip detection', 'Odom1 (100+ Hz)', 'MEDIUM'],
            ['Camera (D435i/D555)', 'AprilTag detection\nStart signal\nChimes depth assist', 'Via AprilTag\n(event-based)', 'LOW'],
        ],
        [3.8*cm, 5.0*cm, 3.0*cm, 2.7*cm]
    ))

    s.append(SP(10))
    s.append(Paragraph('EKF Fusion Architecture', H2))
    s.append(SP(4))

    ekf_rows = [
        ['Layer', 'Inputs', 'Output', 'Rate'],
        ['FAST-LIO\n(LiDAR-inertial)', 'QT64 point cloud + MTLT335D IMU', 'High-rate odometry\n(position + orientation)', '100 Hz'],
        ['Local EKF\n(robot_localization)', 'FAST-LIO odom + wheel encoders\n+ IMU angular velocity', 'Smooth local odom\n(for Nav2 control)', '50 Hz'],
        ['Global EKF\n(robot_localization)', 'Local odom + RTK GPS\n+ AprilTag detections', 'Global pose estimate\n(map frame)', '50 Hz'],
    ]
    tbl = Table(ekf_rows, colWidths=[3.0*cm, 5.5*cm, 4.0*cm, 2.0*cm])
    tbl.setStyle(TableStyle([
        ('BACKGROUND',(0,0),(-1,0),TEAL),
        ('TEXTCOLOR',(0,0),(-1,0),WHITE),
        ('FONTNAME',(0,0),(-1,0),'Helvetica-Bold'),
        ('FONTSIZE',(0,0),(-1,-1),9.5),
        ('ROWBACKGROUNDS',(0,1),(-1,-1),[WHITE, LGREY]),
        ('TOPPADDING',(0,0),(-1,-1),6),('BOTTOMPADDING',(0,0),(-1,-1),6),
        ('LEFTPADDING',(0,0),(-1,-1),8),('RIGHTPADDING',(0,0),(-1,-1),8),
        ('GRID',(0,0),(-1,-1),0.4,colors.HexColor('#E2E8F0')),
        ('VALIGN',(0,0),(-1,-1),'TOP'),
    ]))
    s.append(tbl)

    s.append(SP(8))
    s.append(P('<b>Key insight:</b> Each layer adds robustness. If any single sensor fails, the other layers '
               'continue working. RTK drops? Local EKF + LiDAR still navigates. Wheel slip? LiDAR odom '
               'is unaffected. Camera obscured? Only AprilTag corrections stop — everything else is fine.'))

    # ══════════════════════════════════════════════════════════════════════════
    # SECTION 10: DECISIONS SUMMARY
    # ══════════════════════════════════════════════════════════════════════════
    s.append(PageBreak())
    s.append(Paragraph('10. Decisions Summary', H1))
    s.append(SP(4))

    decisions = [
        ['#', 'Question', 'Decision', 'Status'],
        ['1', 'RTK GPS defeats localization?', 'No — use as secondary EKF correction, not primary', 'DECIDED'],
        ['2', 'Dead-wheel encoders?', 'Yes — add them ($20, easy integration)', 'DECIDED'],
        ['3', 'Visual odometry?', 'No — too compute-heavy, fragile outdoors', 'DECIDED'],
        ['4', 'UWB allowed?', 'Check rules. Probably skip (needs infrastructure)', 'PENDING'],
        ['5', 'AprilTag landmarks?', 'Yes — place during offline map, use for online correction', 'DECIDED'],
        ['6', 'Magnetometer needed?', 'No — 6-DOF is correct, mag unreliable near motors', 'DECIDED'],
        ['7', 'IMU choice?', 'ACEINNA MTLT335D — industrial grade, already have it', 'DECIDED'],
        ['8', 'Camera D435i vs D555?', 'Either works. Swap to D555 if convenient, low priority', 'DECIDED'],
        ['9', 'Multiple Pis?', 'No — Orin Nano handles everything. Add 1 Pi only if needed', 'DECIDED'],
        ['10', 'Separate gyroscope?', 'No — MTLT335D has excellent gyro (1.3°/hr stability)', 'DECIDED'],
        ['11', 'GPU utilization?', 'cuVSLAM in July as stretch goal. Nvblox in Aug. Not before.', 'PLANNED'],
    ]
    tbl_d = Table(decisions, colWidths=[0.8*cm, 4.0*cm, 6.5*cm, 2.2*cm])
    tbl_d.setStyle(TableStyle([
        ('BACKGROUND',(0,0),(-1,0),NAVY),
        ('TEXTCOLOR',(0,0),(-1,0),WHITE),
        ('FONTNAME',(0,0),(-1,0),'Helvetica-Bold'),
        ('FONTSIZE',(0,0),(-1,-1),9.5),
        ('ROWBACKGROUNDS',(0,1),(-1,-1),[WHITE, LGREY]),
        ('TOPPADDING',(0,0),(-1,-1),6),('BOTTOMPADDING',(0,0),(-1,-1),6),
        ('LEFTPADDING',(0,0),(-1,-1),6),('RIGHTPADDING',(0,0),(-1,-1),6),
        ('GRID',(0,0),(-1,-1),0.4,colors.HexColor('#E2E8F0')),
        ('VALIGN',(0,0),(-1,-1),'TOP'),
        ('TEXTCOLOR',(3,1),(3,-1),GREEN),
        ('FONTNAME',(3,1),(3,-1),'Helvetica-Bold'),
    ]))
    s.append(tbl_d)

    s.append(SP(12))
    s.append(info_box('Action Items for Next Session', [
        'Check competition rules for UWB / external infrastructure restrictions',
        'Source RTK GPS module (SparkFun RTK Express or ArduSimple)',
        'Source dead-wheel encoder kit (rotary encoders + mounting brackets)',
        'Get CAN-to-USB adapter for MTLT335D integration',
        'Write MTLT335D ROS 2 driver node (parse CAN/serial → sensor_msgs/Imu)',
        'Plan AprilTag placement during first offline mapping run',
    ], BLUE))

    s.append(SP(16))
    s.append(Paragraph('— Betsybots Team · DIY Challenge 2026', st('FT', fontName='Helvetica-Oblique',
        fontSize=9, textColor=MGREY, alignment=TA_CENTER)))

    doc.build(s)
    print(f'[OK] PDF written to: {OUT_PATH}')


if __name__ == '__main__':
    build_pdf()
