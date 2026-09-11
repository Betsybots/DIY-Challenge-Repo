#!/usr/bin/env python3
"""
generate_custom_nav_stack_design.py — Custom Navigation Stack Design & Zone
Nav Decoupling guide.
Team Juggernauts · DIY Robot Challenge 2026

Produces docs/Custom_Nav_Stack_Design.pdf using ReportLab.
Source content: docs/custom_nav_stack_design.md (kept in sync manually).
Run from repo root:   python3 docs/generate_custom_nav_stack_design.py
"""

import os
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import cm
from reportlab.lib import colors
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.enums import TA_CENTER, TA_JUSTIFY
from reportlab.platypus import (
    SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle,
    PageBreak, HRFlowable, Preformatted, Image,
)

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_PATH = os.path.join(REPO_ROOT, 'docs', 'Custom_Nav_Stack_Design.pdf')
IMG_DIR = os.path.join(REPO_ROOT, 'docs', 'img')

# ── Colour palette (matches this repo's other generated guides) ───────────────
C_NAVY = colors.HexColor('#1A2744')
C_BLUE = colors.HexColor('#2563EB')
C_TEAL = colors.HexColor('#0F766E')
C_GREEN = colors.HexColor('#16A34A')
C_RED = colors.HexColor('#DC2626')
C_LGREY = colors.HexColor('#F1F5F9')
C_MGREY = colors.HexColor('#94A3B8')
C_BLACK = colors.HexColor('#0F172A')
C_WHITE = colors.white


def make_styles():
    base = getSampleStyleSheet()
    s = {}
    s['title'] = ParagraphStyle('DocTitle', parent=base['Title'],
        fontSize=25, leading=31, textColor=C_WHITE, spaceAfter=6, alignment=TA_CENTER)
    s['subtitle'] = ParagraphStyle('DocSubtitle',
        fontSize=12, leading=17, textColor=colors.HexColor('#CBD5E1'),
        spaceAfter=4, alignment=TA_CENTER)
    s['cover_body'] = ParagraphStyle('CoverBody', fontSize=10.5, leading=15,
        textColor=colors.HexColor('#E2E8F0'), spaceAfter=6, alignment=TA_CENTER)
    s['h1'] = ParagraphStyle('H1', fontSize=16, leading=20, textColor=C_NAVY,
        spaceBefore=20, spaceAfter=6, fontName='Helvetica-Bold')
    s['h2'] = ParagraphStyle('H2', fontSize=13, leading=17, textColor=C_BLUE,
        spaceBefore=14, spaceAfter=4, fontName='Helvetica-Bold')
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
        textColor=colors.HexColor('#7C2D12'), backColor=colors.HexColor('#FEF3C7'),
        borderPad=6, spaceAfter=6)
    s['note'] = ParagraphStyle('Note', fontSize=9.5, leading=13,
        textColor=colors.HexColor('#1E3A5F'), backColor=colors.HexColor('#DBEAFE'),
        borderPad=6, spaceAfter=6)
    s['tip'] = ParagraphStyle('Tip', fontSize=9.5, leading=13,
        textColor=colors.HexColor('#065F46'), backColor=colors.HexColor('#D1FAE5'),
        borderPad=6, spaceAfter=6)
    s['th'] = ParagraphStyle('TH', fontName='Helvetica-Bold',
        fontSize=9, textColor=C_WHITE, alignment=TA_CENTER)
    s['tc'] = ParagraphStyle('TC', fontName='Helvetica',
        fontSize=9, textColor=C_BLACK, leading=13)
    s['img_caption'] = ParagraphStyle('ImgCaption', fontName='Helvetica-Oblique',
        fontSize=8.5, textColor=C_MGREY, alignment=TA_CENTER, spaceAfter=10)
    return s


# ── Helper builders ─────────────────────────────────────────────────────────────

def H(text, level, s): return Paragraph(text, s[level])
def P(text, s): return Paragraph(text, s['body'])
def B(items, s): return [Paragraph(f'&bull;  {item}', s['bullet']) for item in items]
def SP(n=6): return Spacer(1, n)


def code_block(label, text, s):
    out = []
    if label:
        out.append(Paragraph(label, s['code_label']))
    out.append(Preformatted(text, s['code']))
    return out


def hr(s): return HRFlowable(width='100%', thickness=0.5, color=C_MGREY, spaceAfter=8)
def warn_box(t, s): return Paragraph('<b>WARNING &mdash;</b>&nbsp; ' + t, s['warn'])
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


def result_table(rows, s, col_widths):
    """Table with a colour-coded PASS/status column (last column)."""
    data = [[Paragraph(h, s['th']) for h in ['#', 'Behavior Tested', 'Result']]]
    for row in rows:
        data.append([Paragraph(str(c), s['tc']) for c in row])
    tbl = Table(data, colWidths=col_widths)
    style = [
        ('BACKGROUND', (0, 0), (-1, 0), C_NAVY),
        ('ROWBACKGROUNDS', (0, 1), (-1, -1), [C_WHITE, C_LGREY]),
        ('GRID', (0, 0), (-1, -1), 0.4, C_MGREY),
        ('VALIGN', (0, 0), (-1, -1), 'TOP'),
        ('TOPPADDING', (0, 0), (-1, -1), 4),
        ('BOTTOMPADDING', (0, 0), (-1, -1), 4),
        ('LEFTPADDING', (0, 0), (-1, -1), 6),
        ('RIGHTPADDING', (0, 0), (-1, -1), 6),
        ('TEXTCOLOR', (2, 1), (2, -1), C_GREEN),
        ('FONTNAME', (2, 1), (2, -1), 'Helvetica-Bold'),
    ]
    tbl.setStyle(TableStyle(style))
    return tbl


def diagram_image(filename, caption, s, max_width_cm=17.0):
    path = os.path.join(IMG_DIR, filename)
    from PIL import Image as PILImage
    with PILImage.open(path) as im:
        w_px, h_px = im.size
    width = max_width_cm * cm
    height = width * (h_px / w_px)
    img = Image(path, width=width, height=height)
    return [img, Paragraph(caption, s['img_caption'])]


# ── Cover page ──────────────────────────────────────────────────────────────────

def cover_page(s):
    story = []
    story.append(SP(50))
    story.append(Paragraph('Team Juggernauts', s['subtitle']))
    story.append(SP(4))
    story.append(Paragraph('Custom Navigation Stack', s['title']))
    story.append(Paragraph('Design &amp; Zone Nav Decoupling', s['title']))
    story.append(SP(8))
    story.append(Paragraph(
        'A* Planner &middot; PD / Pure-Pursuit Controller &middot; Waypoint Sequencer',
        s['subtitle']))
    story.append(SP(40))
    story.append(HRFlowable(width='100%', thickness=1.5, color=C_BLUE, spaceAfter=12))
    story.append(Paragraph(
        'This guide covers: the custom A*+PD/pure-pursuit navigation architecture '
        '&middot; verified findings on why diy_zone_nav is already fully decoupled '
        'from it &middot; two real bugs found and fixed along the way &middot; the '
        'new diy_waypoint_sequencer package for automated goal sequencing, including '
        'its real functional test results.',
        s['cover_body']))
    story.append(SP(20))
    story.append(Paragraph(
        'Companion reference: <b>docs/reuse_plan_step1.md</b> Step 13 (the full '
        'investigation and verification detail behind this document).', s['cover_body']))
    story.append(PageBreak())
    return story


# ── Section 1 : Architecture overview ────────────────────────────────────────────

def section_architecture(s):
    story = []
    story.append(H('1&nbsp;&middot;&nbsp;Architecture Overview', 'h1', s))
    story.append(hr(s))
    story.append(P(
        'This robot uses <b>only nav2_map_server + nav2_lifecycle_manager</b> from '
        'the Nav2 ecosystem &mdash; map serving only, not the full Nav2 navigation '
        'system (no planner_server, controller_server, or costmap layers). Path '
        'planning and control are fully custom.', s))

    story.append(H('Planning Pipeline', 'h2', s))
    story += diagram_image(
        'custom_nav_planner_diagram.png',
        'Source: teammate-provided architecture diagram.', s)

    story.append(H('Control Pipeline', 'h2', s))
    story += diagram_image(
        'custom_nav_controller_diagram.png',
        'Source: teammate-provided architecture diagram.', s)

    story.append(tip_box(
        '/cmd_vel_nav is the SAME topic name cmd_vel_mux_node already reads for '
        'AUTONOMOUS mode &mdash; no mux changes were needed to plug this controller in.', s))

    story.append(PageBreak())
    return story


# ── Section 2 : Zone nav findings ────────────────────────────────────────────────

def section_zone_nav_findings(s):
    story = []
    story.append(H('2&nbsp;&middot;&nbsp;The Zone Nav Question &mdash; What Was Actually Verified', 'h1', s))
    story.append(hr(s))
    story.append(P(
        'The ask: design so diy_zone_nav can be removed entirely with no impact, '
        'since the plan is for the controller to eventually consume "info from the '
        'zone navigator." Investigated the real code before designing anything &mdash; '
        'did not assume.', s))

    rows = [
        ['Does diy_planning/diy_motion_planner reference /nav_mode, '
         '/speed_limit, or zone_nav anywhere?',
         'grep -rn across both packages',
         'Zero hits — no existing coupling at all'],
        ['Does the A* planner crash/hang with no /goal_pose?',
         "Read goal_callback's guard clauses directly",
         'No — early-return, idles safely'],
        ["Does cmd_vel_mux's AUTONOMOUS mode depend on zone_nav?",
         "Read cmd_vel_mux_node.py's mode logic",
         'No — reads /cmd_vel_nav unconditionally; BLIND_DRIVE only via '
         "zone_nav's own explicit service call"],
        ["Does zone_nav crash if Nav2's costmap/controller services "
         "don't exist?",
         'Read the service-call sites directly',
         'No — already checks service_is_ready() first, skips gracefully'],
    ]
    story.append(simple_table(
        ['Check', 'Method', 'Result'], rows,
        [6.0 * cm, 5.2 * cm, 5.8 * cm], s))

    story.append(SP(8))
    story.append(tip_box(
        'diy_zone_nav was ALREADY fully orphaned and safely removable in this '
        'architecture before any code was changed. Its /speed_limit output and '
        'costmap SetParameters calls have no consumer and no valid service target '
        'respectively, in a stack that only runs nav2_map_server. Nothing needed to '
        'be built to satisfy "can take zone navigator out" for the pipeline as it '
        'already existed.', s))

    story.append(PageBreak())
    return story


# ── Section 3 : Bugs found ────────────────────────────────────────────────────────

def section_bugs(s):
    story = []
    story.append(H('3&nbsp;&middot;&nbsp;Two Real Bugs Found and Fixed While Verifying This', 'h1', s))
    story.append(hr(s))

    story.append(H('Bug 1 &mdash; cmd_vel_topic Launch Default Mismatch', 'h2', s))
    story.append(P(
        'pd_navigation.launch.py and pure_pursuit_navigation.launch.py declared '
        'cmd_vel_topic defaulting to /cmd_vel (the simulation value), while their '
        'sibling arguments (base_frame, use_sim_time) already defaulted to hardware '
        'values (base_link, false).', s))
    story.append(warn_box(
        'A bare hardware launch with no override would silently publish velocity '
        'commands into /cmd_vel, which cmd_vel_mux never subscribes to &mdash; the '
        'robot simply would not move, with no error anywhere.', s))
    story.append(tip_box(
        'FIXED: both launch files now default cmd_vel_topic to /cmd_vel_nav, '
        'consistent with their other hardware defaults.', s))

    story.append(H('Bug 2 &mdash; No ROS Signal for "Goal Reached"', 'h2', s))
    story.append(P(
        'Neither pd_motion_planner_node.py nor pure_pursuit_motion_planner_node.py '
        'published anything when the robot reached its goal &mdash; only an internal '
        'log line. This blocks any future automated sequencing (nothing to react to).', s))
    story.append(tip_box(
        'FIXED: added a /pd/goal_reached (std_msgs/Bool) publisher to both '
        'controller nodes, published at the exact point each already detects goal '
        'completion &mdash; purely additive, no existing behavior changed.', s))

    story.append(PageBreak())
    return story


# ── Section 4 : waypoint sequencer ───────────────────────────────────────────────

def section_sequencer(s):
    story = []
    story.append(H('4&nbsp;&middot;&nbsp;New Package: diy_waypoint_sequencer', 'h1', s))
    story.append(hr(s))
    story.append(P(
        'Built to satisfy the actual open design question: something needs to '
        'auto-publish /goal_pose in sequence for a competition run where no human '
        'is clicking RViz goals.', s))

    story.append(H('Key Design Decision', 'h2', s))
    story.append(note_box(
        'Built as a brand-new, standalone, minimal package &mdash; NOT added inside '
        'diy_zone_nav, diy_planning, or diy_motion_planner. Putting it inside '
        'diy_zone_nav would have re-coupled "can I remove zone_nav" with "do I '
        'still get automated sequencing" &mdash; exactly the ambiguity this whole '
        'task was about avoiding.', s))

    story.append(H('How It Works', 'h2', s))
    story += B([
        'Loads a simple waypoints YAML (label, x, y, yaw) &mdash; deliberately NOT '
        "reusing zone_waypoints.yaml's much richer zone/radius/mode schema, which "
        "encodes Nav2-costmap concepts this simpler stack doesn't use.",
        'Waits for /green_light (same competition start-trigger topic convention '
        'already used by zone_nav) &mdash; or starts immediately after a configurable '
        'delay for bench testing.',
        'Publishes the current waypoint on /goal_pose.',
        'Advances to the next waypoint when /pd/goal_reached fires True.',
        'Optionally loops back to the first waypoint after the last (loop: true).',
    ], s)
    story.append(tip_box(
        'If this node is not run: /goal_pose is simply never auto-published &mdash; '
        'a human can still publish it manually (RViz "2D Goal Pose", or ros2 topic '
        'pub) with zero code changes anywhere else in the stack.', s))

    story.append(H('Verification &mdash; Real Functional Test Results', 'h2', s))
    story.append(P(
        'A real functional test (not just written and assumed correct) &mdash; '
        'tf_transformations is not installed in this sandbox and there is no sudo '
        'access to add it, so its one function was stubbed with equivalent pure '
        'math for the test only.', s))
    rows = [
        ['1', 'No goal published before /green_light', 'PASS'],
        ['2', 'First waypoint published correctly on green light', 'PASS'],
        ['3', '/pd/goal_reached advances to the next waypoint', 'PASS'],
        ['4', 'Sequence stops cleanly after the last waypoint (no loop)', 'PASS'],
        ['5', 'Duplicate /green_light after start is a no-op', 'PASS'],
        ['6', 'wait_for_green_light:=false auto-starts with no green light at all', 'PASS'],
        ['7', 'loop:=true wraps back to waypoint 0 after the last', 'PASS'],
    ]
    story.append(result_table(rows, s, [1.0 * cm, 12.5 * cm, 2.5 * cm]))
    story.append(SP(6))
    story.append(tip_box(
        'Also confirmed the package builds clean in a full 15-package workspace '
        'rebuild, and its launch file arguments resolve correctly.', s))

    story.append(PageBreak())
    return story


# ── Section 5 : Launch / test ─────────────────────────────────────────────────────

def section_launch(s):
    story = []
    story.append(H('5&nbsp;&middot;&nbsp;How to Launch / Test This Stack', 'h1', s))
    story.append(hr(s))

    story += code_block('Hardware, PD controller, with automated sequencing', '''\
ros2 launch diy_motion_planner pd_navigation.launch.py \\
    map_yaml:=/absolute/path/to/map.yaml

ros2 launch diy_waypoint_sequencer waypoint_sequencer.launch.py \\
    waypoints_file:=/absolute/path/to/waypoints.yaml''', s)

    story += code_block('Manual goal testing (no sequencer needed)', '''\
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \\
    "{header: {frame_id: map}, pose: {position: \\
    {x: 1.0, y: 1.0}, orientation: {w: 1.0}}}"''', s)

    story.append(note_box('diy_zone_nav is NOT required for any of the above to work.', s))

    story.append(H('Switching maps during testing', 'h2', s))
    story.append(P(
        'The course map (2D occupancy grid, .pgm/.yaml &mdash; feeds '
        'nav2_map_server &rarr; /map &rarr; the A* planner) is expected to '
        'change often as testing progresses. map_yaml in both '
        'pd_navigation.launch.py and pure_pursuit_navigation.launch.py now '
        'defaults to, in priority order: (1) the DIY_MAP_YAML env var if set, '
        'else (2) $DIY_ROS_WS/src/DIY-Challenge-Repo/maps/'
        'course_traced_smooth.yaml (the current final course map &mdash; '
        'hand-traced borders smoothed via preprocess_map.py; see '
        'docs/reuse_plan_step1.md Steps 19-24 for the full history).', s))

    story += code_block('Switching maps &mdash; no launch-file edits needed', '''\
# Once per test session, before launching anything:
export DIY_MAP_YAML=/absolute/path/to/new_map.yaml

# Or override for a single run without exporting anything:
ros2 launch diy_motion_planner pd_navigation.launch.py \\
    map_yaml:=/absolute/path/to/new_map.yaml''', s)

    story.append(note_box(
        'map_yaml (2D occupancy grid, nav2_map_server/A*) and map_pcd_path '
        '(3D point cloud, map_localizer&rsquo;s map&rarr;odom TF &mdash; see '
        'jetson_bringup_guide.md &sect;2/&sect;3) are two separate, unrelated '
        'maps. A .pgm/.yaml pair cannot be used as a map_pcd_path value or '
        'vice versa.', s))

    story.append(H('Generating real waypoints for controller testing', 'h2', s))
    story.append(P(
        'diy_waypoint_sequencer/config/waypoints.yaml ships with placeholder '
        'x/y/yaw values only meant to document the schema &mdash; not real, '
        'map-valid coordinates. scripts/pick_waypoints.py is a click-to-pick '
        'tool (same pattern as scripts/register_zones_to_map.py&rsquo;s --pick '
        'mode) that generates a real waypoints.yaml by clicking points '
        'directly on the actual map you are testing against.', s))

    story += code_block('Pick waypoints on the real test map', '''\
python3 scripts/pick_waypoints.py \\
    --map maps/course_traced_smooth.yaml \\
    --output maps/test_waypoints.yaml \\
    --overlay maps/test_waypoints_preview.png''', s)

    story.append(P(
        'Click waypoints in the order the robot should visit them; each '
        'click is checked against the map&rsquo;s own occupancy data '
        'immediately (a click on an occupied/unknown pixel prints a warning '
        'right away &mdash; press \u2018u\u2019 to undo and click again). '
        'Heading (yaw) is computed automatically as the bearing toward the '
        'next waypoint &mdash; override with --uniform-yaw &lt;radians&gt; '
        'for a fixed heading everywhere. --overlay saves a PNG with numbered '
        'markers + heading arrows to sanity-check the path before running '
        'it on the robot.', s))

    story.append(note_box(
        'The tool also prints a ready-to-copy initial_x/initial_y/'
        'initial_yaw override matching wp1 exactly. map_localizer&rsquo;s '
        'own relocalization initial pose defaults to (0,0,0) &mdash; i.e. '
        'it assumes the robot starts at the MAP FRAME&rsquo;s own origin, '
        'NOT at wp1. If you actually place the robot at wp1&rsquo;s '
        'real-world spot before starting, pass that printed override to '
        'localization.launch.py, or VGICP relocalization has to converge '
        'from however far (0,0) actually is from wp1 (potentially several '
        'metres) &mdash; risking a failed or wrong convergence instead of a '
        'clean one.', s))

    story += code_block('Run the picked waypoints', '''\
ros2 launch diy_waypoint_sequencer waypoint_sequencer.launch.py \\
    waypoints_file:=maps/test_waypoints.yaml''', s)

    story.append(H('6&nbsp;&middot;&nbsp;Still Open', 'h2', s))
    story += B([
        'The custom controller stack is not yet wired into '
        'challenge_master.launch.py &mdash; currently launched standalone only. '
        'Master-launch integration is a separate, larger decision not made in '
        'this pass.',
        'Whether diy_zone_nav should eventually be deleted outright (vs. kept '
        'orphaned-but-present) was not decided &mdash; no urgency since it is '
        'already proven harmless to leave in place.',
        'tf_transformations is not installed in this sandbox (no sudo) &mdash; '
        'affects verifying this and the pre-existing diy_motion_planner package '
        'identically; not a new gap introduced by this work.',
    ], s)

    story.append(SP(20))
    story.append(hr(s))
    story.append(P(
        '<i>Full investigation and verification detail: '
        'docs/reuse_plan_step1.md Step 13.</i>', s))
    return story


# ── Page decoration ───────────────────────────────────────────────────────────────

PAGE_W, PAGE_H = A4


def draw_cover_background(canvas, doc):
    canvas.saveState()
    canvas.setFillColor(C_NAVY)
    canvas.rect(0, 0, PAGE_W, PAGE_H, fill=1, stroke=0)
    canvas.setFillColor(C_BLUE)
    canvas.rect(0, PAGE_H - 0.4 * cm, PAGE_W, 0.4 * cm, fill=1, stroke=0)
    canvas.restoreState()


def draw_content_page(canvas, doc):
    canvas.saveState()
    canvas.setFillColor(C_NAVY)
    canvas.rect(0, PAGE_H - 0.3 * cm, PAGE_W, 0.3 * cm, fill=1, stroke=0)
    canvas.setFont('Helvetica', 8)
    canvas.setFillColor(C_MGREY)
    canvas.drawString(2 * cm, 1.3 * cm, 'Custom Navigation Stack Design \u2014 Team Juggernauts 2026')
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
        title='Custom Navigation Stack Design — Team Juggernauts 2026',
        author='Team Juggernauts',
        subject='DIY Robot Challenge 2026 — Custom A*/PD Nav Stack & Zone Nav Decoupling',
    )

    s = make_styles()
    story = []
    story += cover_page(s)
    story += section_architecture(s)
    story += section_zone_nav_findings(s)
    story += section_bugs(s)
    story += section_sequencer(s)
    story += section_launch(s)

    doc.build(story, onFirstPage=draw_cover_background, onLaterPages=draw_content_page)
    size_kb = os.path.getsize(OUT_PATH) // 1024
    print(f"Generated: {OUT_PATH}  ({size_kb} KB)")


if __name__ == '__main__':
    build_pdf()
