#!/usr/bin/env python3
"""
generate_zone_nav_design.py
Generates docs/Zone_Nav_Design_Guide.html + .pdf
Zone-aware navigation design document — Team Juggernauts 2026
"""

import subprocess
import textwrap
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
OUT_HTML  = REPO_ROOT / "docs" / "Zone_Nav_Design_Guide.html"
OUT_PDF   = REPO_ROOT / "docs" / "Zone_Nav_Design_Guide.pdf"


# ─────────────────────────────────────────────────────────────────────────────
# Graphviz helpers
# ─────────────────────────────────────────────────────────────────────────────
def dot_to_svg(dot_src: str) -> str:
    result = subprocess.run(
        ["dot", "-Tsvg"],
        input=dot_src.encode(),
        capture_output=True,
    )
    svg = result.stdout.decode()
    # Strip everything before <svg — removes XML declaration, DOCTYPE, and DTD URL lines
    idx = svg.find("<svg")
    return svg[idx:] if idx != -1 else svg


# ─────────────────────────────────────────────────────────────────────────────
# Diagram 1 — Node & topic architecture (Graphviz)
# ─────────────────────────────────────────────────────────────────────────────
DOT_ARCHITECTURE = """
digraph zone_nav_arch {
  graph [bgcolor="#0f172a" fontname="Inter" rankdir=LR splines=ortho nodesep=0.6 ranksep=1.1]
  node  [fontname="Inter" fontsize=9 style="filled,rounded" shape=box penwidth=1.5]
  edge  [fontname="Inter" fontsize=8 color="#475569" fontcolor="#94a3b8"]

  green_light_pub  [label="green_light_pub"              fillcolor="#0c1a2e" fontcolor="#22c55e"  color="#22c55e"]
  zone_nav_manager [label="zone_nav_manager"             fillcolor="#172554" fontcolor="#93c5fd"  color="#3b82f6"]
  lidar_odom_gate  [label="lidar_odom_gate"              fillcolor="#172554" fontcolor="#93c5fd"  color="#3b82f6"]
  cmd_vel_mux      [label="cmd_vel_mux"                  fillcolor="#1c1917" fontcolor="#d6d3d1"  color="#78716c"]
  controller_server[label="controller_server\\n(MPPI)"   fillcolor="#431407" fontcolor="#fed7aa"  color="#f97316"]
  ekf2             [label="EKF2\\n/odometry/filtered"    fillcolor="#14532d" fontcolor="#86efac"  color="#22c55e"]
  motors           [label="STM32 Motors"                 fillcolor="#1c1917" fontcolor="#d6d3d1"  color="#78716c"]
  fastlio2         [label="FAST-LIO2\\n/lidar_odometry"  fillcolor="#1e1b4b" fontcolor="#c4b5fd"  color="#a855f7"]

  nav_mode         [label="/nav_mode"              shape=ellipse fillcolor="#0c1a2e" fontcolor="#0ea5e9"  color="#0ea5e9"]
  speed_limit      [label="/speed_limit"           shape=ellipse fillcolor="#0c1a2e" fontcolor="#f97316"  color="#f97316"]
  cmd_vel_zone_nav [label="/cmd_vel_zone_nav"      shape=ellipse fillcolor="#0c1a2e" fontcolor="#94a3b8"  color="#334155"]
  cmd_vel_safe     [label="/cmd_vel_safe"          shape=ellipse fillcolor="#0c1a2e" fontcolor="#22c55e"  color="#22c55e"]
  lidar_gated      [label="/lidar_odometry_gated"  shape=ellipse fillcolor="#0c1a2e" fontcolor="#a855f7"  color="#a855f7"]
  mux_mode         [label="/mux_mode"              shape=ellipse fillcolor="#0c1a2e" fontcolor="#eab308"  color="#eab308"]
  odom_filtered    [label="/odometry/filtered"     shape=ellipse fillcolor="#0c1a2e" fontcolor="#22c55e"  color="#22c55e"]
  ndt_fitness      [label="/ndt_fitness_score"     shape=ellipse fillcolor="#0c1a2e" fontcolor="#94a3b8"  color="#334155"]
  estop_active     [label="/estop_active"          shape=ellipse fillcolor="#0c1a2e" fontcolor="#ef4444"  color="#ef4444"]
  cmd_vel_joy      [label="/cmd_vel_joy"           shape=ellipse fillcolor="#0c1a2e" fontcolor="#94a3b8"  color="#334155"]
  cmd_vel_nav      [label="/cmd_vel_nav"           shape=ellipse fillcolor="#0c1a2e" fontcolor="#94a3b8"  color="#334155"]

  green_light_pub  -> zone_nav_manager
  zone_nav_manager -> nav_mode
  nav_mode         -> lidar_odom_gate
  lidar_odom_gate  -> lidar_gated
  lidar_gated      -> ekf2
  fastlio2         -> lidar_odom_gate [label="/lidar_odometry"]

  zone_nav_manager -> speed_limit        [color="#f97316" fontcolor="#f97316"]
  speed_limit      -> controller_server
  zone_nav_manager -> cmd_vel_zone_nav
  cmd_vel_zone_nav -> cmd_vel_mux
  cmd_vel_mux      -> cmd_vel_safe
  cmd_vel_safe     -> motors

  odom_filtered    -> zone_nav_manager  [color="#22c55e" fontcolor="#22c55e"]
  ndt_fitness      -> zone_nav_manager
  mux_mode         -> zone_nav_manager  [label="feedback" color="#eab308" fontcolor="#eab308"]
  estop_active     -> cmd_vel_mux       [color="#ef4444" fontcolor="#ef4444"]
  cmd_vel_joy      -> cmd_vel_mux
  cmd_vel_nav      -> cmd_vel_mux
}
"""

# ─────────────────────────────────────────────────────────────────────────────
# Diagram 2 — Zone state machine (Graphviz)
# ─────────────────────────────────────────────────────────────────────────────
DOT_STATE_MACHINE = """
digraph state_machine {
  graph [bgcolor="#0f172a" fontname="Inter" rankdir=LR splines=spline nodesep=0.9 ranksep=1.6]
  node  [fontname="Inter" fontsize=10 style="filled,rounded" shape=box penwidth=1.5 width=2.0 height=0.8]
  edge  [fontname="Inter" fontsize=8]

  START    [label="START\\nDetect green light"         fillcolor="#1e293b" fontcolor="#94a3b8" color="#475569"]
  NORMAL   [label="NORMAL_NAV\\nNav2 Smac+MPPI\\nFull speed" fillcolor="#14532d" fontcolor="#86efac" color="#22c55e"]
  SLOW     [label="SLOW_NAV\\nReduced speed\\n0.2 m/s" fillcolor="#164e63" fontcolor="#a5f3fc" color="#06b6d4"]
  BLIND    [label="BLIND_DRIVE\\nWheel+IMU only\\nFixed heading" fillcolor="#450a0a" fontcolor="#fca5a5" color="#ef4444"]
  NAVIGATE [label="NAVIGATE_AROUND\\nLive costmap\\nDynamic replan" fillcolor="#1e1b4b" fontcolor="#c4b5fd" color="#a855f7"]
  PUSH     [label="PUSH_THROUGH\\nStraight drive\\nObstacle layer OFF" fillcolor="#422006" fontcolor="#fdba74" color="#f97316"]
  DONE     [label="DONE\\nStop at finish\\n(lap 2 complete)" fillcolor="#1e293b" fontcolor="#94a3b8" color="#475569"]

  { rank=same; SLOW; BLIND; NAVIGATE; PUSH; DONE }

  START    -> NORMAL   [label="green light detected"  color="#22c55e"  fontcolor="#86efac"]
  NORMAL   -> SLOW     [label="narrow/ramp/bank"      color="#06b6d4"  fontcolor="#a5f3fc"]
  NORMAL   -> BLIND    [label="tunnel entry"          color="#ef4444"  fontcolor="#fca5a5"]
  NORMAL   -> NAVIGATE [label="bucket zone"           color="#a855f7"  fontcolor="#c4b5fd"]
  NORMAL   -> PUSH     [label="car wash zone"         color="#f97316"  fontcolor="#fdba74"]
  NORMAL   -> DONE     [label="lap 2 at finish"       color="#475569"  fontcolor="#94a3b8"]
  SLOW     -> NORMAL   [label="zone exit"             color="#06b6d4"  fontcolor="#a5f3fc"]
  BLIND    -> NORMAL   [label="lidar restored"        color="#ef4444"  fontcolor="#fca5a5"]
  NAVIGATE -> NORMAL   [label="cleared"               color="#a855f7"  fontcolor="#c4b5fd"]
  PUSH     -> NORMAL   [label="zone exit"             color="#f97316"  fontcolor="#fdba74"]
}
"""

# ─────────────────────────────────────────────────────────────────────────────
# Pre-render Graphviz diagrams
# ─────────────────────────────────────────────────────────────────────────────
svg_architecture  = dot_to_svg(DOT_ARCHITECTURE)
svg_state_machine = dot_to_svg(DOT_STATE_MACHINE)


# ─────────────────────────────────────────────────────────────────────────────
# HTML Document
# ─────────────────────────────────────────────────────────────────────────────
CSS = """
@import url('https://fonts.googleapis.com/css2?family=Inter:wght@300;400;500;600;700&family=JetBrains+Mono:wght@400;500&display=swap');

:root {
  --cyan:    #0ea5e9;
  --green:   #22c55e;
  --purple:  #a855f7;
  --orange:  #f97316;
  --yellow:  #eab308;
  --red:     #ef4444;
  --blue:    #6366f1;
  --lime:    #84cc16;
  --dark:    #0f172a;
  --card:    #1e293b;
  --border:  #334155;
  --muted:   #64748b;
  --text:    #e2e8f0;
  --subtext: #94a3b8;
}

@page { size: A4; margin: 20mm 18mm 20mm 18mm; }
* { box-sizing: border-box; margin: 0; padding: 0; }
body { font-family: 'Inter', 'Segoe UI', sans-serif; background: var(--dark); color: var(--text); font-size: 11px; line-height: 1.6; }

.cover { page-break-after: always; min-height: 100vh; display: flex; flex-direction: column; justify-content: center; align-items: center; text-align: center; padding: 60px 40px; background: linear-gradient(160deg, #0f172a 0%, #0c1a2e 50%, #130f1a 100%); }
.cover-badge { background: rgba(14,165,233,0.12); border: 1px solid var(--cyan); color: var(--cyan); padding: 5px 18px; border-radius: 20px; font-size: 10px; font-weight: 600; letter-spacing: 2px; text-transform: uppercase; margin-bottom: 28px; }
.cover h1 { font-size: 34px; font-weight: 700; color: #fff; letter-spacing: -1px; line-height: 1.2; margin-bottom: 14px; }
.cover h1 span { color: var(--cyan); }
.cover .subtitle { font-size: 13px; color: var(--subtext); max-width: 580px; margin-bottom: 36px; }
.cover-meta { display: grid; grid-template-columns: repeat(4,1fr); gap: 14px; max-width: 680px; width: 100%; margin-bottom: 36px; }
.cover-meta-item { background: rgba(255,255,255,0.04); border: 1px solid var(--border); border-radius: 8px; padding: 12px; }
.cover-meta-item .label { font-size: 9px; color: var(--muted); text-transform: uppercase; letter-spacing: 1px; margin-bottom: 3px; }
.cover-meta-item .value { font-size: 11px; font-weight: 600; color: var(--text); }
.cover-stack { display: flex; gap: 8px; flex-wrap: wrap; justify-content: center; }

.tag { padding: 3px 10px; border-radius: 5px; font-size: 10px; font-weight: 500; }
.tag-cyan   { background: rgba(14,165,233,0.15); color: var(--cyan);   border: 1px solid rgba(14,165,233,0.3); }
.tag-green  { background: rgba(34,197,94,0.15);  color: var(--green);  border: 1px solid rgba(34,197,94,0.3); }
.tag-purple { background: rgba(168,85,247,0.15); color: var(--purple); border: 1px solid rgba(168,85,247,0.3); }
.tag-orange { background: rgba(249,115,22,0.15); color: var(--orange); border: 1px solid rgba(249,115,22,0.3); }
.tag-yellow { background: rgba(234,179,8,0.15);  color: var(--yellow); border: 1px solid rgba(234,179,8,0.3); }
.tag-red    { background: rgba(239,68,68,0.15);  color: var(--red);    border: 1px solid rgba(239,68,68,0.3); }
.tag-blue   { background: rgba(99,102,241,0.15); color: var(--blue);   border: 1px solid rgba(99,102,241,0.3); }

.page { max-width: 900px; margin: 0 auto; padding: 32px 24px; }
.pagebreak { page-break-before: always; }
.section { margin-bottom: 36px; }
.section-header { display: flex; align-items: center; gap: 12px; margin-bottom: 20px; padding-bottom: 10px; border-bottom: 1px solid var(--border); }
.section-num { background: var(--cyan); color: var(--dark); font-size: 10px; font-weight: 700; padding: 3px 9px; border-radius: 5px; min-width: 32px; text-align: center; }
.section-num.green  { background: var(--green); }
.section-num.purple { background: var(--purple); }
.section-num.orange { background: var(--orange); }
.section-num.yellow { background: var(--yellow); color: #0f172a; }
.section-num.red    { background: var(--red); }
.section h2 { font-size: 18px; font-weight: 700; color: #fff; }
h3 { font-size: 14px; font-weight: 600; color: var(--text); margin: 20px 0 8px; }
h4 { font-size: 12px; font-weight: 600; color: var(--subtext); margin: 14px 0 6px; text-transform: uppercase; letter-spacing: 0.5px; }
p { color: var(--subtext); margin-bottom: 10px; }
p strong { color: var(--text); font-weight: 600; }

.card { background: var(--card); border: 1px solid var(--border); border-radius: 8px; padding: 16px; margin-bottom: 12px; position: relative; }
.card-title { font-size: 12px; font-weight: 600; color: #fff; margin-bottom: 6px; display: flex; align-items: center; gap: 7px; }
.copy-btn { margin-left: auto; background: rgba(255,255,255,0.06); border: 1px solid var(--border); border-radius: 4px; color: var(--muted); font-size: 9px; padding: 2px 7px; cursor: pointer; flex-shrink: 0; transition: background 0.15s, color 0.15s; }
.copy-btn:hover { background: rgba(255,255,255,0.12); color: #fff; }
.dot { width: 7px; height: 7px; border-radius: 50%; flex-shrink: 0; }
.dot-cyan   { background: var(--cyan); }
.dot-green  { background: var(--green); }
.dot-purple { background: var(--purple); }
.dot-orange { background: var(--orange); }
.dot-red    { background: var(--red); }
.dot-blue   { background: var(--blue); }
.dot-yellow { background: var(--yellow); }

table { width: 100%; border-collapse: collapse; margin: 12px 0; font-size: 10px; }
thead tr { background: rgba(255,255,255,0.06); }
th { padding: 8px 10px; text-align: left; font-weight: 600; color: var(--text); border-bottom: 1px solid var(--border); font-size: 9px; text-transform: uppercase; letter-spacing: 0.5px; }
td { padding: 7px 10px; color: var(--subtext); border-bottom: 1px solid rgba(255,255,255,0.04); vertical-align: top; }
td strong { color: var(--text); }
td code { font-family: 'JetBrains Mono', monospace; font-size: 9px; background: rgba(255,255,255,0.08); padding: 1px 4px; border-radius: 3px; color: var(--cyan); }

pre { background: #0a0e1a; border: 1px solid var(--border); border-left: 3px solid var(--cyan); border-radius: 6px; padding: 12px 16px; font-family: 'JetBrains Mono', monospace; font-size: 9.5px; color: #a0c4ff; overflow-x: auto; margin: 10px 0; line-height: 1.6; white-space: pre; }

.callout { border-radius: 6px; padding: 12px 14px; margin: 12px 0; font-size: 10.5px; border-left: 3px solid; }
.callout-warn  { background: rgba(234,179,8,0.08);  border-color: var(--yellow); color: #fde68a; }
.callout-info  { background: rgba(14,165,233,0.08); border-color: var(--cyan);   color: #bae6fd; }
.callout-crit  { background: rgba(239,68,68,0.08);  border-color: var(--red);    color: #fecaca; }
.callout-good  { background: rgba(34,197,94,0.08);  border-color: var(--green);  color: #bbf7d0; }
.callout .callout-title { font-weight: 700; margin-bottom: 3px; font-size: 10px; text-transform: uppercase; letter-spacing: 0.8px; }

.grid2 { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; margin: 12px 0; }
.grid3 { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 12px; margin: 12px 0; }

@media print {
  .grid2 { grid-template-columns: 1fr; }
  .grid3 { grid-template-columns: 1fr; }
  .copy-btn { display: none; }
}

.diagram-container { background: rgba(255,255,255,0.02); border: 1px solid var(--border); border-radius: 8px; padding: 16px; margin: 14px 0; text-align: center; overflow: hidden; }
.diagram-container svg { max-width: 100%; height: auto; }
.diagram-label { font-size: 9px; color: var(--muted); text-transform: uppercase; letter-spacing: 1px; margin-bottom: 8px; }

.divider { border: none; border-top: 1px solid var(--border); margin: 28px 0; }

.toc { page-break-after: always; }
.toc h2 { font-size: 20px; font-weight: 700; color: #fff; margin-bottom: 20px; padding-bottom: 10px; border-bottom: 1px solid var(--border); }
.toc-section { margin-bottom: 6px; }
.toc-section a { color: var(--subtext); text-decoration: none; display: flex; justify-content: space-between; padding: 4px 8px; border-radius: 4px; font-size: 11px; }
.toc-section .num { color: var(--cyan); font-weight: 600; margin-right: 8px; font-size: 10px; }
.toc-sub { padding-left: 24px; }
.toc-sub a { font-size: 10px; }

.zone-badge { display: inline-block; padding: 2px 8px; border-radius: 4px; font-size: 9px; font-weight: 700; text-transform: uppercase; letter-spacing: 0.5px; margin-right: 4px; }
.zone-normal  { background: rgba(34,197,94,0.2);  color: var(--green);  border: 1px solid rgba(34,197,94,0.4); }
.zone-slow    { background: rgba(6,182,212,0.2);   color: #22d3ee;       border: 1px solid rgba(6,182,212,0.4); }
.zone-blind   { background: rgba(239,68,68,0.2);   color: var(--red);    border: 1px solid rgba(239,68,68,0.4); }
.zone-around  { background: rgba(168,85,247,0.2);  color: var(--purple); border: 1px solid rgba(168,85,247,0.4); }
.zone-push    { background: rgba(249,115,22,0.2);  color: var(--orange); border: 1px solid rgba(249,115,22,0.4); }
"""

HTML = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8"/>
<title>Zone-Aware Navigation — Software Design Guide — DIY Robot Challenge 2026</title>
<style>{CSS}</style>
</head>
<body>

<!-- ═══ COVER ═══ -->
<div class="cover">
  <div class="cover-badge">Zone Nav Software Design Guide v1.0</div>
  <h1>Zone-Aware Navigation<br/><span>Software Design Guide</span></h1>
  <p class="subtitle">
    diy_zone_nav &bull; cmd_vel_mux &bull; lidar_odom_gate<br/>
    Distilled from 46 bugs — hard-won implementation knowledge for the 2026 DIY Robot Challenge.
  </p>
  <div class="cover-meta">
    <div class="cover-meta-item">
      <div class="label">Competition</div>
      <div class="value">October 1, 2026</div>
    </div>
    <div class="cover-meta-item">
      <div class="label">Packages</div>
      <div class="value">diy_zone_nav</div>
    </div>
    <div class="cover-meta-item">
      <div class="label">Platform</div>
      <div class="value">Jetson Orin Nano</div>
    </div>
    <div class="cover-meta-item">
      <div class="label">Team</div>
      <div class="value">Juggernauts 2026</div>
    </div>
  </div>
  <div class="cover-stack">
    <span class="tag tag-cyan">ROS 2 Humble</span>
    <span class="tag tag-green">zone_nav_manager</span>
    <span class="tag tag-purple">cmd_vel_mux</span>
    <span class="tag tag-orange">lidar_odom_gate</span>
    <span class="tag tag-yellow">7-State Machine</span>
    <span class="tag tag-red">46 Bugs Documented</span>
    <span class="tag tag-blue">zone_waypoints.yaml</span>
  </div>
</div>

<!-- ═══ TOC ═══ -->
<div class="page toc">
  <h2>Table of Contents</h2>
  <div class="toc-section"><a href="#s1"><span><span class="num">1</span> System Architecture</span></a></div>
  <div class="toc-section"><a href="#s2"><span><span class="num">2</span> State Machine Reference</span></a></div>
  <div class="toc-section"><a href="#s3"><span><span class="num">3</span> Topic &amp; Service Contracts</span></a></div>
  <div class="toc-section toc-sub"><a href="#s3-1"><span>3.1 zone_nav_manager Publishers</span></a></div>
  <div class="toc-section toc-sub"><a href="#s3-2"><span>3.2 zone_nav_manager Subscribers</span></a></div>
  <div class="toc-section toc-sub"><a href="#s3-3"><span>3.3 cmd_vel_mux Publishers / Subscribers</span></a></div>
  <div class="toc-section toc-sub"><a href="#s3-4"><span>3.4 SetParameters Services</span></a></div>
  <div class="toc-section"><a href="#s4"><span><span class="num">4</span> Parameter Reference</span></a></div>
  <div class="toc-section"><a href="#s5"><span><span class="num">5</span> zone_waypoints.yaml Format</span></a></div>
  <div class="toc-section"><a href="#s6"><span><span class="num">6</span> Key Implementation Gotchas</span></a></div>
  <div class="toc-section toc-sub"><a href="#s6-1"><span>6.1 Speed Control API</span></a></div>
  <div class="toc-section toc-sub"><a href="#s6-2"><span>6.2 Costmap Service Paths — The Doubled Namespace</span></a></div>
  <div class="toc-section toc-sub"><a href="#s6-3"><span>6.3 VoxelLayer Limitations</span></a></div>
  <div class="toc-section toc-sub"><a href="#s6-4"><span>6.4 ESTOP_LOCK is Hardware-Only</span></a></div>
  <div class="toc-section toc-sub"><a href="#s6-5"><span>6.5 BLIND_DRIVE Must-Knows</span></a></div>
  <div class="toc-section toc-sub"><a href="#s6-6"><span>6.6 zone_waypoints.yaml Pitfalls</span></a></div>
  <div class="toc-section toc-sub"><a href="#s6-7"><span>6.7 lidar_odom_gate Must Always Run with use_localization</span></a></div>
  <div class="toc-section"><a href="#s7"><span><span class="num">7</span> Debugging Checklist</span></a></div>
  <div class="toc-section"><a href="#s8"><span><span class="num">8</span> Scripts Reference</span></a></div>
</div>

<!-- ═══ SECTION 1 ═══ -->
<div class="page section" id="s1">
  <div class="section-header">
    <span class="section-num">1</span>
    <h2>System Architecture</h2>
  </div>

  <p>
    The zone navigation stack centres on three cooperating nodes. <strong>zone_nav_manager</strong> monitors
    robot position and NDT fitness, drives the 7-state machine, and publishes control signals.
    <strong>cmd_vel_mux</strong> arbitrates between joystick, Nav2, and blind-drive velocity sources.
    <strong>lidar_odom_gate</strong> gates the FAST-LIO2 odometry stream to EKF2, suppressing lidar
    during BLIND_DRIVE to prevent drift injection.
  </p>

  <div class="diagram-container">
    <div class="diagram-label">Full Node &amp; Topic Data Flow — Zone Navigation Stack</div>
    {svg_architecture}
  </div>

  <div class="grid3">
    <div class="card">
      <div class="card-title"><span class="dot dot-cyan"></span>zone_nav_manager</div>
      <p>The brain. Subscribes to position (<code>/odometry/filtered</code>) and NDT fitness score.
      Checks each registered waypoint zone on every timer tick. Publishes <code>/nav_mode</code>,
      <code>/speed_limit</code>, and sets costmap + mux parameters via service calls.</p>
    </div>
    <div class="card">
      <div class="card-title"><span class="dot dot-orange"></span>cmd_vel_mux</div>
      <p>Velocity arbitrator. Accepts <code>/cmd_vel_joy</code>, <code>/cmd_vel_nav</code>,
      and <code>/cmd_vel_zone_nav</code>. Selects which source reaches <code>/cmd_vel_safe</code>
      based on mode and <code>/estop_active</code>. ESTOP_LOCK overrides all sources.</p>
    </div>
    <div class="card">
      <div class="card-title"><span class="dot dot-purple"></span>lidar_odom_gate</div>
      <p>Inline gate on <code>/lidar_odometry</code> (FAST-LIO2). Passes through when
      <code>/nav_mode</code> ≠ BLIND_DRIVE; blocks when BLIND_DRIVE is active. Prevents corrupt
      lidar odometry from poisoning EKF2 inside the tunnel. 5s watchdog re-opens on manager crash.</p>
    </div>
  </div>

  <div class="callout callout-info">
    <div class="callout-title">Key Dependency Chain</div>
    green_light_pub → zone_nav_manager → /nav_mode → lidar_odom_gate → EKF2 → /odometry/filtered → zone_nav_manager (position feedback).
    Any break in this chain stalls the whole stack. The debugging checklist in Section 7 walks each link.
  </div>
</div>

<!-- ═══ SECTION 2 ═══ -->
<div class="page section pagebreak" id="s2">
  <div class="section-header">
    <span class="section-num green">2</span>
    <h2>State Machine Reference</h2>
  </div>

  <p>
    The zone_nav_manager implements a 7-state machine. Transitions are triggered by zone entry
    (distance from waypoint &lt; <code>radius</code>) and exit (distance &gt; <code>exit_radius</code>).
    DONE is triggered only after lap 2 is complete.
  </p>

  <div class="diagram-container">
    <div class="diagram-label">Zone Navigation State Machine — Mode Transitions</div>
    {svg_state_machine}
  </div>

  <table>
    <thead>
      <tr>
        <th>Mode</th>
        <th>Trigger Condition</th>
        <th>Entry Actions</th>
        <th>Exit Trigger</th>
        <th>Exit Actions</th>
        <th>Speed Limit</th>
        <th>Mux Mode</th>
      </tr>
    </thead>
    <tbody>
      <tr>
        <td><strong>INIT</strong></td>
        <td>Node startup, before green light</td>
        <td>Gate open (pass-through). All params at defaults.</td>
        <td>Green light detected on <code>/green_light</code></td>
        <td>Transition to NORMAL_NAV</td>
        <td>No limit</td>
        <td>JOYSTICK</td>
      </tr>
      <tr>
        <td><strong><span class="zone-badge zone-normal">NORMAL_NAV</span></strong></td>
        <td>Zone exit from any mode, or green light</td>
        <td>Restore full speed. Enable obstacle layers. Set mux AUTONOMOUS.</td>
        <td>Entry into a registered zone</td>
        <td>Apply zone-specific mode</td>
        <td>Configured max</td>
        <td>AUTONOMOUS</td>
      </tr>
      <tr>
        <td><strong><span class="zone-badge zone-slow">SLOW_NAV</span></strong></td>
        <td>Narrow / ramp / bank zone entry</td>
        <td>Publish <code>speed_limit=0.2</code>. Costmap unchanged.</td>
        <td>Distance &gt; exit_radius</td>
        <td>Restore speed limit to 0.0 (=no limit)</td>
        <td>0.2 m/s</td>
        <td>AUTONOMOUS</td>
      </tr>
      <tr>
        <td><strong><span class="zone-badge zone-blind">BLIND_DRIVE</span></strong></td>
        <td>Tunnel entry zone</td>
        <td>Set mux BLIND_DRIVE. Gate closes (blocks lidar to EKF2). Reset fitness to 999. 15s hard timeout armed.</td>
        <td>NDT fitness &lt; 0.8 after gate re-open, OR 15s timeout</td>
        <td>Re-open gate. Restore mux AUTONOMOUS.</td>
        <td>0.15 m/s fixed</td>
        <td>BLIND_DRIVE</td>
      </tr>
      <tr>
        <td><strong><span class="zone-badge zone-around">NAVIGATE_AROUND</span></strong></td>
        <td>Bucket zone entry</td>
        <td>Enable obstacle_layer + voxel_layer. Full costmap active.</td>
        <td>Distance &gt; exit_radius</td>
        <td>Restore NORMAL_NAV</td>
        <td>0.3 m/s</td>
        <td>AUTONOMOUS</td>
      </tr>
      <tr>
        <td><strong><span class="zone-badge zone-push">PUSH_THROUGH</span></strong></td>
        <td>Car wash zone entry</td>
        <td>Disable obstacle_layer.enabled AND voxel_layer.enabled. Straight heading cmd_vel.</td>
        <td>Distance &gt; exit_radius</td>
        <td>Re-enable both layers. Restore NORMAL_NAV.</td>
        <td>0.25 m/s</td>
        <td>BLIND_DRIVE</td>
      </tr>
      <tr>
        <td><strong>DONE</strong></td>
        <td>Finish line zone, lap count == total_laps</td>
        <td>Publish zero cmd_vel. Set mux JOYSTICK. Log completion time.</td>
        <td>N/A — terminal state</td>
        <td>N/A</td>
        <td>0.0 (stop)</td>
        <td>JOYSTICK</td>
      </tr>
    </tbody>
  </table>

  <div class="callout callout-warn">
    <div class="callout-title">INIT keeps the gate open</div>
    When zone_nav_manager is disabled or not yet started, the gate defaults to INIT (open).
    EKF2 receives full lidar odometry. This is intentional — safe pass-through for non-competition runs.
  </div>
</div>

<!-- ═══ SECTION 3 ═══ -->
<div class="page section pagebreak" id="s3">
  <div class="section-header">
    <span class="section-num purple">3</span>
    <h2>Topic &amp; Service Contracts</h2>
  </div>

  <h3 id="s3-1">3.1 zone_nav_manager — Publishers</h3>
  <table>
    <thead>
      <tr><th>Topic</th><th>Type</th><th>QoS</th><th>Purpose</th></tr>
    </thead>
    <tbody>
      <tr>
        <td><code>/nav_mode</code></td>
        <td><code>std_msgs/String</code></td>
        <td>Volatile, depth 10</td>
        <td>Current state machine mode. Read by lidar_odom_gate and all debug tools.</td>
      </tr>
      <tr>
        <td><code>/speed_limit</code></td>
        <td><code>std_msgs/Float32</code></td>
        <td>Volatile, depth 1</td>
        <td>Speed limit to controller_server. Re-sent every 1s (reassertSpeedLimit). 0.0 = NO_LIMIT.</td>
      </tr>
      <tr>
        <td><code>/cmd_vel_zone_nav</code></td>
        <td><code>geometry_msgs/Twist</code></td>
        <td>Volatile, depth 1</td>
        <td>Direct velocity output during BLIND_DRIVE and PUSH_THROUGH modes.</td>
      </tr>
    </tbody>
  </table>

  <h3 id="s3-2">3.2 zone_nav_manager — Subscribers</h3>
  <table>
    <thead>
      <tr><th>Topic</th><th>Type</th><th>Purpose</th></tr>
    </thead>
    <tbody>
      <tr>
        <td><code>/odometry/filtered</code></td>
        <td><code>nav_msgs/Odometry</code></td>
        <td>Robot pose used for zone entry/exit distance checks.</td>
      </tr>
      <tr>
        <td><code>/ndt_fitness_score</code></td>
        <td><code>std_msgs/Float32</code></td>
        <td>NDT scan-match quality. &gt;0.8 = poor (tunnel). &lt;0.8 = localized. Triggers BLIND_DRIVE exit.</td>
      </tr>
      <tr>
        <td><code>/green_light</code></td>
        <td><code>std_msgs/Bool</code></td>
        <td>Race start trigger. True transitions INIT → NORMAL_NAV.</td>
      </tr>
      <tr>
        <td><code>/mux_mode</code></td>
        <td><code>std_msgs/String</code></td>
        <td>Feedback from cmd_vel_mux. Used to verify mode changes landed correctly.</td>
      </tr>
    </tbody>
  </table>

  <h3 id="s3-3">3.3 cmd_vel_mux — Publishers / Subscribers</h3>
  <div class="grid2">
    <div class="card">
      <div class="card-title"><span class="dot dot-orange"></span>Subscribers (velocity sources)</div>
      <table>
        <thead><tr><th>Topic</th><th>Priority</th></tr></thead>
        <tbody>
          <tr><td><code>/cmd_vel_joy</code></td><td>Highest (joystick override)</td></tr>
          <tr><td><code>/cmd_vel_nav</code></td><td>Nav2 MPPI output</td></tr>
          <tr><td><code>/cmd_vel_zone_nav</code></td><td>zone_nav_manager direct cmds</td></tr>
        </tbody>
      </table>
    </div>
    <div class="card">
      <div class="card-title"><span class="dot dot-green"></span>Publishers</div>
      <table>
        <thead><tr><th>Topic</th><th>Purpose</th></tr></thead>
        <tbody>
          <tr><td><code>/cmd_vel_safe</code></td><td>Selected velocity → motors</td></tr>
          <tr><td><code>/mux_mode</code></td><td>Current active mode string</td></tr>
        </tbody>
      </table>
    </div>
  </div>

  <h3 id="s3-4">3.4 SetParameters Services Called by zone_nav_manager</h3>
  <table>
    <thead>
      <tr><th>Service Path</th><th>When Called</th><th>Parameter Set</th></tr>
    </thead>
    <tbody>
      <tr>
        <td><code>/global_costmap/global_costmap/set_parameters</code></td>
        <td>PUSH_THROUGH entry/exit</td>
        <td><code>obstacle_layer.enabled</code>, <code>inflation_layer.inflation_radius</code></td>
      </tr>
      <tr>
        <td><code>/local_costmap/local_costmap/set_parameters</code></td>
        <td>PUSH_THROUGH entry/exit</td>
        <td><code>voxel_layer.enabled</code>, <code>obstacle_layer.enabled</code></td>
      </tr>
      <tr>
        <td><code>/cmd_vel_mux_node/set_parameters</code></td>
        <td>Mode transitions</td>
        <td><code>mode</code> (JOYSTICK / AUTONOMOUS / BLIND_DRIVE)</td>
      </tr>
    </tbody>
  </table>

  <div class="callout callout-crit">
    <div class="callout-title">Doubled Namespace — See Section 6.2</div>
    The costmap service paths contain the node name twice. Using the wrong path is a silent no-op.
    Always use <code>/global_costmap/global_costmap/set_parameters</code>, not <code>/global_costmap/set_parameters</code>.
  </div>
</div>

<!-- ═══ SECTION 4 ═══ -->
<div class="page section pagebreak" id="s4">
  <div class="section-header">
    <span class="section-num yellow">4</span>
    <h2>Parameter Reference</h2>
  </div>

  <h3>zone_nav_manager Parameters</h3>
  <table>
    <thead>
      <tr><th>Parameter</th><th>Default</th><th>Unit</th><th>Description</th></tr>
    </thead>
    <tbody>
      <tr>
        <td><code>waypoints_file</code></td>
        <td><code>zone_waypoints.yaml</code></td>
        <td>path</td>
        <td>Path to zone definition file. Absolute or relative to share directory.</td>
      </tr>
      <tr>
        <td><code>total_laps</code></td>
        <td><code>2</code></td>
        <td>integer</td>
        <td>Number of laps before DONE is triggered at finish line. Must be ≥ 1.</td>
      </tr>
      <tr>
        <td><code>slow_speed_limit</code></td>
        <td><code>0.2</code></td>
        <td>m/s</td>
        <td>Speed limit published during SLOW_NAV zones.</td>
      </tr>
      <tr>
        <td><code>blind_speed</code></td>
        <td><code>0.15</code></td>
        <td>m/s</td>
        <td>Fixed forward speed during BLIND_DRIVE.</td>
      </tr>
      <tr>
        <td><code>blind_timeout_s</code></td>
        <td><code>15.0</code></td>
        <td>s</td>
        <td>Hard timeout in BLIND_DRIVE. Forces exit even if NDT never recovers.</td>
      </tr>
      <tr>
        <td><code>ndt_recovery_threshold</code></td>
        <td><code>0.8</code></td>
        <td>dimensionless</td>
        <td>NDT fitness score below which localization is considered restored. Exit BLIND_DRIVE when fitness &lt; threshold.</td>
      </tr>
      <tr>
        <td><code>speed_reassert_hz</code></td>
        <td><code>1.0</code></td>
        <td>Hz</td>
        <td>Rate at which speed limit is re-published (volatile QoS). Prevents controller_server from reverting to configured max.</td>
      </tr>
      <tr>
        <td><code>push_through_speed</code></td>
        <td><code>0.25</code></td>
        <td>m/s</td>
        <td>Fixed forward speed during PUSH_THROUGH (car wash).</td>
      </tr>
    </tbody>
  </table>

  <h3>cmd_vel_mux Parameters</h3>
  <table>
    <thead>
      <tr><th>Parameter</th><th>Default</th><th>Description</th></tr>
    </thead>
    <tbody>
      <tr>
        <td><code>mode</code></td>
        <td><code>JOYSTICK</code></td>
        <td>Active mux mode. Set via <code>/cmd_vel_mux_node/set_parameters</code>. Valid: JOYSTICK, AUTONOMOUS, BLIND_DRIVE. ESTOP_LOCK set by hardware only.</td>
      </tr>
      <tr>
        <td><code>joy_topic</code></td>
        <td><code>/cmd_vel_joy</code></td>
        <td>Joystick velocity input topic name.</td>
      </tr>
      <tr>
        <td><code>nav_topic</code></td>
        <td><code>/cmd_vel_nav</code></td>
        <td>Nav2 velocity input topic name.</td>
      </tr>
      <tr>
        <td><code>zone_nav_topic</code></td>
        <td><code>/cmd_vel_zone_nav</code></td>
        <td>zone_nav_manager direct velocity input topic name.</td>
      </tr>
      <tr>
        <td><code>output_topic</code></td>
        <td><code>/cmd_vel_safe</code></td>
        <td>Arbitrated output to motors.</td>
      </tr>
    </tbody>
  </table>
</div>

<!-- ═══ SECTION 5 ═══ -->
<div class="page section pagebreak" id="s5">
  <div class="section-header">
    <span class="section-num orange">5</span>
    <h2>zone_waypoints.yaml Format</h2>
  </div>

  <p>
    Zone waypoints are loaded at startup. The file defines a list of named zones. Each zone specifies
    the centre position, entry/exit radii, and the navigation mode to activate. The parser validates
    required fields and rejects invalid entries with an error at startup — it does not silently skip them.
  </p>

  <pre>
zones:
  - label: "tunnel_entry"        # string; optional — silently uses "" if omitted
    x: 12.45                     # float; map frame X coordinate (metres); REQUIRED
    y:  4.32                     # float; map frame Y coordinate (metres); REQUIRED
    radius: 0.5                  # float (metres); entry trigger distance; REQUIRED
    exit_radius: 0.9             # float (metres); exit trigger distance; REQUIRED
    mode: BLIND_DRIVE            # string; one of the 7 modes; REQUIRED

  - label: "narrow_section"
    x: 7.10
    y: 2.80
    radius: 0.4
    exit_radius: 0.8
    mode: SLOW_NAV

  - label: "finish_line"
    x: 0.50
    y: 0.20
    radius: 0.6
    exit_radius: 1.0
    mode: DONE                   # only triggers DONE when lap count == total_laps
</pre>

  <h3>Field Validation Rules</h3>
  <table>
    <thead>
      <tr><th>Rule</th><th>Enforcement</th><th>Effect of Violation</th></tr>
    </thead>
    <tbody>
      <tr>
        <td><code>mode</code> must be one of the 7 valid modes</td>
        <td>Startup validation</td>
        <td><strong>Rejected</strong> — error logged, zone skipped entirely</td>
      </tr>
      <tr>
        <td><code>x</code> and <code>y</code> are required numeric fields</td>
        <td>Startup validation</td>
        <td><strong>Rejected</strong> — error logged, zone skipped entirely</td>
      </tr>
      <tr>
        <td><code>radius</code> and <code>exit_radius</code> are required</td>
        <td>Startup validation</td>
        <td><strong>Rejected</strong> — error logged, zone skipped entirely</td>
      </tr>
      <tr>
        <td><code>radius</code> must be strictly less than <code>exit_radius</code></td>
        <td>Startup validation</td>
        <td><strong>Rejected</strong> — equal values cause mode flip-flop; startup error</td>
      </tr>
      <tr>
        <td><code>label</code> field is optional</td>
        <td>Silent default</td>
        <td>Zone loads with <code>label=""</code> — works but harder to debug</td>
      </tr>
      <tr>
        <td><code>total_laps</code> in manager params must be integer ≥ 1</td>
        <td>Startup param check</td>
        <td>Defaults to 2 if invalid; warning logged</td>
      </tr>
    </tbody>
  </table>

  <div class="callout callout-warn">
    <div class="callout-title">Zone Entry Loop Skips Same-Mode Zones</div>
    The zone entry detection loop skips any zone whose <code>mode == current_mode_</code>. This prevents
    NORMAL_NAV marker zones from re-triggering NORMAL_NAV and locking <code>active_zone_idx_</code>.
    Design implication: do <strong>not</strong> create a <code>tunnel_exit</code> zone with
    <code>mode=NORMAL_NAV</code> — it will block other zone triggers within its <code>exit_radius</code>.
  </div>
</div>

<!-- ═══ SECTION 6 ═══ -->
<div class="page section pagebreak" id="s6">
  <div class="section-header">
    <span class="section-num red">6</span>
    <h2>Key Implementation Gotchas</h2>
  </div>

  <p>
    This section is the most important in the document. It documents the hard-won knowledge distilled
    from 46 bugs encountered during implementation. Read this before writing any zone_nav code.
  </p>

  <h3 id="s6-1">6.1 Speed Control API</h3>
  <div class="card">
    <div class="card-title"><span class="dot dot-orange"></span>Use /speed_limit — Not SetParameters on FollowPath</div>
    <ul style="color:var(--subtext); padding-left:16px; font-size:10px; line-height:2.0;">
      <li><strong>/speed_limit topic is the ONLY working API</strong> for MPPI speed control at runtime</li>
      <li><code>FollowPath.vx_max</code> via <code>SetParameters</code> is <strong>silently ignored</strong> during active runs — the controller does not reload params mid-trajectory</li>
      <li><strong><code>speed_limit=0.0</code> means NO_SPEED_LIMIT</strong> (restores the configured max), NOT "stop the robot". To stop, publish zero Twist on <code>/cmd_vel_zone_nav</code> with mux in BLIND_DRIVE mode.</li>
      <li>controller_server uses <strong>volatile QoS</strong> on <code>/speed_limit</code> — the subscription does not use transient-local. Solution: <code>reassertSpeedLimit()</code> re-sends the limit at 1 Hz so the controller never reverts on a restart or reconnect.</li>
    </ul>
  </div>

  <div class="callout callout-crit">
    <div class="callout-title">Critical — speed_limit=0.0 Is Not Stop</div>
    Publishing <code>0.0</code> on <code>/speed_limit</code> removes the speed cap entirely. The robot will
    accelerate to its configured maximum. To hold position, switch mux to JOYSTICK and publish zero Twist.
  </div>

  <h3 id="s6-2">6.2 Costmap Service Paths — The Doubled Namespace</h3>
  <div class="card">
    <div class="card-title"><span class="dot dot-red"></span>The Namespace Doubling Trap</div>
    <p>Nav2 costmap nodes are launched with a namespace <em>and</em> a node name that matches the namespace.
    This creates a doubled path in the service name. Getting it wrong results in a service call that
    returns success but does nothing — a completely silent failure.</p>
    <pre style="margin-top:8px;">
# CORRECT — doubled namespace:
/global_costmap/global_costmap/set_parameters
/local_costmap/local_costmap/set_parameters

# WRONG — single namespace (silent no-op):
/global_costmap/set_parameters
/local_costmap/set_parameters</pre>
    <p>Verify the actual running service names with: <code>ros2 service list | grep costmap</code></p>
  </div>

  <h3 id="s6-3">6.3 VoxelLayer Limitations</h3>
  <div class="card">
    <div class="card-title"><span class="dot dot-yellow"></span>VoxelLayer Has No min_obstacle_height</div>
    <ul style="color:var(--subtext); padding-left:16px; font-size:10px; line-height:2.0;">
      <li>VoxelLayer does <strong>not</strong> support <code>min_obstacle_height</code> as a dynamic parameter — only <code>enabled</code> bool works dynamically at runtime</li>
      <li>PUSH_THROUGH disables <strong>both</strong> <code>obstacle_layer.enabled</code> and <code>voxel_layer.enabled</code> via the costmap SetParameters services to prevent car-wash ribbons from appearing as a wall</li>
      <li>After PUSH_THROUGH exit, both layers are re-enabled. The costmap will rebuild from fresh lidar scans within one scan cycle (~100ms).</li>
    </ul>
  </div>

  <h3 id="s6-4">6.4 ESTOP_LOCK is Hardware-Only</h3>
  <div class="card">
    <div class="card-title"><span class="dot dot-red"></span>Cannot Set ESTOP_LOCK via ros2 param set</div>
    <ul style="color:var(--subtext); padding-left:16px; font-size:10px; line-height:2.0;">
      <li>ESTOP_LOCK mode on cmd_vel_mux can <strong>only</strong> be triggered by the <code>/estop_active</code> signal going True — it is not settable via <code>ros2 param set</code></li>
      <li>Attempting <code>ros2 param set /cmd_vel_mux_node mode ESTOP_LOCK</code> returns an error</li>
      <li>After estop is released (<code>/estop_active</code> goes False), the mux reverts to <strong>JOYSTICK</strong> (the safest mode) — the operator must explicitly switch to AUTONOMOUS for autonomous navigation to resume</li>
    </ul>
  </div>

  <h3 id="s6-5">6.5 BLIND_DRIVE Must-Knows</h3>
  <div class="card">
    <div class="card-title"><span class="dot dot-red"></span>BLIND_DRIVE Timing and State Details</div>
    <ul style="color:var(--subtext); padding-left:16px; font-size:10px; line-height:2.0;">
      <li>The mux is set to BLIND_DRIVE <strong>before</strong> the first <code>/cmd_vel_zone_nav</code> is published — there is a 50–100ms gap where the mux switches before the zone_nav_manager starts publishing. This is intentional; the mux needs to be ready before the velocity arrives.</li>
      <li><code>latest_fitness_</code> is <strong>reset to 999</strong> on BLIND_DRIVE entry — this prevents the exit condition from triggering immediately on stale pre-entry fitness data that happens to be below the threshold</li>
      <li><code>checkFinishLine()</code> is called <strong>inside the BLIND_DRIVE loop</strong> — if the tunnel is near the finish line, the robot can still detect and respond to the finish line during blind drive</li>
      <li>The <strong>15s hard timeout</strong> triggers if NDT never recovers (e.g., FAST-LIO2 crash). After timeout, the robot returns to NORMAL_NAV without lidar-verified position — proceed with caution.</li>
    </ul>
  </div>

  <h3 id="s6-6">6.6 zone_waypoints.yaml Pitfalls</h3>
  <div class="card">
    <div class="card-title"><span class="dot dot-yellow"></span>Common YAML Configuration Mistakes</div>
    <ul style="color:var(--subtext); padding-left:16px; font-size:10px; line-height:2.0;">
      <li><strong>Do NOT add a tunnel_exit zone with <code>mode=NORMAL_NAV</code></strong> — it sets <code>active_zone_idx_</code> and blocks all other zone triggers within exit_radius for as long as the robot remains inside that radius</li>
      <li>The zone entry loop <strong>skips zones whose mode == current_mode_</strong> — this is a feature, not a bug. It prevents NORMAL_NAV marker zones from re-locking zone detection. Side effect: you can't chain two same-mode zones that are close together.</li>
      <li><strong><code>radius</code> must be strictly less than <code>exit_radius</code></strong> — hysteresis is required. Equal values cause repeated mode flip-flop as the robot hovers at the boundary. The parser rejects equal values at startup.</li>
      <li>An <strong>unlabeled zone</strong> (no <code>label:</code> field) loads silently with <code>label=""</code> — no warning. This makes debug logs harder to read.</li>
      <li>A <strong>zone without <code>mode:</code></strong> is rejected at startup with an error — it does not silently default.</li>
    </ul>
  </div>

  <h3 id="s6-7">6.7 lidar_odom_gate Must Always Run with use_localization</h3>
  <div class="card">
    <div class="card-title"><span class="dot dot-purple"></span>Gate Node Is Not Optional</div>
    <ul style="color:var(--subtext); padding-left:16px; font-size:10px; line-height:2.0;">
      <li>EKF2 subscribes to <code>/lidar_odometry_gated</code> — the gate node's output. <strong>Without the gate node running, EKF2 receives no lidar odometry at all</strong>, even when zone_nav is in NORMAL_NAV.</li>
      <li>The gate has a <strong>5s watchdog</strong>: if zone_nav_manager crashes while in BLIND_DRIVE (gate closed), the gate automatically re-opens after 5s of silence on <code>/nav_mode</code>. This prevents EKF2 from starving permanently if the manager crashes mid-tunnel.</li>
      <li>Default mode is <strong>INIT</strong>, which keeps the gate open (pass-through). Safe for development runs without zone_nav active.</li>
    </ul>
  </div>
</div>

<!-- ═══ SECTION 7 ═══ -->
<div class="page section pagebreak" id="s7">
  <div class="section-header">
    <span class="section-num">7</span>
    <h2>Debugging Checklist</h2>
  </div>

  <p>Step-by-step diagnostics. Work top-to-bottom. Each step assumes the previous passes.</p>

  <h3>Topics to Monitor During a Test Run</h3>
  <div class="grid2">
    <div class="card">
      <div class="card-title"><span class="dot dot-cyan"></span>Monitor Commands</div>
      <pre>
# State machine state (expect INIT until green light)
ros2 topic echo /nav_mode

# Mux arbitration (expect AUTONOMOUS during Nav2 run)
ros2 topic echo /mux_mode

# NDT quality: >0.8 in tunnel, <0.8 after exit
ros2 topic echo /ndt_fitness_score

# Speed cap active
ros2 topic echo /speed_limit

# What actually reaches the motors
ros2 topic echo /cmd_vel_safe

# Must be False for robot to move
ros2 topic echo /estop_active

# Robot position in map frame
ros2 topic echo /odometry/filtered --field pose.pose.position</pre>
    </div>
    <div class="card">
      <div class="card-title"><span class="dot dot-green"></span>Service Verification Commands</div>
      <pre>
# Verify costmap services exist (doubled namespace)
ros2 service list | grep global_costmap
ros2 service list | grep local_costmap
ros2 service list | grep cmd_vel_mux_node

# Manually trigger green light (bench test)
ros2 topic pub --once /green_light \
  std_msgs/msg/Bool "data: true"

# Check current mux mode
ros2 param get /cmd_vel_mux_node mode

# Force AUTONOMOUS mode manually
ros2 param set /cmd_vel_mux_node mode AUTONOMOUS

# Check node list
ros2 node list | grep -E \
  "zone_nav|cmd_vel_mux|lidar_odom_gate"</pre>
    </div>
  </div>

  <h3>Common Failure Modes and Diagnostics</h3>
  <table>
    <thead>
      <tr><th>Symptom</th><th>First Check</th><th>Likely Cause</th><th>Fix</th></tr>
    </thead>
    <tbody>
      <tr>
        <td><strong>Robot won't move after green light</strong></td>
        <td><code>ros2 topic echo /mux_mode</code></td>
        <td>Mux still in JOYSTICK, or <code>/estop_active</code> is True</td>
        <td>Check <code>/estop_active</code>. Manually set mux to AUTONOMOUS if estop clear.</td>
      </tr>
      <tr>
        <td><strong>Robot ignores zone entry</strong></td>
        <td><code>ros2 topic echo /nav_mode</code></td>
        <td>Zone coordinates wrong vs actual robot position, or zone has same mode as current</td>
        <td>Echo <code>/odometry/filtered</code> near zone. Compare against YAML coordinates. Check mode != current_mode_ rule.</td>
      </tr>
      <tr>
        <td><strong>Speed doesn't change in zone</strong></td>
        <td><code>ros2 topic echo /speed_limit</code></td>
        <td>speed_limit not publishing, or controller_server not running</td>
        <td>Check zone_nav_manager is alive. Verify controller_server node is up with <code>ros2 node list</code>.</td>
      </tr>
      <tr>
        <td><strong>Costmap not updating in PUSH_THROUGH</strong></td>
        <td><code>ros2 service list | grep costmap</code></td>
        <td>Wrong service path (single namespace instead of doubled)</td>
        <td>Verify service path has doubled namespace. See Section 6.2.</td>
      </tr>
      <tr>
        <td><strong>Robot drifts severely in tunnel</strong></td>
        <td><code>ros2 topic echo /lidar_odometry_gated</code></td>
        <td>Gate not closing — lidar still feeding EKF2 with corrupt odometry during BLIND_DRIVE</td>
        <td>Confirm lidar_odom_gate node is running. Check /nav_mode is publishing BLIND_DRIVE. Check gate node logs.</td>
      </tr>
      <tr>
        <td><strong>DONE never triggered at finish</strong></td>
        <td>Check finish_line coordinates in YAML</td>
        <td>Finish zone coordinates wrong, or lap counter never reached total_laps</td>
        <td>Echo <code>/odometry/filtered</code> at finish. Verify <code>/ndt_fitness_score</code> recovered after tunnel (needed for position accuracy).</td>
      </tr>
      <tr>
        <td><strong>EKF2 has no lidar input</strong></td>
        <td><code>ros2 topic hz /lidar_odometry_gated</code></td>
        <td>lidar_odom_gate not running, or gate stuck closed (zone_nav crashed in BLIND_DRIVE)</td>
        <td>Start lidar_odom_gate. If crashed mid-tunnel, gate re-opens after 5s watchdog — or restart gate node.</td>
      </tr>
    </tbody>
  </table>

  <div class="callout callout-good">
    <div class="callout-title">Quick Health Check</div>
    Run <code>scripts/health_check_zone_nav.sh</code> before every test run. It checks all topics,
    services, and current mode values in under 30 seconds and gives you a PASS/FAIL summary.
  </div>
</div>

<!-- ═══ SECTION 8 ═══ -->
<div class="page section pagebreak" id="s8">
  <div class="section-header">
    <span class="section-num green">8</span>
    <h2>Scripts Reference</h2>
  </div>

  <p>
    All scripts live in <code>scripts/</code> at the repository root. Run <code>chmod +x scripts/*.sh</code>
    once after cloning. All scripts require a sourced ROS 2 environment.
  </p>

  <div class="grid2">
    <div class="card">
      <div class="card-title"><span class="dot dot-green"></span>health_check_zone_nav.sh</div>
      <p><strong>Pre-run sanity check.</strong> Run before every test. Checks all required topics are
      publishing (<code>/nav_mode</code>, <code>/mux_mode</code>, <code>/odometry/filtered</code>,
      <code>/ndt_fitness_score</code>, <code>/cmd_vel_safe</code>). Verifies all three SetParameters
      services are available. Reads current mode values. Prints colour-coded PASS/FAIL for each check.
      Exits 0 if all pass, 1 if any fail.</p>
      <pre>./scripts/health_check_zone_nav.sh</pre>
    </div>
    <div class="card">
      <div class="card-title"><span class="dot dot-cyan"></span>inspect_zone_nav.sh</div>
      <p><strong>Live dashboard.</strong> Updates every 1 second. Shows current state machine mode,
      mux mode, e-stop status, NDT fitness score, robot X/Y position, active speed limit,
      and current <code>/cmd_vel_safe</code> output. Handles timeouts gracefully (shows TIMEOUT in red).
      Press Ctrl-C to exit.</p>
      <pre>./scripts/inspect_zone_nav.sh</pre>
    </div>
    <div class="card">
      <div class="card-title"><span class="dot dot-yellow"></span>trigger_green_light.sh</div>
      <p><strong>Bench testing without the physical green light.</strong> Publishes a single True message
      on <code>/green_light</code> to trigger the INIT → NORMAL_NAV transition. Use this when developing
      on a bench without the physical starting light hardware connected.</p>
      <pre>./scripts/trigger_green_light.sh</pre>
    </div>
    <div class="card">
      <div class="card-title"><span class="dot dot-purple"></span>set_mux_mode.sh</div>
      <p><strong>Manual mode switch helper.</strong> Calls <code>ros2 param set</code> on
      <code>/cmd_vel_mux_node</code> to change mux mode. Accepts JOYSTICK, AUTONOMOUS, or BLIND_DRIVE.
      Refuses ESTOP_LOCK (hardware-only) with an explanatory error. Prints current mode if called
      with no argument.</p>
      <pre>./scripts/set_mux_mode.sh AUTONOMOUS
./scripts/set_mux_mode.sh JOYSTICK</pre>
    </div>
  </div>

  <div class="callout callout-info">
    <div class="callout-title">Recommended Test Run Order</div>
    1. Source ROS 2 workspace. 2. Launch full stack. 3. Run <code>health_check_zone_nav.sh</code> — fix any FAILs.
    4. Open <code>inspect_zone_nav.sh</code> in a second terminal. 5. Run <code>trigger_green_light.sh</code>.
    6. Watch inspect dashboard: nav_mode should change INIT → NORMAL_NAV, mux_mode → AUTONOMOUS.
  </div>

  <hr class="divider"/>
  <p style="text-align:center; color: var(--muted); font-size:9px;">
    DIY Robot Challenge 2026 — Zone-Aware Navigation Software Design Guide v1.0 — Team Juggernauts — 2026
  </p>
</div>


<script>
document.querySelectorAll('.card-title').forEach(function(title) {{
  var btn = document.createElement('button');
  btn.className = 'copy-btn';
  btn.textContent = 'copy';
  btn.addEventListener('click', function() {{
    var card = title.closest('.card');
    var text = card.innerText.replace(/^copy$/m, '').trim();
    navigator.clipboard.writeText(text).then(function() {{
      btn.textContent = 'copied!';
      setTimeout(function() {{ btn.textContent = 'copy'; }}, 1500);
    }});
  }});
  title.appendChild(btn);
}});
</script>

</body>
</html>
"""

# ─────────────────────────────────────────────────────────────────────────────
# Write HTML
# ─────────────────────────────────────────────────────────────────────────────
OUT_HTML.write_text(HTML, encoding="utf-8")
print(f"[gen] HTML written → {OUT_HTML}")

# ─────────────────────────────────────────────────────────────────────────────
# Generate PDF via WeasyPrint
# ─────────────────────────────────────────────────────────────────────────────
try:
    import weasyprint
    wp = weasyprint.HTML(filename=str(OUT_HTML))
    wp.write_pdf(str(OUT_PDF))
    print(f"[gen] PDF written  → {OUT_PDF}")
except Exception as e:
    print(f"[gen] PDF generation failed: {e}")
    print(f"[gen] HTML is still available at {OUT_HTML}")
