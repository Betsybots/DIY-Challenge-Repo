#!/usr/bin/env python3
"""
generate_nav_design.py
Generates docs/Navigation_Design_Guide.html + .pdf
Obstacle course navigation design document — Team Juggernauts 2026
"""

import subprocess
import textwrap
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
OUT_HTML  = REPO_ROOT / "docs" / "Navigation_Design_Guide.html"
OUT_PDF   = REPO_ROOT / "docs" / "Navigation_Design_Guide.pdf"


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
    # Strip XML declaration and DOCTYPE for inline embedding
    lines = [l for l in svg.splitlines() if not l.startswith("<?xml") and not l.startswith("<!DOCTYPE")]
    return "\n".join(lines)


# ─────────────────────────────────────────────────────────────────────────────
# Diagram 1 — Localization flow (Graphviz)
# ─────────────────────────────────────────────────────────────────────────────
DOT_LOCALIZATION = """
digraph localization {
  graph [bgcolor="#0f172a" fontname="Inter" rankdir=LR splines=ortho nodesep=0.6 ranksep=0.9]
  node  [fontname="Inter" fontsize=10 style="filled,rounded" shape=box penwidth=1.5]
  edge  [fontname="Inter" fontsize=9 color="#475569"]

  hesai   [label="Hesai QT64\\n/hesai/points" fillcolor="#0c1a2e" fontcolor="#0ea5e9" color="#0ea5e9"]
  imu     [label="ACEINNA IMU\\n/imu/data"    fillcolor="#0c1a2e" fontcolor="#a855f7" color="#a855f7"]
  wheel   [label="Dead-Wheel\\nEncoders\\n/wheel_cmd_vel" fillcolor="#0c1a2e" fontcolor="#f97316" color="#f97316"]

  fastlio [label="FAST-LIO2\\n(lidar-inertial\\nodometry)"  fillcolor="#172554" fontcolor="#93c5fd" color="#3b82f6"]
  ekf1    [label="EKF1\\nWheel + IMU\\n→ /wimu_odom"        fillcolor="#14532d" fontcolor="#86efac" color="#22c55e"]
  ekf2    [label="EKF2\\nWImuOdom + LiDAR\\n→ /odometry/filtered" fillcolor="#14532d" fontcolor="#86efac" color="#22c55e"]
  ndt     [label="NDT-OMP\\n(map matching)\\n→ map→odom TF" fillcolor="#1e1b4b" fontcolor="#c4b5fd" color="#a855f7"]
  pcd     [label="GlobalMap.pcd\\n(LIO-SAM output)"         fillcolor="#0c1a2e" fontcolor="#94a3b8" color="#334155" shape=cylinder]

  nav2    [label="Nav2\\nSmac + MPPI\\n→ /cmd_vel_nav"      fillcolor="#431407" fontcolor="#fed7aa" color="#f97316"]
  mux     [label="cmd_vel_mux\\n(joy/nav/estop)"            fillcolor="#1c1917" fontcolor="#d6d3d1" color="#78716c"]
  motors  [label="STM32\\nMotors"                           fillcolor="#1c1917" fontcolor="#d6d3d1" color="#78716c"]

  hesai  -> fastlio
  imu    -> fastlio
  imu    -> ekf1
  wheel  -> ekf1
  fastlio -> ekf2  [label="/lidar_odometry"]
  ekf1    -> ekf2  [label="/wimu_odom"]
  pcd    -> ndt
  hesai  -> ndt    [label="live scan"]
  ndt    -> nav2   [label="map→odom TF" color="#a855f7" fontcolor="#a855f7"]
  ekf2   -> nav2   [label="/odometry/filtered" color="#22c55e" fontcolor="#22c55e"]
  nav2   -> mux
  mux    -> motors
}
"""

# ─────────────────────────────────────────────────────────────────────────────
# Diagram 2 — Zone state machine (Graphviz)
# ─────────────────────────────────────────────────────────────────────────────
DOT_STATE_MACHINE = """
digraph state_machine {
  graph [bgcolor="#0f172a" fontname="Inter" rankdir=TB splines=curved nodesep=0.5 ranksep=0.7]
  node  [fontname="Inter" fontsize=10 style="filled,rounded" shape=box penwidth=1.5 width=1.8]
  edge  [fontname="Inter" fontsize=8 color="#475569"]

  START      [label="START\\nWait for pose" fillcolor="#1e293b" fontcolor="#94a3b8" color="#475569"]
  NORMAL     [label="NORMAL_NAV\\nNav2 Smac+MPPI\\nFull speed"    fillcolor="#14532d" fontcolor="#86efac" color="#22c55e"]
  SLOW       [label="SLOW_NAV\\nNav2 reduced speed\\n0.2 m/s"     fillcolor="#164e63" fontcolor="#a5f3fc" color="#06b6d4"]
  BLIND      [label="BLIND_DRIVE\\nWheel+IMU only\\nFixed heading" fillcolor="#450a0a" fontcolor="#fca5a5" color="#ef4444"]
  NAVIGATE   [label="NAVIGATE_AROUND\\nLive costmap\\nDynamic replan" fillcolor="#1e1b4b" fontcolor="#c4b5fd" color="#a855f7"]
  PUSH       [label="PUSH_THROUGH\\nStraight waypoint\\nObstacle layer OFF" fillcolor="#422006" fontcolor="#fdba74" color="#f97316"]
  DONE       [label="DONE\\nStop + signal" fillcolor="#1e293b" fontcolor="#94a3b8" color="#475569"]

  START   -> NORMAL   [label="pose valid"]
  NORMAL  -> SLOW     [label="narrow / ramp\\n/ bank zone"]
  NORMAL  -> BLIND    [label="tunnel entry\\n(lidar loss)"]
  NORMAL  -> NAVIGATE [label="obstacle zone\\n(buckets)"]
  NORMAL  -> PUSH     [label="car wash zone"]
  SLOW    -> NORMAL   [label="zone exit"]
  BLIND   -> NORMAL   [label="tunnel exit\\n(lidar restored)"]
  NAVIGATE -> NORMAL  [label="obstacle cleared"]
  PUSH    -> NORMAL   [label="zone exit"]
  NORMAL  -> DONE     [label="finish waypoint"]
}
"""

# ─────────────────────────────────────────────────────────────────────────────
# Diagram 3 — Costmap layer architecture (Graphviz)
# ─────────────────────────────────────────────────────────────────────────────
DOT_COSTMAP = """
digraph costmap {
  graph [bgcolor="#0f172a" fontname="Inter" rankdir=TB splines=ortho nodesep=0.4 ranksep=0.5]
  node  [fontname="Inter" fontsize=10 style="filled,rounded" shape=box penwidth=1.2]
  edge  [fontname="Inter" fontsize=8 color="#475569"]

  subgraph cluster_global {
    label="Global Costmap  (rolling_window: false)"
    fontname="Inter" fontsize=10 fontcolor="#94a3b8"
    color="#334155" style=rounded bgcolor="#111827"

    obs_g  [label="obstacle_layer\\n/hesai/points → mark/clear" fillcolor="#172554" fontcolor="#93c5fd" color="#3b82f6"]
    inf_g  [label="inflation_layer\\nradius: 0.45m" fillcolor="#14532d" fontcolor="#86efac" color="#22c55e"]
    obs_g -> inf_g [label="feeds"]
  }

  subgraph cluster_local {
    label="Local Costmap  (rolling_window: true, 4×4m)"
    fontname="Inter" fontsize=10 fontcolor="#94a3b8"
    color="#334155" style=rounded bgcolor="#111827"

    vox_l  [label="voxel_layer\\n/hesai/points → 3D voxels" fillcolor="#172554" fontcolor="#93c5fd" color="#3b82f6"]
    inf_l  [label="inflation_layer\\nradius: 0.40m" fillcolor="#14532d" fontcolor="#86efac" color="#22c55e"]
    vox_l -> inf_l [label="feeds"]
  }

  smac  [label="Smac Hybrid-A*\\nGlobal planner" fillcolor="#1e1b4b" fontcolor="#c4b5fd" color="#a855f7"]
  mppi  [label="MPPI Controller\\nLocal controller" fillcolor="#431407" fontcolor="#fed7aa" color="#f97316"]
  note  [label="No static_layer.\\nNo .pgm file needed.\\nAll from live lidar." fillcolor="#0f172a" fontcolor="#22c55e" color="#22c55e" shape=note]

  inf_g -> smac  [label="costmap"]
  inf_l -> mppi  [label="costmap"]
  note  -> obs_g [style=dashed color="#22c55e"]
}
"""

# ─────────────────────────────────────────────────────────────────────────────
# Diagram 4 — Tunnel blind drive sequence (hand-crafted SVG)
# ─────────────────────────────────────────────────────────────────────────────
SVG_TUNNEL = """
<svg viewBox="0 0 760 180" xmlns="http://www.w3.org/2000/svg" font-family="Inter,sans-serif">
  <defs>
    <marker id="arr" markerWidth="8" markerHeight="8" refX="6" refY="3" orient="auto">
      <path d="M0,0 L0,6 L8,3 z" fill="#94a3b8"/>
    </marker>
    <marker id="arr-green" markerWidth="8" markerHeight="8" refX="6" refY="3" orient="auto">
      <path d="M0,0 L0,6 L8,3 z" fill="#22c55e"/>
    </marker>
    <marker id="arr-red" markerWidth="8" markerHeight="8" refX="6" refY="3" orient="auto">
      <path d="M0,0 L0,6 L8,3 z" fill="#ef4444"/>
    </marker>
    <marker id="arr-orange" markerWidth="8" markerHeight="8" refX="6" refY="3" orient="auto">
      <path d="M0,0 L0,6 L8,3 z" fill="#f97316"/>
    </marker>
  </defs>

  <!-- Background -->
  <rect width="760" height="180" fill="#0f172a" rx="8"/>

  <!-- Course ground -->
  <rect x="20" y="100" width="720" height="12" fill="#1e293b" rx="2"/>

  <!-- Pre-tunnel: open section -->
  <rect x="20" y="60" width="160" height="52" fill="none" stroke="#334155" stroke-dasharray="4,3" rx="4"/>
  <text x="100" y="50" text-anchor="middle" fill="#94a3b8" font-size="9">Open section</text>
  <text x="100" y="88" text-anchor="middle" fill="#22c55e" font-size="9">FAST-LIO2 active</text>
  <text x="100" y="100" text-anchor="middle" fill="#22c55e" font-size="9">EKF fusing</text>

  <!-- Tunnel walls -->
  <rect x="200" y="62" width="340" height="8" fill="#334155" rx="2"/>
  <rect x="200" y="102" width="340" height="8" fill="#334155" rx="2"/>
  <!-- Tunnel roof fill -->
  <rect x="200" y="70" width="340" height="32" fill="#1a1a2e" rx="0"/>
  <text x="370" y="52" text-anchor="middle" fill="#ef4444" font-size="9" font-weight="600">TUNNEL (foil-lined — lidar blocked)</text>
  <text x="370" y="92" text-anchor="middle" fill="#fca5a5" font-size="9">BLIND_DRIVE: wheel+IMU EKF only</text>
  <!-- RF/lidar block icon -->
  <text x="370" y="106" text-anchor="middle" fill="#ef4444" font-size="8">⚡ lidar signal lost</text>

  <!-- Post-tunnel: open again -->
  <rect x="560" y="60" width="160" height="52" fill="none" stroke="#334155" stroke-dasharray="4,3" rx="4"/>
  <text x="640" y="50" text-anchor="middle" fill="#94a3b8" font-size="9">Open section</text>
  <text x="640" y="88" text-anchor="middle" fill="#22c55e" font-size="9">FAST-LIO2 resumes</text>
  <text x="640" y="100" text-anchor="middle" fill="#22c55e" font-size="9">TF drift corrected</text>

  <!-- Robot icon pre-tunnel -->
  <rect x="130" y="74" width="28" height="22" fill="#0ea5e9" rx="3"/>
  <text x="144" y="89" text-anchor="middle" fill="#fff" font-size="8">🤖</text>

  <!-- Robot icon mid-tunnel -->
  <rect x="340" y="74" width="28" height="22" fill="#ef4444" rx="3"/>
  <text x="354" y="89" text-anchor="middle" fill="#fff" font-size="8">🤖</text>

  <!-- Robot icon post-tunnel -->
  <rect x="600" y="74" width="28" height="22" fill="#22c55e" rx="3"/>
  <text x="614" y="89" text-anchor="middle" fill="#fff" font-size="8">🤖</text>

  <!-- Arrows -->
  <line x1="158" y1="85" x2="196" y2="85" stroke="#94a3b8" stroke-width="1.5" marker-end="url(#arr)"/>
  <line x1="368" y1="85" x2="536" y2="85" stroke="#ef4444" stroke-width="1.5" stroke-dasharray="5,3" marker-end="url(#arr-red)"/>
  <line x1="558" y1="85" x2="596" y2="85" stroke="#22c55e" stroke-width="1.5" marker-end="url(#arr-green)"/>

  <!-- Entry/exit markers -->
  <line x1="200" y1="55" x2="200" y2="115" stroke="#f97316" stroke-width="1" stroke-dasharray="3,2"/>
  <text x="200" y="130" text-anchor="middle" fill="#f97316" font-size="8">Entry</text>
  <text x="200" y="140" text-anchor="middle" fill="#f97316" font-size="8">freeze TF</text>

  <line x1="540" y1="55" x2="540" y2="115" stroke="#22c55e" stroke-width="1" stroke-dasharray="3,2"/>
  <text x="540" y="130" text-anchor="middle" fill="#22c55e" font-size="8">Exit</text>
  <text x="540" y="140" text-anchor="middle" fill="#22c55e" font-size="8">restore TF</text>

  <!-- Legend -->
  <rect x="20" y="150" width="10" height="10" fill="#0ea5e9" rx="2"/>
  <text x="34" y="159" fill="#94a3b8" font-size="8">Normal nav (FAST-LIO2 active)</text>
  <rect x="220" y="150" width="10" height="10" fill="#ef4444" rx="2"/>
  <text x="234" y="159" fill="#94a3b8" font-size="8">Blind drive (dead reckoning)</text>
  <rect x="430" y="150" width="10" height="10" fill="#22c55e" rx="2"/>
  <text x="444" y="159" fill="#94a3b8" font-size="8">Resumed nav (re-localised)</text>
</svg>
"""

# ─────────────────────────────────────────────────────────────────────────────
# Diagram 5 — Course zone map (hand-crafted SVG)
# ─────────────────────────────────────────────────────────────────────────────
SVG_COURSE_MAP = """
<svg viewBox="0 0 760 420" xmlns="http://www.w3.org/2000/svg" font-family="Inter,sans-serif" font-size="9">
  <rect width="760" height="420" fill="#0f172a" rx="8"/>

  <!-- Outer boundary -->
  <rect x="20" y="20" width="720" height="380" fill="none" stroke="#334155" stroke-width="2" rx="6"/>

  <!-- START/FINISH -->
  <rect x="300" y="340" width="160" height="44" fill="#1e293b" stroke="#22c55e" stroke-width="1.5" rx="4"/>
  <text x="380" y="358" text-anchor="middle" fill="#22c55e" font-weight="600">START / FINISH</text>
  <text x="380" y="374" text-anchor="middle" fill="#94a3b8">32" wide straight</text>

  <!-- Car wash -->
  <rect x="460" y="340" width="120" height="44" fill="#172554" stroke="#3b82f6" stroke-width="1.5" rx="4"/>
  <text x="520" y="358" text-anchor="middle" fill="#93c5fd" font-weight="600">CAR WASH</text>
  <text x="520" y="370" text-anchor="middle" fill="#64748b">hanging ribbons</text>
  <text x="520" y="382" text-anchor="middle" fill="#ef4444">⚠ costmap filter</text>

  <!-- Hoop section -->
  <rect x="580" y="240" width="120" height="90" fill="#1e1b4b" stroke="#a855f7" stroke-width="1.5" rx="4"/>
  <text x="640" y="258" text-anchor="middle" fill="#c4b5fd" font-weight="600">HOOP</text>
  <text x="640" y="271" text-anchor="middle" fill="#94a3b8">random positions</text>
  <text x="640" y="284" text-anchor="middle" fill="#94a3b8">36" wide</text>
  <text x="640" y="298" text-anchor="middle" fill="#22c55e">NORMAL_NAV</text>

  <!-- Obstacle section (buckets) -->
  <rect x="200" y="180" width="260" height="150" fill="#1c1917" stroke="#f97316" stroke-width="1.5" rx="4"/>
  <text x="330" y="198" text-anchor="middle" fill="#fdba74" font-weight="600">OBSTACLE SECTION</text>
  <text x="330" y="211" text-anchor="middle" fill="#94a3b8">2–9 gallon buckets (random)</text>
  <!-- Bucket circles -->
  <circle cx="270" cy="250" r="22" fill="#292524" stroke="#f97316" stroke-width="1.2" stroke-dasharray="4,2"/>
  <circle cx="320" cy="280" r="22" fill="#292524" stroke="#f97316" stroke-width="1.2" stroke-dasharray="4,2"/>
  <circle cx="370" cy="245" r="22" fill="#292524" stroke="#f97316" stroke-width="1.2" stroke-dasharray="4,2"/>
  <circle cx="410" cy="290" r="22" fill="#292524" stroke="#f97316" stroke-width="1.2" stroke-dasharray="4,2"/>
  <text x="330" y="320" text-anchor="middle" fill="#f97316">NAVIGATE_AROUND</text>

  <!-- Gravel + ramps (top) -->
  <rect x="140" y="30" width="460" height="70" fill="#1c1917" stroke="#eab308" stroke-width="1.5" rx="4"/>
  <text x="370" y="50" text-anchor="middle" fill="#fde68a" font-weight="600">GRAVEL SECTION  +  RAMPS</text>
  <text x="370" y="65" text-anchor="middle" fill="#94a3b8">4×8 box / 1.5" deep pea gravel  ·  2" ramp down+up  ·  48" wide</text>
  <text x="370" y="80" text-anchor="middle" fill="#eab308">SLOW_NAV (0.3 m/s)  —  IMU pitch detection</text>

  <!-- Bank section (top right) -->
  <rect x="610" y="30" width="120" height="70" fill="#1a1a2e" stroke="#6366f1" stroke-width="1.5" rx="4"/>
  <text x="670" y="55" text-anchor="middle" fill="#a5b4fc" font-weight="600">BANK</text>
  <text x="670" y="68" text-anchor="middle" fill="#94a3b8">8.5° lateral tilt</text>
  <text x="670" y="81" text-anchor="middle" fill="#6366f1">SLOW_NAV</text>

  <!-- Narrow section (left) -->
  <rect x="30" y="120" width="100" height="140" fill="#1a1a1a" stroke="#84cc16" stroke-width="1.5" rx="4"/>
  <text x="80" y="145" text-anchor="middle" fill="#bef264" font-weight="600">NARROW</text>
  <text x="80" y="158" text-anchor="middle" fill="#94a3b8">20" wide</text>
  <text x="80" y="171" text-anchor="middle" fill="#94a3b8">curved path</text>
  <text x="80" y="188" text-anchor="middle" fill="#84cc16">SLOW_NAV</text>
  <text x="80" y="201" text-anchor="middle" fill="#84cc16">&lt;4cm lateral</text>
  <text x="80" y="214" text-anchor="middle" fill="#84cc16">accuracy req.</text>

  <!-- Tunnel (left lower) -->
  <rect x="30" y="270" width="160" height="60" fill="#450a0a" stroke="#ef4444" stroke-width="1.5" rx="4"/>
  <text x="110" y="293" text-anchor="middle" fill="#fca5a5" font-weight="600">TUNNEL</text>
  <text x="110" y="306" text-anchor="middle" fill="#94a3b8">32" wide · 6" long</text>
  <text x="110" y="319" text-anchor="middle" fill="#ef4444">BLIND_DRIVE ⚡</text>

  <!-- Ramp section (bottom left) -->
  <rect x="30" y="340" width="150" height="44" fill="#1c1917" stroke="#eab308" stroke-width="1.5" rx="4"/>
  <text x="105" y="358" text-anchor="middle" fill="#fde68a" font-weight="600">RAMP / HELIX</text>
  <text x="105" y="372" text-anchor="middle" fill="#94a3b8">4' radius · 11% grade</text>

  <!-- Pothole (wide section) -->
  <rect x="140" y="110" width="200" height="60" fill="#1a1a1a" stroke="#94a3b8" stroke-width="1" rx="4" stroke-dasharray="4,2"/>
  <text x="240" y="132" text-anchor="middle" fill="#94a3b8" font-weight="600">POTHOLE / WIDE</text>
  <text x="240" y="147" text-anchor="middle" fill="#64748b">0.75" bumps · 11×26' area</text>
  <text x="240" y="160" text-anchor="middle" fill="#94a3b8">NORMAL_NAV</text>

  <!-- Arrows showing rough flow direction -->
  <text x="380" y="410" text-anchor="middle" fill="#475569" font-size="8">Course layout approximate — not to scale</text>

  <!-- Legend -->
  <rect x="590" y="355" width="10" height="10" fill="#450a0a" stroke="#ef4444" stroke-width="1" rx="1"/>
  <text x="604" y="364" fill="#94a3b8">BLIND_DRIVE</text>
  <rect x="590" y="370" width="10" height="10" fill="#14532d" stroke="#22c55e" stroke-width="1" rx="1"/>
  <text x="604" y="379" fill="#94a3b8">NORMAL_NAV</text>
  <rect x="670" y="355" width="10" height="10" fill="#164e63" stroke="#06b6d4" stroke-width="1" rx="1"/>
  <text x="684" y="364" fill="#94a3b8">SLOW_NAV</text>
  <rect x="670" y="370" width="10" height="10" fill="#1c1917" stroke="#f97316" stroke-width="1" rx="1"/>
  <text x="684" y="379" fill="#94a3b8">NAVIGATE_AROUND</text>
</svg>
"""

# ─────────────────────────────────────────────────────────────────────────────
# Pre-render Graphviz diagrams
# ─────────────────────────────────────────────────────────────────────────────
svg_localization   = dot_to_svg(DOT_LOCALIZATION)
svg_state_machine  = dot_to_svg(DOT_STATE_MACHINE)
svg_costmap        = dot_to_svg(DOT_COSTMAP)


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

.card { background: var(--card); border: 1px solid var(--border); border-radius: 8px; padding: 16px; margin-bottom: 12px; }
.card-title { font-size: 12px; font-weight: 600; color: #fff; margin-bottom: 6px; display: flex; align-items: center; gap: 7px; }
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
<title>Obstacle Course Navigation Design — DIY Robot Challenge 2026</title>
<style>{CSS}</style>
</head>
<body>

<!-- ═══ COVER ═══ -->
<div class="cover">
  <div class="cover-badge">Navigation Design Document v1.0</div>
  <h1>Obstacle Course<br/><span>Navigation Design</span></h1>
  <p class="subtitle">
    How we navigate the 2026 DIY Robot Challenge obstacle course — zone-aware state machine,
    map-free costmap strategy, dual EKF fusion, and sensor-loss recovery for the tunnel section.
    No static .pgm map required.
  </p>
  <div class="cover-meta">
    <div class="cover-meta-item">
      <div class="label">Competition</div>
      <div class="value">October 1, 2026</div>
    </div>
    <div class="cover-meta-item">
      <div class="label">Aug 14 Demo</div>
      <div class="value">Autonomous Motion</div>
    </div>
    <div class="cover-meta-item">
      <div class="label">Platform</div>
      <div class="value">Jetson Orin Nano</div>
    </div>
    <div class="cover-meta-item">
      <div class="label">Team</div>
      <div class="value">Juggernauts</div>
    </div>
  </div>
  <div class="cover-stack">
    <span class="tag tag-cyan">ROS 2 Humble</span>
    <span class="tag tag-green">FAST-LIO2</span>
    <span class="tag tag-purple">NDT-OMP</span>
    <span class="tag tag-orange">Nav2 MPPI</span>
    <span class="tag tag-yellow">Dual EKF</span>
    <span class="tag tag-red">No PGM Map</span>
    <span class="tag tag-blue">Zone State Machine</span>
  </div>
</div>

<!-- ═══ TOC ═══ -->
<div class="page toc">
  <h2>Table of Contents</h2>
  <div class="toc-section"><a href="#s1"><span><span class="num">1</span> The Problem with a Static Map</span></a></div>
  <div class="toc-section"><a href="#s2"><span><span class="num">2</span> Localization Stack — No .pgm Needed</span></a></div>
  <div class="toc-section toc-sub"><a href="#s2-1"><span>2.1 LIO-SAM Offline Map Generation</span></a></div>
  <div class="toc-section toc-sub"><a href="#s2-2"><span>2.2 NDT-OMP Runtime Localization</span></a></div>
  <div class="toc-section toc-sub"><a href="#s2-3"><span>2.3 Dual EKF Fusion Architecture</span></a></div>
  <div class="toc-section"><a href="#s3"><span><span class="num">3</span> Costmap Strategy — Live Lidar Only</span></a></div>
  <div class="toc-section"><a href="#s4"><span><span class="num">4</span> Obstacle Course — Zone Analysis</span></a></div>
  <div class="toc-section"><a href="#s5"><span><span class="num">5</span> Zone-Aware Navigation State Machine</span></a></div>
  <div class="toc-section toc-sub"><a href="#s5-1"><span>5.1 NORMAL_NAV — Nav2 Smac + MPPI</span></a></div>
  <div class="toc-section toc-sub"><a href="#s5-2"><span>5.2 SLOW_NAV — Narrow, Ramp, Bank</span></a></div>
  <div class="toc-section toc-sub"><a href="#s5-3"><span>5.3 BLIND_DRIVE — Tunnel Dead Reckoning</span></a></div>
  <div class="toc-section toc-sub"><a href="#s5-4"><span>5.4 NAVIGATE_AROUND — Dynamic Obstacle Buckets</span></a></div>
  <div class="toc-section toc-sub"><a href="#s5-5"><span>5.5 PUSH_THROUGH — Car Wash</span></a></div>
  <div class="toc-section"><a href="#s6"><span><span class="num">6</span> Nav2 Configuration for This Course</span></a></div>
  <div class="toc-section"><a href="#s7"><span><span class="num">7</span> Implementation Priority</span></a></div>
</div>

<!-- ═══ SECTION 1 ═══ -->
<div class="page section" id="s1">
  <div class="section-header">
    <span class="section-num red">1</span>
    <h2>The Problem with a Static Map</h2>
  </div>

  <p>
    The standard Nav2 setup uses a <strong>static occupancy grid</strong> — a .pgm image file — loaded by
    <code>map_server</code> to give the global planner a pre-known picture of the environment. This works
    well indoors on flat surfaces. For our outdoor obstacle course it breaks down in several ways.
  </p>

  <div class="grid2">
    <div class="card">
      <div class="card-title"><span class="dot dot-red"></span>Why PCD → PGM Fails Outdoors</div>
      <p>LIO-SAM gives us a 3D point cloud (<code>GlobalMap.pcd</code>). Converting it to a 2D .pgm
      requires projecting down to a single horizontal slice. On our course:</p>
      <ul style="color:var(--subtext); padding-left:16px; margin-top:6px; font-size:10px; line-height:1.8;">
        <li>Ramps make the floor non-flat — a 2D slice shows the ramp as a wall</li>
        <li>Gravel returns are inconsistent — holes appear in the free space</li>
        <li>Bucket obstacles baked in at mapping time reappear at competition even after moving</li>
        <li>Tunnel ceiling returns appear as impassable walls in 2D</li>
        <li>Getting the map origin and orientation right is fiddly and error-prone</li>
      </ul>
    </div>
    <div class="card">
      <div class="card-title"><span class="dot dot-green"></span>What We Do Instead</div>
      <p>We skip the static layer entirely. Here's the insight:</p>
      <ul style="color:var(--subtext); padding-left:16px; margin-top:6px; font-size:10px; line-height:1.8;">
        <li><strong>Localization</strong> (where am I?) is solved by NDT-OMP matching live scans against <code>GlobalMap.pcd</code> directly — no 2D projection</li>
        <li><strong>Obstacle avoidance</strong> (what's around me?) is solved by the live lidar feeding the <code>obstacle_layer</code> in real time</li>
        <li><strong>Path planning</strong> works fine without a static layer using <code>allow_unknown: true</code></li>
        <li>Dynamic obstacles — like randomly placed buckets — are always current</li>
      </ul>
    </div>
  </div>

  <div class="callout callout-good">
    <div class="callout-title">Key Insight</div>
    The .pgm file was only ever needed to tell the planner where the walls are. Our lidar sees those walls live at 10 Hz.
    We already have better, fresher information — we just need to use it correctly.
  </div>
</div>

<!-- ═══ SECTION 2 ═══ -->
<div class="page section pagebreak" id="s2">
  <div class="section-header">
    <span class="section-num">2</span>
    <h2>Localization Stack — No .pgm Needed</h2>
  </div>

  <p>
    The full localization pipeline is shown below. It has three independent pieces that each answer a different question:
    NDT-OMP answers <em>where am I globally</em>, FAST-LIO2 answers <em>how am I moving</em>,
    and the dual EKF answers <em>what is my best smoothed estimate</em>.
  </p>

  <div class="diagram-container">
    <div class="diagram-label">Full Localization + Navigation Data Flow</div>
    {svg_localization}
  </div>

  <h3 id="s2-1">2.1 LIO-SAM — Offline Map Generation (done once before competition)</h3>
  <p>
    We drive the course once with LIO-SAM running. It builds a globally consistent 3D point cloud map
    using a factor graph with loop closure. When we're happy with the map quality in RViz, we call the
    <code>/lio_sam/save_map</code> service and get three files: <code>CornerMap.pcd</code>,
    <code>SurfMap.pcd</code>, and most importantly <code>GlobalMap.pcd</code> — the combined map
    that everything else works from.
  </p>
  <div class="callout callout-warn">
    <div class="callout-title">Map Quality is Foundation</div>
    The quality of this single mapping run determines how well NDT-OMP localises on competition day.
    Drive slowly, complete full loop closure, verify in RViz before saving. Build this map early — it's
    the single most important pre-competition task.
  </div>

  <h3 id="s2-2">2.2 NDT-OMP — Runtime Map Matching</h3>
  <p>
    NDT-OMP (Normal Distributions Transform with OpenMP parallelism) loads <code>GlobalMap.pcd</code>
    at startup and continuously matches each incoming Hesai scan against it. The output is the
    <strong>map→odom transform</strong> — this is what tells Nav2 where the robot is in the global frame.
    It publishes directly into the TF tree, so Nav2 doesn't need to know anything about how it works.
  </p>
  <p>
    This replaces AMCL entirely. Unlike AMCL, NDT-OMP works in 3D and doesn't need a 2D map.
    It is already in our repo as the <code>ndt_omp_ros2</code> submodule.
  </p>

  <h3 id="s2-3">2.3 Dual EKF Fusion</h3>
  <p>
    FAST-LIO2 runs at 10 Hz — once per lidar scan. Between scans there's a 100ms gap where the
    robot's pose estimate doesn't update. For the MPPI controller running at 20 Hz, this gap
    causes jerky velocity commands. The dual EKF eliminates this.
  </p>

  <div class="grid2">
    <div class="card">
      <div class="card-title"><span class="dot dot-orange"></span>EKF1 — High Rate Local (ekf_wimu.yaml)</div>
      <p>Fuses dead-wheel encoder odometry (<code>/wheel_cmd_vel</code>) with IMU angular velocity
      (<code>/imu/data</code>) to produce <code>/wimu_odom</code> at 50–100 Hz. This is fast,
      smooth, and fills the gaps between lidar updates. It drifts over time but is excellent over
      short intervals.</p>
    </div>
    <div class="card">
      <div class="card-title"><span class="dot dot-green"></span>EKF2 — Accurate Fused Output (ekf_local.yaml)</div>
      <p>Takes <code>/wimu_odom</code> (high-rate smooth) and <code>/lidar_odometry</code>
      (FAST-LIO2, accurate, lower rate) and fuses them into <code>/odometry/filtered</code>.
      The result is both smooth and accurate — MPPI gets 50 Hz smooth inputs, drift is corrected
      every lidar scan.</p>
    </div>
  </div>
</div>

<!-- ═══ SECTION 3 ═══ -->
<div class="page section pagebreak" id="s3">
  <div class="section-header">
    <span class="section-num green">3</span>
    <h2>Costmap Strategy — Live Lidar Only</h2>
  </div>

  <p>
    Nav2's costmap system has a modular plugin architecture. Each plugin contributes information
    to a shared 2D grid. We use <strong>only the live lidar</strong> — no static layer, no pre-built map.
    This is a deliberate design choice, not a compromise.
  </p>

  <div class="diagram-container">
    <div class="diagram-label">Costmap Layer Architecture</div>
    {svg_costmap}
  </div>

  <h3>How the Global Costmap Builds Up Without a Static Layer</h3>
  <p>
    The global costmap has <code>rolling_window: false</code> — it doesn't clear itself as the robot moves.
    Every obstacle the Hesai lidar sees gets marked in the grid and <strong>stays there</strong>.
    As the robot drives around the course, the costmap accumulates a complete picture of what's been observed.
    By the time the robot needs to plan a long path, it has already seen most of the environment.
  </p>
  <p>
    The planner is configured with <code>allow_unknown: true</code> — it will plan through unseen
    space if needed. On a fixed known course, this is safe because the course boundaries are
    quickly observed and marked.
  </p>

  <div class="grid3">
    <div class="card">
      <div class="card-title"><span class="dot dot-cyan"></span>obstacle_layer</div>
      <p>Subscribes to <code>/hesai/points</code>. Marks cells occupied when a point lands within
      <code>max_obstacle_height: 2.0m</code> and <code>obstacle_max_range: 5.0m</code>.
      Raycasts back to robot to clear free space. Updates at 10 Hz.</p>
    </div>
    <div class="card">
      <div class="card-title"><span class="dot dot-green"></span>inflation_layer</div>
      <p>Inflates every occupied cell by <code>inflation_radius: 0.45m</code> with a cost gradient.
      The planner naturally routes paths away from walls. MPPI uses the cost gradient for
      smooth avoidance.</p>
    </div>
    <div class="card">
      <div class="card-title"><span class="dot dot-red"></span>static_layer — removed</div>
      <p>Not used. This was the only plugin that required a <code>.pgm</code> file. With it gone,
      <code>map_server</code> still runs (for the map→odom TF chain) but loads nothing.
      Three lines removed from <code>nav2_params.yaml</code>.</p>
    </div>
  </div>

  <div class="callout callout-info">
    <div class="callout-title">Dynamic Obstacles are Better Than Static</div>
    The bucket obstacles in the obstacle section are <em>randomly placed</em> — they cannot be pre-mapped.
    The live obstacle_layer handles them perfectly. Buckets that moved between laps are automatically
    updated. This is actually better than a static map for this course.
  </div>
</div>

<!-- ═══ SECTION 4 ═══ -->
<div class="page section pagebreak" id="s4">
  <div class="section-header">
    <span class="section-num yellow">4</span>
    <h2>Obstacle Course — Zone Analysis</h2>
  </div>

  <p>
    The 2026 course is 65' × 48' and contains ten distinct sections, each posing a different challenge
    to the navigation stack. The map below shows each section colour-coded by the navigation mode it requires.
  </p>

  <div class="diagram-container">
    <div class="diagram-label">Course Zone Map — Navigation Modes</div>
    {SVG_COURSE_MAP}
  </div>

  <table>
    <thead>
      <tr>
        <th>Section</th>
        <th>Key Dimensions</th>
        <th>Primary Challenge</th>
        <th>FAST-LIO2 Status</th>
        <th>Navigation Mode</th>
      </tr>
    </thead>
    <tbody>
      <tr>
        <td><strong>Start / Finish</strong></td>
        <td>32" wide, flat</td>
        <td>Clean start, consistent positioning</td>
        <td>✅ Full</td>
        <td><span class="zone-badge zone-normal">NORMAL_NAV</span></td>
      </tr>
      <tr>
        <td><strong>Car Wash</strong></td>
        <td>48" wide</td>
        <td>Hanging ribbons register as lidar wall</td>
        <td>⚠️ Corrupted</td>
        <td><span class="zone-badge zone-push">PUSH_THROUGH</span></td>
      </tr>
      <tr>
        <td><strong>Hoop Section</strong></td>
        <td>36" wide</td>
        <td>Random hoop positions on path</td>
        <td>✅ Full</td>
        <td><span class="zone-badge zone-normal">NORMAL_NAV</span></td>
      </tr>
      <tr>
        <td><strong>Obstacle (buckets)</strong></td>
        <td>~11' × 26' open</td>
        <td>2–9 randomly placed 5-gallon buckets</td>
        <td>✅ Full</td>
        <td><span class="zone-badge zone-around">NAVIGATE_AROUND</span></td>
      </tr>
      <tr>
        <td><strong>Gravel + Ramps</strong></td>
        <td>48" wide, 2" ramp</td>
        <td>Surface noise, pitch change, wheel slip</td>
        <td>⚠️ Degraded</td>
        <td><span class="zone-badge zone-slow">SLOW_NAV</span></td>
      </tr>
      <tr>
        <td><strong>Bank Section</strong></td>
        <td>48" wide, 8.5° tilt</td>
        <td>Lateral roll corrupts 2D odometry</td>
        <td>⚠️ Minor</td>
        <td><span class="zone-badge zone-slow">SLOW_NAV</span></td>
      </tr>
      <tr>
        <td><strong>Narrow Section</strong></td>
        <td>20" wide, curved</td>
        <td>&lt;2" clearance — needs &lt;4cm lateral accuracy</td>
        <td>✅ Full</td>
        <td><span class="zone-badge zone-slow">SLOW_NAV</span></td>
      </tr>
      <tr>
        <td><strong>Tunnel</strong></td>
        <td>32" wide, 6' long</td>
        <td>Foil-lined foam — lidar and RF completely blocked</td>
        <td>❌ Dead</td>
        <td><span class="zone-badge zone-blind">BLIND_DRIVE</span></td>
      </tr>
      <tr>
        <td><strong>Ramp / Helix</strong></td>
        <td>32" wide, 4' radius, 11% grade</td>
        <td>Tight turn on grade, IMU pitch + roll</td>
        <td>⚠️ Degraded</td>
        <td><span class="zone-badge zone-slow">SLOW_NAV</span></td>
      </tr>
      <tr>
        <td><strong>Pothole / Wide</strong></td>
        <td>11' × 26'</td>
        <td>0.75" bumps, changeable boundaries</td>
        <td>✅ Full</td>
        <td><span class="zone-badge zone-normal">NORMAL_NAV</span></td>
      </tr>
    </tbody>
  </table>
</div>

<!-- ═══ SECTION 5 ═══ -->
<div class="page section pagebreak" id="s5">
  <div class="section-header">
    <span class="section-num orange">5</span>
    <h2>Zone-Aware Navigation State Machine</h2>
  </div>

  <p>
    The robot doesn't use a single navigation strategy for the whole course — that would mean
    compromising everywhere. Instead, a <strong>zone-aware state machine</strong> monitors the
    robot's position against pre-defined waypoint zones and switches navigation mode appropriately.
    Zones are defined as simple radius checks against known waypoints — no computer vision needed.
  </p>

  <div class="diagram-container">
    <div class="diagram-label">Navigation State Machine — Mode Transitions</div>
    {svg_state_machine}
  </div>

  <h3 id="s5-1">5.1 NORMAL_NAV — Full Nav2 Smac + MPPI</h3>
  <p>
    This is the default mode for open sections — pothole area, hoop section, wide open areas.
    Nav2's Smac Hybrid-A* planner generates a global path respecting the robot's turn radius.
    The MPPI controller samples thousands of trajectory rollouts and picks the one that best
    follows the path while avoiding obstacles. The full live costmap is active.
  </p>
  <p>
    Typical speed: <strong>0.5–0.6 m/s</strong>. The costmap accumulates obstacle data as the robot
    drives, so each subsequent run benefits from prior observations in the same session.
  </p>

  <h3 id="s5-2">5.2 SLOW_NAV — Reduced Speed for Sensitive Sections</h3>
  <p>
    The narrow section, ramps, bank, and helix all trigger SLOW_NAV. Same Nav2 stack, but with
    velocity limits lowered. The key concern in the narrow section is <strong>lateral accuracy</strong>
    — with only 2 inches of clearance on each side, we need NDT-OMP localization to be solid
    before entering. The state machine forces an NDT re-localization check at the zone entry waypoint
    before proceeding.
  </p>
  <p>
    Typical speed: <strong>0.2–0.3 m/s</strong>. IMU pitch threshold triggers the ramp detection automatically.
  </p>

  <h3 id="s5-3">5.3 BLIND_DRIVE — Tunnel Dead Reckoning</h3>
  <p>
    The tunnel is the hardest section. The foil-lined foam walls block both lidar returns and RF signals.
    FAST-LIO2 loses its input the moment the robot enters and can't recover until it exits.
    This is a known, fixed-length obstacle — we handle it with deliberate dead reckoning.
  </p>

  <div class="diagram-container">
    <div class="diagram-label">Tunnel Entry / Drive / Exit Sequence</div>
    {SVG_TUNNEL}
  </div>

  <div class="grid2">
    <div class="card">
      <div class="card-title"><span class="dot dot-red"></span>Entry Sequence</div>
      <ul style="color:var(--subtext); padding-left:16px; font-size:10px; line-height:1.9;">
        <li>Zone waypoint triggers BLIND_DRIVE mode</li>
        <li>Current map→odom TF is <strong>frozen</strong> (cached)</li>
        <li>FAST-LIO2 output is ignored</li>
        <li>EKF2 switches to wheel+IMU only (<code>/wimu_odom</code>)</li>
        <li>Robot drives straight at fixed heading and low speed</li>
      </ul>
    </div>
    <div class="card">
      <div class="card-title"><span class="dot dot-green"></span>Exit Sequence</div>
      <ul style="color:var(--subtext); padding-left:16px; font-size:10px; line-height:1.9;">
        <li>Tunnel exit zone waypoint detected (distance-based)</li>
        <li>Wait for FAST-LIO2 to produce a valid scan match</li>
        <li>NDT-OMP re-localises against <code>GlobalMap.pcd</code></li>
        <li>Map→odom TF updated with corrected pose</li>
        <li>Return to NORMAL_NAV or SLOW_NAV</li>
      </ul>
    </div>
  </div>

  <div class="callout callout-crit">
    <div class="callout-title">Critical — Implement First</div>
    Without BLIND_DRIVE, the robot will stop at the tunnel entrance when FAST-LIO2 drops out.
    This is the highest-priority navigation feature to implement after the Aug 14 demo.
  </div>

  <h3 id="s5-4">5.4 NAVIGATE_AROUND — Dynamic Bucket Obstacles</h3>
  <p>
    The obstacle section has 2–9 five-gallon buckets placed arbitrarily. These cannot be pre-mapped —
    they change position between runs. This is actually the section where our live-costmap approach
    shines brightest.
  </p>
  <p>
    In NAVIGATE_AROUND mode, the full obstacle_layer is active. Each Hesai scan marks the buckets
    as occupied cells. The Smac planner finds a path through the gaps — buckets placed so that a
    path always exists. MPPI follows the path while continuously reacting to the costmap.
    If a bucket is nudged by the robot, the costmap updates within one scan cycle (100ms).
  </p>
  <p>
    No special code is needed for this mode — it is exactly what stock Nav2 Smac + MPPI does.
    The key is making sure the robot enters the obstacle section with a good global pose so the
    initial Smac plan is in the right coordinate frame.
  </p>

  <h3 id="s5-5">5.5 PUSH_THROUGH — Car Wash</h3>
  <p>
    The car wash has densely hanging rubber ribbons. From the lidar's perspective, these look like
    a solid wall across the path. If the obstacle_layer is active, Nav2 will classify the entry as
    blocked and refuse to proceed.
  </p>
  <p>
    PUSH_THROUGH disables the obstacle_layer (or raises the <code>max_obstacle_height</code> threshold
    below the ribbon height) for the car wash zone. The robot drives straight through on a fixed
    heading using the pre-computed centerline waypoint. There's nothing to avoid — the ribbons are
    flexible and just brush past. This is the same approach as the tunnel but without the lidar blackout.
  </p>
</div>

<!-- ═══ SECTION 6 ═══ -->
<div class="page section pagebreak" id="s6">
  <div class="section-header">
    <span class="section-num purple">6</span>
    <h2>Nav2 Configuration for This Course</h2>
  </div>

  <p>
    The changes needed to <code>nav2_params.yaml</code> are minimal — three lines removed to drop
    the static layer, and one parameter confirmed. Everything else in the existing config is already
    well-suited to this course.
  </p>

  <h3>Global Costmap — Remove static_layer</h3>
  <pre>
# BEFORE (requires .pgm):
global_costmap:
  plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
  static_layer:
    plugin: "nav2_costmap_2d::StaticLayer"
    map_topic: /map
    map_subscribe_transient_local: true

# AFTER (no .pgm needed):
global_costmap:
  plugins: ["obstacle_layer", "inflation_layer"]
  # static_layer block removed entirely</pre>

  <h3>Key Parameters Already Correct</h3>
  <table>
    <thead>
      <tr><th>Parameter</th><th>Value</th><th>Why It Matters for This Course</th></tr>
    </thead>
    <tbody>
      <tr>
        <td><code>rolling_window: false</code></td>
        <td>global costmap</td>
        <td>Costmap accumulates the whole course as robot drives — critical for long-range planning</td>
      </tr>
      <tr>
        <td><code>allow_unknown: true</code></td>
        <td>Smac planner</td>
        <td>Robot can plan through unseen space at start — doesn't need full map before first goal</td>
      </tr>
      <tr>
        <td><code>inflation_radius: 0.45m</code></td>
        <td>global costmap</td>
        <td>With 20" narrow section and ~28" robot width, this is tight — may need tuning to 0.3m for narrow zones</td>
      </tr>
      <tr>
        <td><code>raytrace_max_range: 6.0m</code></td>
        <td>obstacle_layer</td>
        <td>Clears free space up to 6m ahead — keeps costmap from filling with stale marks</td>
      </tr>
      <tr>
        <td><code>motion_model: "DiffDrive"</code></td>
        <td>MPPI</td>
        <td>Correct for our differential drive robot — constrains rollouts to physically valid motions</td>
      </tr>
      <tr>
        <td><code>vx_max: 0.6</code></td>
        <td>MPPI</td>
        <td>Reasonable for obstacle course — increase for open sections, override per zone</td>
      </tr>
      <tr>
        <td><code>footprint: [[0.35,0.20],...]</code></td>
        <td>both costmaps</td>
        <td>Used for collision checking. Verify this matches actual robot dimensions before competition</td>
      </tr>
    </tbody>
  </table>

  <div class="callout callout-warn">
    <div class="callout-title">Inflation Radius vs Narrow Section</div>
    The current <code>inflation_radius: 0.45m</code> on the global costmap may mark the 20" narrow
    section as completely impassable — the walls are only ~10" from the robot sides. When entering
    the narrow zone, consider temporarily reducing inflation_radius to 0.2m via a dynamic reconfigure
    call from the state machine.
  </div>
</div>

<!-- ═══ SECTION 7 ═══ -->
<div class="page section pagebreak" id="s7">
  <div class="section-header">
    <span class="section-num green">7</span>
    <h2>Implementation Priority</h2>
  </div>

  <p>
    Nothing described in this document is implemented yet — this is the design. Here is the
    recommended build order based on dependency and competition risk.
  </p>

  <table>
    <thead>
      <tr><th>#</th><th>Feature</th><th>Why This Order</th><th>Target</th></tr>
    </thead>
    <tbody>
      <tr>
        <td><strong style="color:var(--red)">1</strong></td>
        <td><strong>Aug 14 Demo</strong> — autonomous straight → 90° turn → straight → stop</td>
        <td>Hard deadline. Robot executes under its own control using odometry feedback. Uses existing motion_plan_executor. Needs hardware running.</td>
        <td>Aug 14</td>
      </tr>
      <tr>
        <td><strong style="color:var(--red)">2</strong></td>
        <td><strong>LIO-SAM map generation</strong> — drive course, save GlobalMap.pcd</td>
        <td>Everything else depends on having a good map. Do this as early as possible.</td>
        <td>Aug–Sep</td>
      </tr>
      <tr>
        <td><strong style="color:var(--red)">3</strong></td>
        <td><strong>NDT-OMP integration</strong> — wire into localization.launch.py</td>
        <td>Replaces static map for localization. Required before any Nav2 competition run.</td>
        <td>Aug–Sep</td>
      </tr>
      <tr>
        <td><strong style="color:var(--orange)">4</strong></td>
        <td><strong>Remove static_layer</strong> from nav2_params.yaml</td>
        <td>One-line change. Unblocks Nav2 from requiring .pgm.</td>
        <td>Aug</td>
      </tr>
      <tr>
        <td><strong style="color:var(--orange)">5</strong></td>
        <td><strong>Dual EKF</strong> — add ekf_wimu.yaml, update ekf_local.yaml</td>
        <td>Smooth MPPI inputs. Needed before tuning MPPI on hardware.</td>
        <td>Aug–Sep</td>
      </tr>
      <tr>
        <td><strong style="color:var(--orange)">6</strong></td>
        <td><strong>BLIND_DRIVE mode</strong> — tunnel dead reckoning</td>
        <td>Without this, robot stops at tunnel. Critical for full-course run.</td>
        <td>Sep</td>
      </tr>
      <tr>
        <td><strong style="color:var(--yellow)">7</strong></td>
        <td><strong>Zone waypoint system</strong> — define all zone boundaries</td>
        <td>Required to trigger mode switches. Simple radius-check against known waypoints.</td>
        <td>Sep</td>
      </tr>
      <tr>
        <td><strong style="color:var(--yellow)">8</strong></td>
        <td><strong>Car wash costmap filter</strong> — height threshold override</td>
        <td>Without this, Nav2 refuses to enter car wash.</td>
        <td>Sep</td>
      </tr>
      <tr>
        <td><strong style="color:var(--yellow)">9</strong></td>
        <td><strong>Inflation radius override</strong> for narrow section</td>
        <td>May be required to navigate the 20" narrow section with current footprint.</td>
        <td>Sep–Oct</td>
      </tr>
      <tr>
        <td><strong style="color:var(--green)">10</strong></td>
        <td><strong>E-stop safety review</strong></td>
        <td>Required by Sep 25 — heartbeat, assertion, clearing demonstration.</td>
        <td>Sep 25</td>
      </tr>
    </tbody>
  </table>

  <div class="callout callout-info">
    <div class="callout-title">Milestone Check</div>
    By Aug 15: hardware running, FAST-LIO2 publishing, motion_plan_demo passing. That's the foundation.
    Everything in this document builds on top of a working robot. Get the robot moving first.
  </div>

  <hr class="divider"/>
  <p style="text-align:center; color: var(--muted); font-size:9px;">
    DIY Robot Challenge 2026 — Obstacle Course Navigation Design v1.0 — Team Juggernauts — July 2026
  </p>
</div>

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
