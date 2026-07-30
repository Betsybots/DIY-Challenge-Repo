#!/usr/bin/env python3
"""
Generate DIY_Robot_SAD.pdf — System Architecture Document
Uses Graphviz (dot) for diagrams → SVG, embeds in HTML, renders via WeasyPrint.
Updated: May 2026 — No RTK GPS, multi-level EKF, cuVSLAM stretch goal.
"""

import subprocess
import tempfile
import os
from pathlib import Path

# ─────────────────────────────────────────────────────────────────────────────
# GRAPHVIZ DIAGRAMS
# ─────────────────────────────────────────────────────────────────────────────

DIAGRAM_SYSTEM_OVERVIEW = r"""
digraph system_overview {
    rankdir=TB;
    bgcolor="transparent";
    node [fontname="Inter", fontsize=10, style="filled,rounded", shape=box];
    edge [fontname="Inter", fontsize=8, color="#64748b"];

    subgraph cluster_sensors {
        label="Physical Sensors"; labeljust=l; fontname="Inter"; fontsize=11; fontcolor="#0ea5e9";
        style="dashed"; color="#334155";
        lidar [label="Hesai QT64\n(64ch LiDAR)", fillcolor="#0c4a6e", fontcolor="#bae6fd"];
        imu [label="ACEINNA MTLT335D\n(Industrial 6-DOF IMU)", fillcolor="#0c4a6e", fontcolor="#bae6fd"];
        camera [label="RealSense D435i\n(Stereo + IMU)", fillcolor="#0c4a6e", fontcolor="#bae6fd"];
        encoders [label="Dead-Wheel Encoders\n(2x, non-driven)", fillcolor="#0c4a6e", fontcolor="#bae6fd"];
    }

    subgraph cluster_compute {
        label="Jetson Orin Nano 8GB (40 TOPS)"; labeljust=l; fontname="Inter"; fontsize=11; fontcolor="#22c55e";
        style="dashed"; color="#334155";

        subgraph cluster_drivers {
            label="Driver Layer"; labeljust=l; fontname="Inter"; fontsize=9; fontcolor="#94a3b8";
            style="filled"; fillcolor="#1a2332"; color="#334155";
            hesai_drv [label="hesai_ros_driver", fillcolor="#1e3a5f", fontcolor="#7dd3fc"];
            rs_drv [label="realsense2_ros", fillcolor="#1e3a5f", fontcolor="#7dd3fc"];
            imu_drv [label="ACEINNA driver\n(UART/RS232)", fillcolor="#1e3a5f", fontcolor="#7dd3fc"];
            enc_drv [label="encoder_driver\n(GPIO/SPI)", fillcolor="#1e3a5f", fontcolor="#7dd3fc"];
        }

        subgraph cluster_localization {
            label="Localization Layer"; labeljust=l; fontname="Inter"; fontsize=9; fontcolor="#94a3b8";
            style="filled"; fillcolor="#1a2332"; color="#334155";
            fastlio [label="FAST-LIO2\n(LiDAR-Inertial Odom)", fillcolor="#14532d", fontcolor="#bbf7d0"];
            ekf_local [label="EKF Local\n(robot_localization)", fillcolor="#14532d", fontcolor="#bbf7d0"];
            cuVSLAM [label="cuVSLAM\n(GPU Visual Odom)\n[STRETCH - July]", fillcolor="#3b0764", fontcolor="#e9d5ff", style="filled,rounded,dashed"];
            amcl [label="AMCL / NDT Match\nmap→odom transform", fillcolor="#14532d", fontcolor="#bbf7d0"];
        }

        subgraph cluster_perception {
            label="Perception Layer"; labeljust=l; fontname="Inter"; fontsize=9; fontcolor="#94a3b8";
            style="filled"; fillcolor="#1a2332"; color="#334155";
            pcl_node [label="PCL Obstacle\nClassifier", fillcolor="#431407", fontcolor="#fed7aa"];
            costmap [label="Costmap Manager\n(static + voxel + inflate)", fillcolor="#431407", fontcolor="#fed7aa"];
        }

        subgraph cluster_nav {
            label="Navigation Layer"; labeljust=l; fontname="Inter"; fontsize=9; fontcolor="#94a3b8";
            style="filled"; fillcolor="#1a2332"; color="#334155";
            planner [label="Nav2 Smac\nHybrid-A*", fillcolor="#312e81", fontcolor="#c7d2fe"];
            controller [label="Nav2 MPPI\nController", fillcolor="#312e81", fontcolor="#c7d2fe"];
        }

        subgraph cluster_mission {
            label="Mission Layer"; labeljust=l; fontname="Inter"; fontsize=9; fontcolor="#94a3b8";
            style="filled"; fillcolor="#1a2332"; color="#334155";
            state_machine [label="Mission State\nMachine", fillcolor="#4a1d1d", fontcolor="#fecaca"];
            behavior [label="Behavior\nSwitcher", fillcolor="#4a1d1d", fontcolor="#fecaca"];
        }
    }

    subgraph cluster_actuator {
        label="Actuation (STM32 + micro-ROS)"; labeljust=l; fontname="Inter"; fontsize=11; fontcolor="#f97316";
        style="dashed"; color="#334155";
        stm32 [label="STM32\n(E-Stop + CAN)", fillcolor="#431407", fontcolor="#fed7aa"];
        motors [label="FWD Motors\n(P-CAN)", fillcolor="#431407", fontcolor="#fed7aa"];
    }

    // Sensor → Driver edges
    lidar -> hesai_drv [label="Ethernet UDP"];
    imu -> imu_drv [label="UART/RS232"];
    camera -> rs_drv [label="USB 3.0"];
    encoders -> enc_drv [label="GPIO/SPI"];

    // Driver → Localization
    hesai_drv -> fastlio [label="/hesai/points"];
    imu_drv -> fastlio [label="/imu/data"];
    imu_drv -> ekf_local [label="/imu/data"];
    enc_drv -> ekf_local [label="/wheel/odom"];
    fastlio -> ekf_local [label="/lidar_odom\n(pose+twist)"];
    rs_drv -> cuVSLAM [label="stereo + IMU", style=dashed];
    cuVSLAM -> ekf_local [label="/visual_odom", style=dashed];
    amcl -> ekf_local [label="map→odom\ntransform", style=dotted];

    // Driver → Perception
    hesai_drv -> pcl_node [label="/hesai/points"];
    pcl_node -> costmap [label="/obstacle_class"];

    // Localization → Navigation
    ekf_local -> controller [label="/odom/filtered"];
    costmap -> planner;
    costmap -> controller;
    planner -> controller [label="global path"];

    // Mission control
    state_machine -> behavior;
    behavior -> costmap [label="layer enable/disable", style=dashed];
    state_machine -> planner [label="waypoint goals"];

    // Nav → Actuation
    controller -> stm32 [label="/cmd_vel\n(UART micro-ROS)"];
    stm32 -> motors [label="P-CAN frames"];
}
"""

DIAGRAM_EKF_FUSION = r"""
digraph ekf_fusion {
    rankdir=LR;
    bgcolor="transparent";
    node [fontname="Inter", fontsize=10, style="filled,rounded", shape=box];
    edge [fontname="Inter", fontsize=8, color="#64748b"];

    subgraph cluster_sources {
        label="Odometry Sources"; labeljust=l; fontname="Inter"; fontsize=10; fontcolor="#0ea5e9";
        style="dashed"; color="#334155";
        fastlio [label="FAST-LIO2\npose + twist\n10-20 Hz", fillcolor="#14532d", fontcolor="#bbf7d0"];
        encoders [label="Dead-Wheel Encoders\ntwist only\n50-100 Hz", fillcolor="#14532d", fontcolor="#bbf7d0"];
        imu [label="ACEINNA IMU\nangular vel + accel\n200 Hz", fillcolor="#14532d", fontcolor="#bbf7d0"];
        cuVSLAM [label="cuVSLAM (GPU)\npose + twist\n30 Hz\n[STRETCH]", fillcolor="#3b0764", fontcolor="#e9d5ff", style="filled,rounded,dashed"];
    }

    subgraph cluster_ekf {
        label="EKF Local (odom frame)"; labeljust=l; fontname="Inter"; fontsize=10; fontcolor="#22c55e";
        style="filled"; fillcolor="#1a2332"; color="#22c55e";
        ekf [label="robot_localization\nekf_local_node\n\nOutputs:\n/odom/filtered\nodom → base_link", fillcolor="#052e16", fontcolor="#bbf7d0", shape=record];
    }

    subgraph cluster_global {
        label="Global Localization (map frame)"; labeljust=l; fontname="Inter"; fontsize=10; fontcolor="#a855f7";
        style="filled"; fillcolor="#1a2332"; color="#a855f7";
        amcl [label="AMCL or NDT match\nagainst pre-built map\n\nPublishes:\nmap → odom transform", fillcolor="#2e1065", fontcolor="#e9d5ff", shape=record];
    }

    subgraph cluster_consumers {
        label="Consumers"; labeljust=l; fontname="Inter"; fontsize=10; fontcolor="#f97316";
        style="dashed"; color="#334155";
        nav2 [label="Nav2\nMPPI Controller", fillcolor="#431407", fontcolor="#fed7aa"];
        mission [label="Mission State\nMachine", fillcolor="#431407", fontcolor="#fed7aa"];
    }

    // Source → EKF
    fastlio -> ekf [label="weight: HIGH\npose+twist"];
    encoders -> ekf [label="weight: MED\ntwist only"];
    imu -> ekf [label="weight: MED\nprediction"];
    cuVSLAM -> ekf [label="weight: MED\npose+twist", style=dashed];

    // Global → TF
    amcl -> ekf [label="map→odom\ncorrection", style=dotted, color="#a855f7"];

    // EKF → Consumers
    ekf -> nav2 [label="/odom/filtered\nhigh-rate"];
    ekf -> mission [label="pose for\nzone detection"];
}
"""

DIAGRAM_TF_TREE = r"""
digraph tf_tree {
    rankdir=TB;
    bgcolor="transparent";
    node [fontname="Inter", fontsize=10, style="filled,rounded", shape=box];
    edge [fontname="Inter", fontsize=8, color="#64748b"];

    map [label="map", fillcolor="#2e1065", fontcolor="#e9d5ff"];
    odom [label="odom", fillcolor="#14532d", fontcolor="#bbf7d0"];
    base_link [label="base_link", fillcolor="#0c4a6e", fontcolor="#bae6fd"];
    lidar_link [label="lidar_link\n(Hesai QT64)", fillcolor="#1e3a5f", fontcolor="#7dd3fc"];
    imu_link [label="imu_link\n(ACEINNA MTLT335D)", fillcolor="#1e3a5f", fontcolor="#7dd3fc"];
    camera_link [label="camera_link\n(D435i)", fillcolor="#1e3a5f", fontcolor="#7dd3fc"];
    left_wheel [label="left_encoder_link", fillcolor="#1e3a5f", fontcolor="#7dd3fc"];
    right_wheel [label="right_encoder_link", fillcolor="#1e3a5f", fontcolor="#7dd3fc"];

    map -> odom [label="AMCL / NDT\n(map matching)"];
    odom -> base_link [label="EKF local\n(robot_localization)"];
    base_link -> lidar_link [label="static\n(extrinsic calib)"];
    base_link -> imu_link [label="static\n(extrinsic calib)"];
    base_link -> camera_link [label="static\n(extrinsic calib)"];
    base_link -> left_wheel [label="static\n(measured)"];
    base_link -> right_wheel [label="static\n(measured)"];
}
"""

DIAGRAM_ESTOP = r"""
digraph estop {
    rankdir=LR;
    bgcolor="transparent";
    node [fontname="Inter", fontsize=10, style="filled,rounded", shape=box];
    edge [fontname="Inter", fontsize=8, color="#64748b"];

    wired [label="Course Wired\nE-Stop", fillcolor="#450a0a", fontcolor="#fecaca"];
    wireless [label="Wireless E-Stop\n(RF, 300ft range)", fillcolor="#450a0a", fontcolor="#fecaca"];
    stm32 [label="STM32\nE-Stop Handler\n(GPIO interrupt)", fillcolor="#431407", fontcolor="#fed7aa", shape=record];
    can [label="CAN Bus\ncmd = 0", fillcolor="#1e3a5f", fontcolor="#7dd3fc"];
    motors [label="Motors\nSTOP", fillcolor="#0c4a6e", fontcolor="#bae6fd"];
    ros2 [label="ROS2 (advisory)\n/estop_active", fillcolor="#1a2332", fontcolor="#94a3b8", style="filled,rounded,dashed"];

    wired -> stm32 [label="GPIO\n(hardwired)"];
    wireless -> stm32 [label="GPIO\n(watchdog: 800ms)"];
    stm32 -> can [label="immediate"];
    can -> motors [label="zero velocity"];
    stm32 -> ros2 [label="optional\n(informational)", style=dashed];
}
"""

DIAGRAM_MISSION_STATES = r"""
digraph mission_states {
    rankdir=TB;
    bgcolor="transparent";
    node [fontname="Inter", fontsize=10, style="filled,rounded", shape=box];
    edge [fontname="Inter", fontsize=8, color="#64748b"];

    wait [label="WAIT_FOR_START", fillcolor="#1e3a5f", fontcolor="#bae6fd"];
    navigate [label="NAVIGATE_TO\nWAYPOINT", fillcolor="#14532d", fontcolor="#bbf7d0"];
    zone_entry [label="OBSTACLE_ZONE\nENTRY", fillcolor="#431407", fontcolor="#fed7aa"];
    behavior [label="BEHAVIOR_SWITCH\n(mode active)", fillcolor="#4a1d1d", fontcolor="#fecaca"];
    lap_done [label="LAP_COMPLETE", fillcolor="#2e1065", fontcolor="#e9d5ff"];
    stop [label="STOP\n(cmd_vel = 0)", fillcolor="#450a0a", fontcolor="#fecaca"];

    wait -> navigate [label="visual start\ndetected"];
    navigate -> zone_entry [label="zone proximity\nthreshold"];
    zone_entry -> behavior [label="classification\nresolved"];
    behavior -> navigate [label="zone exit\n(encoder dist)"];
    navigate -> lap_done [label="final waypoint\nreached"];
    lap_done -> navigate [label="lap < 2\n(restart)"];
    lap_done -> stop [label="lap == 2"];
}
"""

DIAGRAM_TIMELINE = r"""
digraph timeline {
    rankdir=LR;
    bgcolor="transparent";
    node [fontname="Inter", fontsize=9, style="filled,rounded", shape=box, width=2.2];
    edge [fontname="Inter", fontsize=8, color="#334155", style=bold];

    june [label="JUNE\nCalibration &\nMapping\n\n• IMU/LiDAR calib\n• Prior map\n• EKF local working", fillcolor="#0c4a6e", fontcolor="#bae6fd"];
    july [label="JULY\nPerception &\nNav Tuning\n\n• Nav2 path follow\n• PCL classifier\n• cuVSLAM (stretch)", fillcolor="#14532d", fontcolor="#bbf7d0"];
    aug [label="AUG 1-15\nMission Logic &\nIntegration\n\n• State machine\n• Full flow test\n• E-stop verified", fillcolor="#2e1065", fontcolor="#e9d5ff"];
    buffer [label="AUG 16 → SEP 30\n45-Day Buffer\n\n• Bug fixes only\n• Practice runs\n• Race-day runbook\n• NO NEW FEATURES", fillcolor="#450a0a", fontcolor="#fecaca"];
    comp [label="OCT 1\nCompetition\nDay", fillcolor="#431407", fontcolor="#fed7aa"];

    june -> july -> aug -> buffer -> comp;
}
"""


# ─────────────────────────────────────────────────────────────────────────────
# SVG RENDERING
# ─────────────────────────────────────────────────────────────────────────────

def render_dot_to_svg(dot_source: str) -> str:
    """Render a DOT string to SVG using the `dot` command."""
    result = subprocess.run(
        ['dot', '-Tsvg'],
        input=dot_source.encode(),
        capture_output=True,
        check=True
    )
    svg = result.stdout.decode()
    # Strip XML declaration and DOCTYPE for embedding
    lines = svg.split('\n')
    svg_start = next(i for i, l in enumerate(lines) if '<svg' in l)
    return '\n'.join(lines[svg_start:])


# ─────────────────────────────────────────────────────────────────────────────
# HTML TEMPLATE
# ─────────────────────────────────────────────────────────────────────────────

def build_html(diagrams: dict) -> str:
    return f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8"/>
<title>System Architecture Document — DIY Robot Challenge 2026</title>
<style>
@import url('https://fonts.googleapis.com/css2?family=Inter:wght@300;400;500;600;700&family=JetBrains+Mono:wght@400;500&display=swap');

:root {{
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
}}

@page {{
  size: A4;
  margin: 20mm 18mm 20mm 18mm;
}}

* {{ box-sizing: border-box; margin: 0; padding: 0; }}

body {{
  font-family: 'Inter', 'Segoe UI', sans-serif;
  background: var(--dark);
  color: var(--text);
  font-size: 11px;
  line-height: 1.6;
}}

/* ─── COVER ─── */
.cover {{
  page-break-after: always;
  min-height: 100vh;
  display: flex;
  flex-direction: column;
  justify-content: center;
  align-items: center;
  text-align: center;
  padding: 60px 40px;
  background: linear-gradient(160deg, #0f172a 0%, #0c1a2e 50%, #130f1a 100%);
}}
.cover-badge {{
  background: rgba(14,165,233,0.12);
  border: 1px solid var(--cyan);
  color: var(--cyan);
  padding: 5px 18px;
  border-radius: 20px;
  font-size: 10px;
  font-weight: 600;
  letter-spacing: 2px;
  text-transform: uppercase;
  margin-bottom: 28px;
}}
.cover h1 {{
  font-size: 34px;
  font-weight: 700;
  color: #fff;
  letter-spacing: -1px;
  line-height: 1.2;
  margin-bottom: 14px;
}}
.cover h1 span {{ color: var(--cyan); }}
.cover .subtitle {{
  font-size: 14px;
  color: var(--subtext);
  max-width: 580px;
  margin-bottom: 36px;
}}
.cover-meta {{
  display: grid;
  grid-template-columns: repeat(4, 1fr);
  gap: 14px;
  max-width: 680px;
  width: 100%;
  margin-bottom: 36px;
}}
.cover-meta-item {{
  background: rgba(255,255,255,0.04);
  border: 1px solid var(--border);
  border-radius: 8px;
  padding: 12px;
}}
.cover-meta-item .label {{ font-size: 9px; color: var(--muted); text-transform: uppercase; letter-spacing: 1px; margin-bottom: 3px; }}
.cover-meta-item .value {{ font-size: 11px; font-weight: 600; color: var(--text); }}
.cover-stack {{ display: flex; gap: 8px; flex-wrap: wrap; justify-content: center; }}
.tag {{ padding: 3px 10px; border-radius: 5px; font-size: 10px; font-weight: 500; }}
.tag-cyan   {{ background: rgba(14,165,233,0.15); color: var(--cyan); border: 1px solid rgba(14,165,233,0.3); }}
.tag-green  {{ background: rgba(34,197,94,0.15);  color: var(--green); border: 1px solid rgba(34,197,94,0.3); }}
.tag-purple {{ background: rgba(168,85,247,0.15); color: var(--purple); border: 1px solid rgba(168,85,247,0.3); }}
.tag-orange {{ background: rgba(249,115,22,0.15); color: var(--orange); border: 1px solid rgba(249,115,22,0.3); }}
.tag-yellow {{ background: rgba(234,179,8,0.15);  color: var(--yellow); border: 1px solid rgba(234,179,8,0.3); }}
.tag-red    {{ background: rgba(239,68,68,0.15);  color: var(--red);    border: 1px solid rgba(239,68,68,0.3); }}
.tag-blue   {{ background: rgba(99,102,241,0.15); color: var(--blue);   border: 1px solid rgba(99,102,241,0.3); }}
.tag-lime   {{ background: rgba(132,204,22,0.15); color: var(--lime);   border: 1px solid rgba(132,204,22,0.3); }}

/* ─── LAYOUT ─── */
.page {{ max-width: 900px; margin: 0 auto; padding: 32px 24px; }}
.pagebreak {{ page-break-before: always; }}

/* ─── HEADINGS ─── */
.section {{ margin-bottom: 36px; }}
.section-header {{
  display: flex; align-items: center; gap: 12px;
  margin-bottom: 20px; padding-bottom: 10px;
  border-bottom: 1px solid var(--border);
}}
.section-num {{
  background: var(--cyan); color: var(--dark);
  font-size: 10px; font-weight: 700;
  padding: 3px 9px; border-radius: 5px;
  min-width: 32px; text-align: center;
}}
.section-num.green  {{ background: var(--green); }}
.section-num.purple {{ background: var(--purple); }}
.section-num.orange {{ background: var(--orange); }}
.section-num.yellow {{ background: var(--yellow); }}
.section-num.red    {{ background: var(--red); }}
.section-num.blue   {{ background: var(--blue); }}
.section-num.lime   {{ background: var(--lime); }}
.section h2 {{ font-size: 18px; font-weight: 700; color: #fff; }}
h3 {{ font-size: 14px; font-weight: 600; color: var(--text); margin: 20px 0 8px; }}
h4 {{ font-size: 12px; font-weight: 600; color: var(--subtext); margin: 14px 0 6px; text-transform: uppercase; letter-spacing: 0.5px; }}
p {{ color: var(--subtext); margin-bottom: 10px; }}
p strong {{ color: var(--text); font-weight: 600; }}

/* ─── CARDS ─── */
.card {{
  background: var(--card); border: 1px solid var(--border);
  border-radius: 8px; padding: 16px; margin-bottom: 12px;
}}
.card-title {{
  font-size: 12px; font-weight: 600; color: #fff;
  margin-bottom: 6px; display: flex; align-items: center; gap: 7px;
}}
.dot {{ width: 7px; height: 7px; border-radius: 50%; flex-shrink: 0; }}
.dot-cyan   {{ background: var(--cyan); }}
.dot-green  {{ background: var(--green); }}
.dot-purple {{ background: var(--purple); }}
.dot-orange {{ background: var(--orange); }}
.dot-red    {{ background: var(--red); }}
.dot-blue   {{ background: var(--blue); }}

/* ─── TABLES ─── */
table {{ width: 100%; border-collapse: collapse; margin: 12px 0; font-size: 10px; }}
thead tr {{ background: rgba(255,255,255,0.06); }}
th {{ padding: 8px 10px; text-align: left; font-weight: 600; color: var(--text); border-bottom: 1px solid var(--border); font-size: 9px; text-transform: uppercase; letter-spacing: 0.5px; }}
td {{ padding: 7px 10px; color: var(--subtext); border-bottom: 1px solid rgba(255,255,255,0.04); vertical-align: top; }}
td strong {{ color: var(--text); }}
td code {{ font-family: 'JetBrains Mono', monospace; font-size: 9px; background: rgba(255,255,255,0.08); padding: 1px 4px; border-radius: 3px; color: var(--cyan); }}

/* ─── CODE ─── */
pre {{
  background: #0a0e1a; border: 1px solid var(--border);
  border-left: 3px solid var(--cyan); border-radius: 6px;
  padding: 12px 16px; font-family: 'JetBrains Mono', monospace;
  font-size: 9.5px; color: #a0c4ff; overflow-x: auto;
  margin: 10px 0; line-height: 1.6; white-space: pre;
}}

/* ─── CALLOUTS ─── */
.callout {{
  border-radius: 6px; padding: 12px 14px;
  margin: 12px 0; font-size: 10.5px; border-left: 3px solid;
}}
.callout-warn  {{ background: rgba(234,179,8,0.08);  border-color: var(--yellow); color: #fde68a; }}
.callout-info  {{ background: rgba(14,165,233,0.08); border-color: var(--cyan);   color: #bae6fd; }}
.callout-crit  {{ background: rgba(239,68,68,0.08);  border-color: var(--red);    color: #fecaca; }}
.callout-good  {{ background: rgba(34,197,94,0.08);  border-color: var(--green);  color: #bbf7d0; }}
.callout .callout-title {{ font-weight: 700; margin-bottom: 3px; font-size: 10px; text-transform: uppercase; letter-spacing: 0.8px; }}

/* ─── GRID ─── */
.grid2 {{ display: grid; grid-template-columns: 1fr 1fr; gap: 12px; margin: 12px 0; }}
.grid3 {{ display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 12px; margin: 12px 0; }}

/* ─── DIAGRAMS ─── */
.diagram-container {{
  background: rgba(255,255,255,0.02);
  border: 1px solid var(--border);
  border-radius: 8px;
  padding: 16px;
  margin: 14px 0;
  text-align: center;
  overflow: hidden;
}}
.diagram-container svg {{
  max-width: 100%;
  height: auto;
}}
.diagram-label {{
  font-size: 9px;
  color: var(--muted);
  text-transform: uppercase;
  letter-spacing: 1px;
  margin-bottom: 8px;
}}

/* ─── DIVIDER ─── */
.divider {{ border: none; border-top: 1px solid var(--border); margin: 28px 0; }}

/* ─── TOC ─── */
.toc {{ page-break-after: always; }}
.toc h2 {{ font-size: 20px; font-weight: 700; color: #fff; margin-bottom: 20px; padding-bottom: 10px; border-bottom: 1px solid var(--border); }}
.toc-section {{ margin-bottom: 6px; }}
.toc-section a {{ color: var(--subtext); text-decoration: none; display: flex; justify-content: space-between; padding: 4px 8px; border-radius: 4px; font-size: 11px; }}
.toc-section .num {{ color: var(--cyan); font-weight: 600; margin-right: 8px; font-size: 10px; }}
.toc-sub {{ padding-left: 24px; }}
.toc-sub a {{ font-size: 10px; }}

/* ─── DECISION BOX ─── */
.decision {{
  background: rgba(239,68,68,0.05);
  border: 1px solid rgba(239,68,68,0.2);
  border-radius: 8px;
  padding: 14px;
  margin: 12px 0;
}}
.decision-header {{
  font-weight: 700; font-size: 11px;
  color: var(--red); margin-bottom: 6px;
  text-transform: uppercase; letter-spacing: 0.5px;
}}
.decision-rejected {{
  background: rgba(239,68,68,0.05);
  border: 1px solid rgba(239,68,68,0.2);
}}
.decision-accepted {{
  background: rgba(34,197,94,0.05);
  border: 1px solid rgba(34,197,94,0.2);
}}
.decision-accepted .decision-header {{ color: var(--green); }}

/* ─── PRINT ─── */
@media print {{
  body {{ background: #fff; color: #111827; }}
  .cover {{ background: #0f172a !important; }}
  .cover h1, .cover h1 span, .cover .subtitle {{ color: #e2e8f0 !important; }}
  h2, h3, h4 {{ color: #111827; }}
  .section h2 {{ color: #111827 !important; }}
  .card {{ background: #f8fafc; border-color: #e2e8f0; }}
  .card-title {{ color: #111827 !important; }}
  p {{ color: #374151; }}
  td {{ color: #374151; }}
  pre {{ background: #f1f5f9; color: #1e293b !important; }}
}}
</style>
</head>
<body>

<!-- ═══════════════════════════════ COVER ═══════════════════════════════ -->
<div class="cover">
  <div class="cover-badge">System Architecture Document v2.0</div>
  <h1>DIY Robot Challenge<br/><span>2026 — System Architecture</span></h1>
  <p class="subtitle">Full technical architecture for an autonomous outdoor obstacle-course robot. ROS 2 Humble on Jetson Orin Nano, multi-level EKF fusion (FAST-LIO2 + dead-wheel encoders + ACEINNA IMU), Nav2 navigation, no GPS dependency.</p>
  <div class="cover-meta">
    <div class="cover-meta-item">
      <div class="label">Competition</div>
      <div class="value">October 1, 2026</div>
    </div>
    <div class="cover-meta-item">
      <div class="label">Feature Lock</div>
      <div class="value">August 15, 2026</div>
    </div>
    <div class="cover-meta-item">
      <div class="label">Version</div>
      <div class="value">2.0 — May 2026</div>
    </div>
    <div class="cover-meta-item">
      <div class="label">Team</div>
      <div class="value">Betsybots</div>
    </div>
  </div>
  <div class="cover-stack">
    <span class="tag tag-cyan">ROS 2 Humble</span>
    <span class="tag tag-green">FAST-LIO2</span>
    <span class="tag tag-purple">Nav2</span>
    <span class="tag tag-orange">Hesai QT64</span>
    <span class="tag tag-blue">RealSense D435i</span>
    <span class="tag tag-yellow">ACEINNA MTLT335D</span>
    <span class="tag tag-lime">Dead-Wheel Encoders</span>
    <span class="tag tag-red">Jetson Orin Nano 8GB</span>
  </div>
</div>

<!-- ═══════════════════════════════ TOC ═══════════════════════════════ -->
<div class="page toc">
  <h2>Table of Contents</h2>
  <div class="toc-section"><a href="#s1"><span><span class="num">1</span> System Overview &amp; Goals</span></a></div>
  <div class="toc-section"><a href="#s2"><span><span class="num">2</span> Hardware Architecture &amp; Sensors</span></a></div>
  <div class="toc-section"><a href="#s3"><span><span class="num">3</span> Compute Architecture — Jetson Orin Nano + STM32</span></a></div>
  <div class="toc-section"><a href="#s4"><span><span class="num">4</span> ROS 2 Software Stack</span></a></div>
  <div class="toc-sub">
    <div class="toc-section"><a href="#s4-1"><span>4.1 Driver Layer</span></a></div>
    <div class="toc-section"><a href="#s4-2"><span>4.2 Localization — Multi-Level EKF (No GPS)</span></a></div>
    <div class="toc-section"><a href="#s4-3"><span>4.3 Perception — PCL Obstacle Classifier</span></a></div>
    <div class="toc-section"><a href="#s4-4"><span>4.4 Navigation — Nav2 (Smac + MPPI)</span></a></div>
    <div class="toc-section"><a href="#s4-5"><span>4.5 Mission Layer — State Machine</span></a></div>
  </div>
  <div class="toc-section"><a href="#s5"><span><span class="num">5</span> Multi-Level EKF Fusion Design</span></a></div>
  <div class="toc-section"><a href="#s6"><span><span class="num">6</span> GPU Acceleration &amp; cuVSLAM</span></a></div>
  <div class="toc-section"><a href="#s7"><span><span class="num">7</span> Offline Pre-Competition Pipeline</span></a></div>
  <div class="toc-section"><a href="#s8"><span><span class="num">8</span> Obstacle Handling Strategy</span></a></div>
  <div class="toc-section"><a href="#s9"><span><span class="num">9</span> Safety &amp; E-Stop Architecture</span></a></div>
  <div class="toc-section"><a href="#s10"><span><span class="num">10</span> Sensor Calibration Procedures</span></a></div>
  <div class="toc-section"><a href="#s11"><span><span class="num">11</span> Development Timeline</span></a></div>
  <div class="toc-section"><a href="#s12"><span><span class="num">12</span> Key Decisions &amp; Rejected Alternatives</span></a></div>
  <div class="toc-section"><a href="#s13"><span><span class="num">13</span> Risks &amp; Mitigations</span></a></div>
</div>

<!-- ═══════════════════════════════ SECTION 1 ═══════════════════════════════ -->
<div class="page section pagebreak" id="s1">
  <div class="section-header">
    <span class="section-num">01</span>
    <h2>System Overview &amp; Goals</h2>
  </div>

  <h3>1.1 Competition Context</h3>
  <p>The CAT Robotics Culture Club DIY Robot Challenge 2026 is an autonomous outdoor obstacle-course race. Teams build custom robots competing on a <strong>speed course</strong> (3 laps) and an <strong>obstacle course</strong> (2 laps). Scoring is time-based with cumulative penalties for skipped obstacles.</p>

  <table>
    <thead><tr><th>Parameter</th><th>Value</th><th>Notes</th></tr></thead>
    <tbody>
      <tr><td><strong>Max dimensions</strong></td><td>16" W × 24" L × 16" H</td><td>Hard limit</td></tr>
      <tr><td><strong>Max weight</strong></td><td>25 lbs</td><td>Including all electronics</td></tr>
      <tr><td><strong>Obstacle course</strong></td><td>2 consecutive laps</td><td>Timed, best heat used</td></tr>
      <tr><td><strong>Skip penalty</strong></td><td>15s × n per skip</td><td>Cumulative — skipping 5 = 15×15 = 225s total</td></tr>
      <tr><td><strong>Min track width</strong></td><td>20 inches</td><td>Robot is 16" — only 2" clearance each side</td></tr>
      <tr><td><strong>Heat duration</strong></td><td>10 minutes</td><td>Multiple runs per heat</td></tr>
      <tr><td><strong>Communication</strong></td><td>None during run (Rule 2.4)</td><td>E-Stop exempt</td></tr>
    </tbody>
  </table>

  <h3>1.2 Design Goals</h3>
  <div class="grid2">
    <div class="card">
      <div class="card-title"><span class="dot dot-green"></span>MVP — Reliable 2-Lap Completion</div>
      <p>Complete both laps of the obstacle course within 10 minutes using correct behaviors (push-through vs navigate-around) for each obstacle type. Zero GPS dependency — localization entirely from LiDAR + IMU + encoders against pre-built map.</p>
    </div>
    <div class="card">
      <div class="card-title"><span class="dot dot-cyan"></span>Stretch — Sub-4-Minute Run</div>
      <p>Zero obstacle skips, consistent 2-lap completion under 4 minutes with sub-5cm localization accuracy at critical waypoints. cuVSLAM provides visual-inertial redundancy in case of LiDAR degradation.</p>
    </div>
  </div>

  <h3>1.3 Key Changes from v1.0 Architecture</h3>
  <table>
    <thead><tr><th>Component</th><th>v1.0 (Previous)</th><th>v2.0 (Current)</th><th>Reason</th></tr></thead>
    <tbody>
      <tr><td><strong>GPS</strong></td><td>RTK GNSS ($2500 BX992)</td><td>REJECTED — No GPS</td><td>$2500 + base station + correction service complexity. LiDAR localization sufficient.</td></tr>
      <tr><td><strong>Compute</strong></td><td>Jetson Nano (4GB)</td><td>Jetson Orin Nano (8GB, 40 TOPS)</td><td>10× GPU, 2× RAM, same form factor. Enables cuVSLAM.</td></tr>
      <tr><td><strong>IMU</strong></td><td>Generic 6-DOF</td><td>ACEINNA MTLT335D (industrial)</td><td>1.3°/hr bias stability, CAN/RS232. Dramatically better drift.</td></tr>
      <tr><td><strong>Wheel Odom</strong></td><td>Removed (slip issues)</td><td>Dead-wheel encoders (non-driven)</td><td>Non-driven wheels don't slip. Fills gaps between LiDAR scans.</td></tr>
      <tr><td><strong>Global Loc</strong></td><td>RTK EKF2 + navsat_transform</td><td>AMCL/NDT match against pre-built map</td><td>No GPS means map-matching provides map→odom.</td></tr>
      <tr><td><strong>GPU Odom</strong></td><td>None</td><td>cuVSLAM (stretch, July)</td><td>Free GPU cycles on Orin — visual odom as redundancy.</td></tr>
      <tr><td><strong>Feature Lock</strong></td><td>Sep 15</td><td>Aug 15 (45-day buffer)</td><td>More practice time before competition.</td></tr>
    </tbody>
  </table>
</div>

<!-- ═══════════════════════════════ SECTION 2 ═══════════════════════════════ -->
<div class="page section pagebreak" id="s2">
  <div class="section-header">
    <span class="section-num green">02</span>
    <h2>Hardware Architecture &amp; Sensors</h2>
  </div>

  <div class="diagram-container">
    <div class="diagram-label">Figure 1 — Full System Architecture</div>
    {diagrams['system_overview']}
  </div>

  <h3>2.1 Sensor Inventory</h3>
  <table>
    <thead><tr><th>Component</th><th>Model</th><th>Interface</th><th>Role</th><th>Notes</th></tr></thead>
    <tbody>
      <tr><td><strong>3D LiDAR</strong></td><td>Hesai QT64</td><td>Ethernet (UDP)</td><td>FAST-LIO2 odom, PCL classifier, costmap</td><td>64ch, ±52° VFOV, ~20Hz</td></tr>
      <tr><td><strong>IMU</strong></td><td>ACEINNA MTLT335D</td><td>UART / RS232</td><td>FAST-LIO2 fusion, EKF prediction</td><td>Industrial 6-DOF, 1.3°/hr gyro bias, CAN/RS232</td></tr>
      <tr><td><strong>Camera</strong></td><td>RealSense D435i</td><td>USB 3.0</td><td>cuVSLAM (stretch), AprilTag calib, visual start</td><td>Stereo + built-in IMU. On-sensor depth ASIC.</td></tr>
      <tr><td><strong>Encoders</strong></td><td>Dead-wheel (×2)</td><td>GPIO / SPI</td><td>Twist input to EKF, fills LiDAR gaps</td><td>Non-driven — no slip. Mounted on passive wheels.</td></tr>
      <tr><td><strong>Compute</strong></td><td>Jetson Orin Nano 8GB</td><td>—</td><td>All ROS 2 stack</td><td>40 TOPS GPU, 6 ARM cores, 8GB RAM</td></tr>
      <tr><td><strong>Microcontroller</strong></td><td>STM32</td><td>UART (micro-ROS)</td><td>Motor control, E-Stop</td><td>Hard real-time, CAN bus master</td></tr>
      <tr><td><strong>Motors</strong></td><td>FWD Controllers</td><td>P-CAN</td><td>Drive + steering</td><td>Via STM32 CAN bridge</td></tr>
      <tr><td><strong>E-Stop (Wired)</strong></td><td>Course-provided</td><td>GPIO → STM32</td><td>Safety stop (Rule 1.2.2)</td><td>Hardwired, bypasses Jetson</td></tr>
      <tr><td><strong>E-Stop (Wireless)</strong></td><td>Team RF unit</td><td>RF → STM32</td><td>300ft range (Rule 1.2.4)</td><td>Fail-safe: stop if signal lost >1s</td></tr>
    </tbody>
  </table>

  <div class="callout callout-info">
    <div class="callout-title">Why Dead-Wheel Encoders Instead of Motor Encoders?</div>
    FWD motor encoders measure wheel rotation — but on gravel, inclines, and turns the driven wheels slip, producing false odometry that corrupts the EKF. Dead-wheel encoders are mounted on passive, non-driven wheels that maintain ground contact without slip. They provide reliable twist (dx, dy, dθ) at 50-100 Hz that fills the gap between 10-20 Hz LiDAR scans.
  </div>
</div>

<!-- ═══════════════════════════════ SECTION 3 ═══════════════════════════════ -->
<div class="page section pagebreak" id="s3">
  <div class="section-header">
    <span class="section-num purple">03</span>
    <h2>Compute Architecture</h2>
  </div>

  <h3>3.1 Responsibilities Split</h3>
  <div class="grid2">
    <div class="card">
      <div class="card-title"><span class="dot dot-orange"></span>STM32 — Real-Time Layer</div>
      <ul style="color: var(--subtext); padding-left: 14px; margin-top: 6px; font-size: 10px;">
        <li>CAN bus: <code>/cmd_vel</code> → motor frames</li>
        <li>E-Stop: wired + wireless → cmd=0 within 1s</li>
        <li>Watchdog: wireless signal lost >800ms → auto-stop</li>
        <li>micro-ROS bridge over UART to Jetson</li>
      </ul>
    </div>
    <div class="card">
      <div class="card-title"><span class="dot dot-green"></span>Jetson Orin Nano — ROS 2 Stack</div>
      <ul style="color: var(--subtext); padding-left: 14px; margin-top: 6px; font-size: 10px;">
        <li>All sensor drivers (LiDAR, IMU, camera, encoders)</li>
        <li>FAST-LIO2 LiDAR-inertial odometry (CPU)</li>
        <li>robot_localization EKF (CPU)</li>
        <li>Nav2 planner + controller (CPU)</li>
        <li>cuVSLAM visual odometry (GPU — stretch)</li>
        <li>Mission state machine + behavior switcher</li>
      </ul>
    </div>
  </div>

  <h3>3.2 Compute Load Estimate</h3>
  <table>
    <thead><tr><th>Process</th><th>CPU %</th><th>GPU %</th><th>RAM</th><th>Notes</th></tr></thead>
    <tbody>
      <tr><td><code>FAST-LIO2</code></td><td>~20%</td><td>0%</td><td>~400MB</td><td>Primary odom</td></tr>
      <tr><td><code>Nav2 (planner + MPPI)</code></td><td>~18%</td><td>0%</td><td>~300MB</td><td>Main CPU consumer</td></tr>
      <tr><td><code>PCL classifier</code></td><td>~8%</td><td>0%</td><td>~150MB</td><td>Per-scan processing</td></tr>
      <tr><td><code>hesai_ros_driver</code></td><td>~4%</td><td>0%</td><td>~100MB</td><td>UDP parse only</td></tr>
      <tr><td><code>realsense2_ros</code></td><td>~5%</td><td>0%</td><td>~200MB</td><td>Depth on-sensor ASIC</td></tr>
      <tr><td><code>cuVSLAM</code> (stretch)</td><td>~2%</td><td>~25%</td><td>~300MB</td><td>Runs entirely on GPU</td></tr>
      <tr><td><code>robot_localization</code></td><td>~2%</td><td>0%</td><td>~50MB</td><td>Lightweight EKF</td></tr>
      <tr><td><strong>Total</strong></td><td><strong>~60%</strong></td><td><strong>~25%</strong></td><td><strong>~1.5GB</strong></td><td>Plenty of headroom</td></tr>
    </tbody>
  </table>

  <div class="callout callout-good">
    <div class="callout-title">Orin Nano Advantage</div>
    With 6 ARM cores and 8GB RAM, the Orin Nano has ~40% CPU headroom vs the old Jetson Nano's ~15%. The 40 TOPS GPU is almost entirely idle without cuVSLAM — making it a free resource for stretch goals.
  </div>
</div>

<!-- ═══════════════════════════════ SECTION 4 ═══════════════════════════════ -->
<div class="page section pagebreak" id="s4">
  <div class="section-header">
    <span class="section-num orange">04</span>
    <h2>ROS 2 Software Stack</h2>
  </div>

  <h3 id="s4-1">4.1 Driver Layer</h3>
  <table>
    <thead><tr><th>Node</th><th>Input</th><th>Output Topic</th><th>Rate</th></tr></thead>
    <tbody>
      <tr><td><code>hesai_ros_driver</code></td><td>Ethernet UDP</td><td><code>/hesai/points</code> (PointCloud2)</td><td>~20 Hz</td></tr>
      <tr><td><code>aceinna_driver</code></td><td>UART/RS232</td><td><code>/imu/data</code> (Imu)</td><td>200 Hz</td></tr>
      <tr><td><code>realsense2_ros</code></td><td>USB 3.0</td><td><code>/camera/color/image_raw</code>, <code>/camera/depth/...</code></td><td>30 Hz</td></tr>
      <tr><td><code>encoder_driver</code></td><td>GPIO/SPI</td><td><code>/wheel/odom</code> (Odometry, twist only)</td><td>50-100 Hz</td></tr>
      <tr><td><code>micro_ros_agent</code></td><td>UART (STM32)</td><td>relay of <code>/cmd_vel</code></td><td>on demand</td></tr>
      <tr><td><code>joy_node</code></td><td>USB gamepad</td><td><code>/cmd_vel</code> (mapping only)</td><td>50 Hz</td></tr>
    </tbody>
  </table>

  <hr class="divider"/>

  <h3 id="s4-2">4.2 Localization — Multi-Level EKF (No GPS)</h3>
  <p>With RTK GPS rejected, localization relies entirely on <strong>FAST-LIO2 + dead-wheel encoders + ACEINNA IMU</strong> fused in a local EKF, with AMCL/NDT providing the global map→odom correction.</p>

  <div class="diagram-container">
    <div class="diagram-label">Figure 2 — EKF Fusion Architecture</div>
    {diagrams['ekf_fusion']}
  </div>

  <h4>FAST-LIO2 — Primary Odometry</h4>
  <table>
    <thead><tr><th>Property</th><th>Value</th></tr></thead>
    <tbody>
      <tr><td>Algorithm</td><td>Iterative Extended Kalman Filter (iEKF) on point-to-plane</td></tr>
      <tr><td>Inputs</td><td><code>/hesai/points</code> + <code>/imu/data</code></td></tr>
      <tr><td>Output</td><td><code>/lidar_odom</code> (Odometry) @ 10-20 Hz</td></tr>
      <tr><td>Drift (calibrated)</td><td>0.1–0.3% of distance traveled</td></tr>
      <tr><td>Tunnel behavior</td><td>Rich wall features — no degradation</td></tr>
    </tbody>
  </table>

  <h4>EKF Local — robot_localization</h4>
  <pre>
Inputs:
  /lidar_odom    (FAST-LIO2, pose+twist, 10-20Hz)  — weight: HIGH
  /wheel/odom    (encoders, twist only, 50-100Hz)   — weight: MEDIUM
  /imu/data      (ACEINNA, angular vel, 200Hz)      — weight: MEDIUM (prediction)
  /visual_odom   (cuVSLAM, pose+twist, 30Hz)       — weight: MEDIUM [STRETCH]

Output:  /odom/filtered  (high-rate fused odometry)
Frame:   odom → base_link
Purpose: Feeds Nav2 MPPI controller — low latency, high rate</pre>

  <h4>Global Localization — AMCL / NDT Match</h4>
  <pre>
Method:  Match live LiDAR scans against pre-built map
Output:  map → odom transform (corrects accumulated drift)
Rate:    ~1-5 Hz (matching frequency)
Purpose: Provides global consistency without GPS
Note:    No navsat_transform needed — purely map-based</pre>

  <hr class="divider"/>

  <h3 id="s4-3">4.3 Perception — PCL Obstacle Classifier</h3>
  <pre>
Pipeline (per LiDAR scan, ~20Hz):
  1. Ground removal     → RANSAC plane fit, remove points within 5cm
  2. Clustering         → Euclidean extraction (tolerance: 0.15m)
  3. Map differencing   → Compare vs prior_map.pcd (new obstacles only)
  4. Classification     → Per-cluster: CHIMES / MOVABLE / TUNNEL / WALL
  Output: /obstacle_class (zone_id, classification, confidence)</pre>

  <hr class="divider"/>

  <h3 id="s4-4">4.4 Navigation — Nav2</h3>
  <div class="grid2">
    <div class="card">
      <div class="card-title"><span class="dot dot-purple"></span>Global: Smac Hybrid-A*</div>
      <p>Non-holonomic path planning respecting turn radius. Pre-computed from centerline waypoints. Re-invoked only on WALL classification.</p>
    </div>
    <div class="card">
      <div class="card-title"><span class="dot dot-orange"></span>Local: MPPI Controller</div>
      <p>Samples thousands of trajectory rollouts. Smooth velocity profiles on gravel/ramps. Better than DWB for dynamic obstacles.</p>
    </div>
  </div>

  <hr class="divider"/>

  <h3 id="s4-5">4.5 Mission Layer — State Machine</h3>
  <div class="diagram-container">
    <div class="diagram-label">Figure 3 — Mission State Machine</div>
    {diagrams['mission_states']}
  </div>

  <table>
    <thead><tr><th>Behavior Mode</th><th>Trigger</th><th>Actions</th><th>Exit</th></tr></thead>
    <tbody>
      <tr><td><strong>PUSH_THROUGH</strong></td><td>TUNNEL, CHIMES</td><td>Disable voxel costmap, hold heading, 0.2 m/s</td><td>Encoder distance past zone</td></tr>
      <tr><td><strong>NAVIGATE_AROUND</strong></td><td>MOVABLE object</td><td>Keep costmap, re-plan path around</td><td>Past obstacle + 0.5m</td></tr>
      <tr><td><strong>BLIND_DRIVE</strong></td><td>CHIMES (fallback)</td><td>All obstacle layers off, straight, 0.15 m/s</td><td>Distance > zone_width</td></tr>
      <tr><td><strong>NORMAL_NAV</strong></td><td>Default</td><td>Full Nav2 active</td><td>Zone entry detected</td></tr>
    </tbody>
  </table>
</div>

<!-- ═══════════════════════════════ SECTION 5 ═══════════════════════════════ -->
<div class="page section pagebreak" id="s5">
  <div class="section-header">
    <span class="section-num green">05</span>
    <h2>Multi-Level EKF Fusion Design</h2>
  </div>

  <p>This is the core localization architecture. With no GPS, all global consistency comes from map matching. The EKF fuses multiple odometry sources for high-rate, low-latency local pose estimates.</p>

  <h3>5.1 Source Characteristics</h3>
  <table>
    <thead><tr><th>Source</th><th>Provides</th><th>Rate</th><th>Strength</th><th>Weakness</th></tr></thead>
    <tbody>
      <tr><td><strong>FAST-LIO2</strong></td><td>Full 6DOF pose + twist</td><td>10-20 Hz</td><td>Best absolute accuracy (0.1-0.3% drift)</td><td>Degrades in featureless areas, lower rate</td></tr>
      <tr><td><strong>Dead-wheel encoders</strong></td><td>2D twist (dx, dy, dθ)</td><td>50-100 Hz</td><td>High rate, fills LiDAR gaps, no slip</td><td>2D only, no vertical, accumulates drift</td></tr>
      <tr><td><strong>ACEINNA IMU</strong></td><td>Angular vel + linear accel</td><td>200 Hz</td><td>Highest rate, prediction between updates</td><td>Double-integration drift (mitigated by 1.3°/hr bias)</td></tr>
      <tr><td><strong>cuVSLAM</strong> (stretch)</td><td>Full 6DOF pose + twist</td><td>30 Hz</td><td>GPU-only (0% CPU), independent modality</td><td>Fails in darkness/texture-poor areas</td></tr>
    </tbody>
  </table>

  <h3>5.2 EKF Configuration</h3>
  <pre>
# ekf_local.yaml — robot_localization config
ekf_local_node:
  ros__parameters:
    frequency: 50.0
    two_d_mode: false
    publish_tf: true
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    # FAST-LIO2: pose + twist (highest weight)
    odom0: /lidar_odom
    odom0_config: [true, true, true,    # x, y, z
                   true, true, true,    # roll, pitch, yaw
                   true, true, true,    # vx, vy, vz
                   true, true, true,    # vroll, vpitch, vyaw
                   false, false, false] # ax, ay, az

    # Dead-wheel encoders: twist only
    odom1: /wheel/odom
    odom1_config: [false, false, false,
                   false, false, false,
                   true, true, false,    # vx, vy (2D twist)
                   false, false, true,   # vyaw
                   false, false, false]

    # ACEINNA IMU: angular velocity + linear acceleration
    imu0: /imu/data
    imu0_config: [false, false, false,
                  false, false, false,
                  false, false, false,
                  true, true, true,     # vroll, vpitch, vyaw
                  true, true, true]     # ax, ay, az

    # cuVSLAM (stretch): pose + twist
    # odom2: /visual_odom
    # odom2_config: [true, true, true, true, true, true,
    #                true, true, true, true, true, true,
    #                false, false, false]</pre>

  <h3>5.3 TF Tree</h3>
  <div class="diagram-container">
    <div class="diagram-label">Figure 4 — TF Frame Tree</div>
    {diagrams['tf_tree']}
  </div>

  <div class="callout callout-crit">
    <div class="callout-title">Critical — No navsat_transform</div>
    With GPS removed, there is NO EKF2/navsat_transform. The map→odom transform comes from AMCL or NDT matching the live scan against the pre-built map. This is simpler but requires a high-quality prior map. Map building is the #1 priority in June.
  </div>

  <h3>5.4 Accuracy Expectations (No GPS)</h3>
  <table>
    <thead><tr><th>Scenario</th><th>Expected Accuracy</th><th>Notes</th></tr></thead>
    <tbody>
      <tr><td>FAST-LIO2 alone, calibrated</td><td>5–15cm per full lap</td><td>Outdoor, mixed terrain</td></tr>
      <tr><td>+ Encoders + IMU in EKF</td><td>3–8cm per lap</td><td>Encoder twist fills scan gaps</td></tr>
      <tr><td>+ AMCL map matching (open area)</td><td>&lt;5cm absolute</td><td>Continuous drift correction</td></tr>
      <tr><td>+ cuVSLAM redundancy (stretch)</td><td>&lt;3cm absolute</td><td>Independent visual confirmation</td></tr>
      <tr><td>Inside tunnel (no GPS anyway)</td><td>~5–10cm drift over length</td><td>Rich wall features for LiDAR</td></tr>
    </tbody>
  </table>

  <div class="callout callout-warn">
    <div class="callout-title">Narrow Passage Requirement</div>
    Track width 20" − robot 16" = 4" total clearance (2" per side = 5cm). Need lateral accuracy &lt;4cm at passage entry. AMCL correction immediately before narrow passages provides this. Failsafe: slow to 0.1 m/s and hold heading through passage.
  </div>
</div>

<!-- ═══════════════════════════════ SECTION 6 ═══════════════════════════════ -->
<div class="page section pagebreak" id="s6">
  <div class="section-header">
    <span class="section-num blue">06</span>
    <h2>GPU Acceleration &amp; cuVSLAM</h2>
  </div>

  <p>The Jetson Orin Nano's 40 TOPS GPU is largely idle in the base configuration. This section documents what can (and cannot) be offloaded to GPU.</p>

  <h3>6.1 CUDA Offloading Reality Check</h3>
  <table>
    <thead><tr><th>Component</th><th>CUDA Accelerated?</th><th>Reality</th></tr></thead>
    <tbody>
      <tr><td><strong>Hesai LiDAR driver</strong></td><td>No</td><td>Just UDP packet parsing — trivial CPU load (~4%)</td></tr>
      <tr><td><strong>RealSense driver</strong></td><td>No</td><td>Depth computed on-sensor ASIC, not host CPU</td></tr>
      <tr><td><strong>Isaac ROS image_proc</strong></td><td><strong>Yes</strong></td><td>GPU rectification/resize of camera images</td></tr>
      <tr><td><strong>Isaac ROS depth_image_proc</strong></td><td><strong>Yes</strong></td><td>GPU depth→pointcloud conversion</td></tr>
      <tr><td><strong>cuVSLAM</strong></td><td><strong>Yes</strong></td><td>Full visual SLAM on GPU — zero CPU cost</td></tr>
      <tr><td><strong>Nvblox</strong></td><td><strong>Yes</strong></td><td>3D costmap on GPU (replaces CPU costmap2D)</td></tr>
      <tr><td><strong>FAST-LIO2</strong></td><td>No</td><td>CPU-only iEKF — no CUDA path exists</td></tr>
      <tr><td><strong>Nav2 planner/controller</strong></td><td>No</td><td>CPU-only — no CUDA path exists</td></tr>
      <tr><td><strong>PCL/NDT</strong></td><td>Partial</td><td>ndt_omp uses OpenMP (CPU threads), not CUDA</td></tr>
    </tbody>
  </table>

  <div class="callout callout-info">
    <div class="callout-title">Bottom Line</div>
    The raw sensor drivers are NOT CPU-heavy — there's nothing meaningful to offload. The real GPU value comes from cuVSLAM (visual odometry) and potentially Nvblox (3D costmap). FAST-LIO2 and Nav2 remain CPU-bound regardless.
  </div>

  <h3>6.2 cuVSLAM Integration Plan (July Stretch Goal)</h3>
  <table>
    <thead><tr><th>Step</th><th>Action</th><th>Time Estimate</th></tr></thead>
    <tbody>
      <tr><td>1</td><td>Install Isaac ROS cuVSLAM package (apt)</td><td>30 min</td></tr>
      <tr><td>2</td><td>Configure for D435i stereo + built-in IMU</td><td>1 hour</td></tr>
      <tr><td>3</td><td>Verify <code>/visual_odom</code> output in RViz</td><td>1 hour</td></tr>
      <tr><td>4</td><td>Add as <code>odom2</code> in ekf_local.yaml</td><td>30 min</td></tr>
      <tr><td>5</td><td>Test: disable FAST-LIO, run on cuVSLAM alone</td><td>1 hour</td></tr>
      <tr><td>6</td><td>Tune relative weights in EKF</td><td>2 hours</td></tr>
    </tbody>
  </table>

  <div class="callout callout-warn">
    <div class="callout-title">DO NOT touch until July</div>
    cuVSLAM is a stretch goal. The base stack (FAST-LIO + encoders + IMU) must work perfectly first. Adding cuVSLAM prematurely introduces debugging complexity. Only add it when the base EKF is producing clean, consistent laps.
  </div>

  <h3>6.3 Nvblox — 3D Costmap (August Stretch)</h3>
  <p>If cuVSLAM works well in July, consider Nvblox in Aug 1-15. Nvblox builds a GPU-accelerated 3D TSDF/ESDF costmap from depth images. Benefits: better obstacle representation than 2D voxel layer, handles overhanging obstacles. Risk: adds complexity near feature lock. <strong>Only pursue if specific obstacle types require 3D reasoning.</strong></p>
</div>

<!-- ═══════════════════════════════ SECTION 7 ═══════════════════════════════ -->
<div class="page section pagebreak" id="s7">
  <div class="section-header">
    <span class="section-num lime">07</span>
    <h2>Offline Pre-Competition Pipeline</h2>
  </div>

  <h3>7.1 Phase 1 — Prior Map Building (June)</h3>
  <table>
    <thead><tr><th>Step</th><th>Action</th><th>Output</th></tr></thead>
    <tbody>
      <tr><td>1</td><td>Joystick drive — 2 full laps at 0.3 m/s (clean track, no obstacles)</td><td>rosbag</td></tr>
      <tr><td>2</td><td>Record: <code>/hesai/points /imu/data /wheel/odom /camera/... /tf /tf_static</code></td><td>~2GB bag</td></tr>
      <tr><td>3</td><td>Offline LIO-SAM processing (loop closure on lap 2)</td><td><code>prior_map.pcd</code></td></tr>
      <tr><td>4</td><td>octomap_server → 2D occupancy grid</td><td><code>static_map.yaml</code></td></tr>
      <tr><td>5</td><td>Voronoi skeletonization → centerline waypoints</td><td><code>waypoints.yaml</code></td></tr>
      <tr><td>6</td><td>Manual zone boundary annotation</td><td><code>zone_template.yaml</code></td></tr>
    </tbody>
  </table>

  <h3>7.2 Phase 2 — Day-Before Classification</h3>
  <table>
    <thead><tr><th>Step</th><th>Action</th><th>Output</th></tr></thead>
    <tbody>
      <tr><td>1</td><td>Slow scan lap with obstacles present (0.2 m/s)</td><td>day-of rosbag</td></tr>
      <tr><td>2</td><td>Map diff: subtract prior_map.pcd (threshold 0.1m)</td><td>new obstacle points</td></tr>
      <tr><td>3</td><td>Zone classifier: cluster + classify per zone</td><td>behavior per zone</td></tr>
      <tr><td>4</td><td>Generate <code>zone_config.yaml</code>, verify in RViz</td><td>final config</td></tr>
      <tr><td>5</td><td>Deploy to robot, verify full stack startup</td><td>ready for race</td></tr>
    </tbody>
  </table>

  <h3>7.3 AprilTag EKF Tuning (Pre-Competition Test Track)</h3>
  <p>8-10 AprilTags at surveyed positions in your own test track provide 6DOF ground truth. Run 10+ laps → compute Innovation-based Adaptive EKF Q/R matrices. NOT placed on competition course — purely a calibration tool.</p>
</div>

<!-- ═══════════════════════════════ SECTION 8 ═══════════════════════════════ -->
<div class="page section pagebreak" id="s8">
  <div class="section-header">
    <span class="section-num red">08</span>
    <h2>Obstacle Handling Strategy</h2>
  </div>

  <table>
    <thead><tr><th>Obstacle</th><th>Detection</th><th>Challenge</th><th>Behavior</th></tr></thead>
    <tbody>
      <tr><td><strong>Tunnel</strong></td><td>Zone boundary + bounded walls</td><td>No sky (irrelevant — no GPS anyway). Rich LiDAR features.</td><td>PUSH_THROUGH</td></tr>
      <tr><td><strong>Ramp</strong></td><td>Zone + IMU pitch change</td><td>Surface change. FAST-LIO2 handles natively.</td><td>NORMAL_NAV (slow)</td></tr>
      <tr><td><strong>Chimes</strong></td><td>PCL: thin vertical clusters</td><td>Ground-touching, can't scan below.</td><td>PUSH_THROUGH / BLIND_DRIVE</td></tr>
      <tr><td><strong>Movable</strong></td><td>PCL: isolated dense + map diff</td><td>Nudging corrupts lap-2 map.</td><td>NAVIGATE_AROUND</td></tr>
      <tr><td><strong>Narrow passage</strong></td><td>Zone + walls close on sides</td><td>2" clearance. Need &lt;4cm lateral accuracy.</td><td>PUSH_THROUGH (slow)</td></tr>
      <tr><td><strong>Gravel</strong></td><td>Zone + IMU vibration</td><td>Dead-wheel encoders handle surface change.</td><td>NORMAL_NAV (slower)</td></tr>
    </tbody>
  </table>
</div>

<!-- ═══════════════════════════════ SECTION 9 ═══════════════════════════════ -->
<div class="page section pagebreak" id="s9">
  <div class="section-header">
    <span class="section-num red">09</span>
    <h2>Safety &amp; E-Stop Architecture</h2>
  </div>

  <div class="diagram-container">
    <div class="diagram-label">Figure 5 — E-Stop Data Flow</div>
    {diagrams['estop']}
  </div>

  <div class="callout callout-crit">
    <div class="callout-title">Critical Design Principle</div>
    The E-Stop path NEVER passes through ROS 2 or the Jetson. If the Jetson crashes, the E-Stop still works. STM32 handles safety independently via hardware interrupts.
  </div>

  <table>
    <thead><tr><th>Requirement</th><th>Rule</th><th>Implementation</th></tr></thead>
    <tbody>
      <tr><td>Stop within 1 second</td><td>1.2.7.2</td><td>STM32 GPIO interrupt → CAN cmd=0 immediately</td></tr>
      <tr><td>Wireless signal loss → stop</td><td>1.2.7.1</td><td>800ms watchdog timer on STM32</td></tr>
      <tr><td>Course wired E-Stop</td><td>1.2.2</td><td>Hardwired to STM32, bypasses Jetson</td></tr>
      <tr><td>Team wireless, 300ft</td><td>1.2.4</td><td>RF receiver → STM32 GPIO</td></tr>
    </tbody>
  </table>
</div>

<!-- ═══════════════════════════════ SECTION 10 ═══════════════════════════════ -->
<div class="page section pagebreak" id="s10">
  <div class="section-header">
    <span class="section-num yellow">10</span>
    <h2>Sensor Calibration Procedures</h2>
  </div>

  <table>
    <thead><tr><th>#</th><th>Procedure</th><th>Tool</th><th>Output</th><th>Priority</th></tr></thead>
    <tbody>
      <tr><td>1</td><td>IMU intrinsic calibration (2-3hr stationary)</td><td><code>imu_utils</code></td><td>Noise density, bias instability → FAST-LIO2 config</td><td>Week 1</td></tr>
      <tr><td>2</td><td>IMU–LiDAR extrinsic (figure-8 motion)</td><td><code>lidar_imu_calib</code></td><td>6DOF transform → URDF + FAST-LIO2</td><td>Week 1</td></tr>
      <tr><td>3</td><td>Camera–LiDAR extrinsic</td><td><code>cam_lidar_calib</code></td><td>Transform → URDF</td><td>Week 2</td></tr>
      <tr><td>4</td><td>Encoder wheel diameter + baseline</td><td>Physical measurement</td><td>Tick-to-meters conversion</td><td>Week 1</td></tr>
      <tr><td>5</td><td>IMU–LiDAR time sync verification</td><td>Visual check (smeared scans)</td><td>Timestamp offset correction</td><td>Week 2</td></tr>
      <tr><td>6</td><td>EKF Q/R tuning via AprilTag ground truth</td><td>Innovation covariance method</td><td><code>competition.yaml</code></td><td>Late June</td></tr>
    </tbody>
  </table>

  <div class="callout callout-crit">
    <div class="callout-title">Calibration Order Matters</div>
    Each step depends on the previous. IMU intrinsics affect FAST-LIO2 accuracy, which affects extrinsic calibration quality. Do NOT skip to software development until steps 1-4 are complete.
  </div>
</div>

<!-- ═══════════════════════════════ SECTION 11 ═══════════════════════════════ -->
<div class="page section pagebreak" id="s11">
  <div class="section-header">
    <span class="section-num cyan" style="background: var(--cyan); color: var(--dark);">11</span>
    <h2>Development Timeline</h2>
  </div>

  <div class="diagram-container">
    <div class="diagram-label">Figure 6 — Development Roadmap (Aug 15 Feature Lock)</div>
    {diagrams['timeline']}
  </div>

  <table>
    <thead><tr><th>Phase</th><th>Dates</th><th>Deliverables</th><th>Gate</th></tr></thead>
    <tbody>
      <tr><td><strong>Calibration &amp; Mapping</strong></td><td>June 1–30</td><td>All calibrations done. Prior map built. EKF local producing clean odom. TF tree verified.</td><td>Clean 1-lap odom in RViz</td></tr>
      <tr><td><strong>Perception &amp; Nav</strong></td><td>July 1–31</td><td>Nav2 path following. PCL classifier working. cuVSLAM added (stretch). E-stop tested.</td><td>3 clean autonomous laps</td></tr>
      <tr><td><strong>Mission &amp; Integration</strong></td><td>Aug 1–15</td><td>State machine. Behavior switcher. Full startup→run→stop flow. Obstacle handling end-to-end.</td><td>Full 2-lap obstacle run</td></tr>
      <tr><td><strong style="color:var(--red)">FEATURE LOCK</strong></td><td><strong>Aug 15</strong></td><td colspan="2"><strong>No new features after this date. Only bug fixes and practice runs for 45 days.</strong></td></tr>
      <tr><td><strong>Bug Fixes &amp; Practice</strong></td><td>Aug 16 – Sep 30</td><td>Multiple full-course dry runs. Race-day runbook. Backup procedures. Hardware spares.</td><td>5 consecutive clean runs</td></tr>
      <tr><td><strong>Competition</strong></td><td>Oct 1</td><td>Day-before scan + classify. Race morning: stack verify. Trust the system.</td><td>—</td></tr>
    </tbody>
  </table>
</div>

<!-- ═══════════════════════════════ SECTION 12 ═══════════════════════════════ -->
<div class="page section pagebreak" id="s12">
  <div class="section-header">
    <span class="section-num orange">12</span>
    <h2>Key Decisions &amp; Rejected Alternatives</h2>
  </div>

  <div class="decision decision-rejected">
    <div class="decision-header">❌ Rejected: RTK GPS (BX992 — $2,500)</div>
    <p><strong>Rationale:</strong> $2,500 receiver + $1,000+/yr correction service + base station setup + antenna integration. LiDAR-based localization (FAST-LIO2 + AMCL) achieves &lt;5cm accuracy without any of this infrastructure. GPS degrades near buildings/trees (common on outdoor courses). The tunnel obstacle makes GPS useless for a significant portion of the track anyway.</p>
    <p><strong>What replaces it:</strong> AMCL/NDT matching against pre-built map provides map→odom transform. Dead-wheel encoders provide high-rate twist. Combined with FAST-LIO2's excellent accuracy, no GPS needed.</p>
  </div>

  <div class="decision decision-accepted">
    <div class="decision-header">✅ Accepted: Dead-Wheel Encoders</div>
    <p><strong>Rationale:</strong> Non-driven wheels don't slip — unlike motor encoders on FWD. Provides reliable 50-100 Hz twist that fills the 50-100ms gaps between FAST-LIO2 LiDAR updates. Critical for smooth MPPI controller inputs. Low cost (~$50-100 for two rotary encoders + mounting).</p>
  </div>

  <div class="decision decision-accepted">
    <div class="decision-header">✅ Accepted: ACEINNA MTLT335D Industrial IMU</div>
    <p><strong>Rationale:</strong> 1.3°/hr gyro bias stability (vs >10°/hr for consumer IMUs). CAN/RS232 interface (no USB latency jitter). Temperature-compensated. The single largest factor in FAST-LIO2 accuracy is IMU quality — this is worth the cost.</p>
  </div>

  <div class="decision decision-accepted">
    <div class="decision-header">✅ Accepted: cuVSLAM as Stretch Goal (July)</div>
    <p><strong>Rationale:</strong> Runs entirely on Orin Nano GPU (0% CPU impact). Provides independent visual-inertial odometry from D435i stereo. If FAST-LIO2 degrades (featureless area, dust), cuVSLAM maintains localization. 2-3 hour integration once base stack works.</p>
  </div>
</div>

<!-- ═══════════════════════════════ SECTION 13 ═══════════════════════════════ -->
<div class="page section pagebreak" id="s13">
  <div class="section-header">
    <span class="section-num red">13</span>
    <h2>Risks &amp; Mitigations</h2>
  </div>

  <table>
    <thead><tr><th>Risk</th><th>Likelihood</th><th>Severity</th><th>Mitigation</th></tr></thead>
    <tbody>
      <tr><td>AMCL map match fails (featureless area)</td><td>Low</td><td>High</td><td>EKF local continues with FAST-LIO2+encoders. Drift is only ~5-15cm/lap without global correction. Acceptable for most obstacles.</td></tr>
      <tr><td>Dead-wheel encoder mounting loses contact</td><td>Low</td><td>Medium</td><td>Spring-loaded mount. EKF gracefully degrades — FAST-LIO2 still provides primary odom.</td></tr>
      <tr><td>Chimes classified as wall (stops robot)</td><td>Medium</td><td>High</td><td>BLIND_DRIVE_MODE as guaranteed fallback — bypasses all perception.</td></tr>
      <tr><td>cuVSLAM destabilizes EKF</td><td>Low</td><td>Medium</td><td>Add in July only. Test extensively. Keep as odom2 with lower weight. Easy to disable if problematic.</td></tr>
      <tr><td>Narrow passage lateral drift &gt;4cm</td><td>Medium</td><td>High</td><td>Force AMCL correction before passage entry. Slow to 0.1 m/s. Hold centerline heading.</td></tr>
      <tr><td>Prior map quality poor</td><td>Medium</td><td>High</td><td>Build map early (June). Multiple re-recordings. Verify loop closure quality. This is the foundation — invest time here.</td></tr>
      <tr><td>TF tree errors found late</td><td>Low</td><td>High</td><td>Build and verify URDF in Week 1. Run tf2_echo on all pairs before any odom work.</td></tr>
    </tbody>
  </table>

  <hr class="divider"/>

  <h3>Package Reference</h3>
  <table>
    <thead><tr><th>Package</th><th>Purpose</th></tr></thead>
    <tbody>
      <tr><td><code>FAST-LIO</code></td><td>LiDAR-IMU odometry (primary)</td></tr>
      <tr><td><code>robot_localization</code></td><td>EKF fusion (local frame)</td></tr>
      <tr><td><code>nav2_bringup</code></td><td>Navigation stack</td></tr>
      <tr><td><code>nav2_smac_planner</code></td><td>Hybrid-A* global planner</td></tr>
      <tr><td><code>nav2_mppi_controller</code></td><td>MPPI local controller</td></tr>
      <tr><td><code>LIO-SAM</code></td><td>Offline SLAM for prior map</td></tr>
      <tr><td><code>isaac_ros_visual_slam</code></td><td>cuVSLAM (stretch)</td></tr>
      <tr><td><code>pcl_ros</code></td><td>PCL bindings</td></tr>
      <tr><td><code>imu_utils</code></td><td>IMU Allan variance calibration</td></tr>
      <tr><td><code>lidar_imu_calib</code></td><td>LiDAR-IMU extrinsic</td></tr>
      <tr><td><code>micro_ros_agent</code></td><td>STM32 bridge</td></tr>
    </tbody>
  </table>
</div>

<!-- FOOTER -->
<div class="page" style="padding-top: 16px; border-top: 1px solid var(--border);">
  <p style="text-align:center; color: var(--muted); font-size: 9px;">
    DIY Robot Challenge 2026 — System Architecture Document v2.0 — May 2026<br/>
    Team Betsybots — Feature Lock: Aug 15 — Competition: Oct 1<br/>
    Generated with Graphviz + WeasyPrint
  </p>
</div>

</body>
</html>"""


# ─────────────────────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────────────────────

def main():
    output_dir = Path(__file__).parent
    pdf_path = output_dir / "DIY_Robot_SAD.pdf"
    html_path = output_dir / "diy-sad-v2.html"

    print("[1/3] Rendering Graphviz diagrams to SVG...")
    diagrams = {
        'system_overview': render_dot_to_svg(DIAGRAM_SYSTEM_OVERVIEW),
        'ekf_fusion': render_dot_to_svg(DIAGRAM_EKF_FUSION),
        'tf_tree': render_dot_to_svg(DIAGRAM_TF_TREE),
        'estop': render_dot_to_svg(DIAGRAM_ESTOP),
        'mission_states': render_dot_to_svg(DIAGRAM_MISSION_STATES),
        'timeline': render_dot_to_svg(DIAGRAM_TIMELINE),
    }
    print(f"    ✓ {len(diagrams)} diagrams rendered")

    print("[2/3] Building HTML document...")
    html = build_html(diagrams)
    html_path.write_text(html)
    print(f"    ✓ HTML written to: {html_path}")

    print("[3/3] Generating PDF with WeasyPrint...")
    from weasyprint import HTML
    HTML(filename=str(html_path)).write_pdf(str(pdf_path))
    print(f"    ✓ PDF written to: {pdf_path}")

    # Clean up intermediate HTML if desired (keep for debugging)
    # html_path.unlink()

    print(f"\n[OK] {pdf_path}")


if __name__ == '__main__':
    main()
