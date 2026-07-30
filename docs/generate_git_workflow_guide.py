#!/usr/bin/env python3
"""
generate_git_workflow_guide.py
Produces docs/Git_Workflow_Guide.pdf — developer-facing git management reference.
Run:  python3 docs/generate_git_workflow_guide.py
"""

import os
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import cm
from reportlab.lib import colors
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.enums import TA_LEFT, TA_CENTER
from reportlab.platypus import (
    SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle,
    PageBreak, HRFlowable, ListFlowable, ListItem,
)

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_PATH  = os.path.join(REPO_ROOT, 'docs', 'Git_Workflow_Guide.pdf')

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

# ─── Styles ──────────────────────────────────────────────────────────────────
_n = [0]
def st(name, **kw):
    _n[0] += 1
    return ParagraphStyle(f'{name}_{_n[0]}', **kw)

TITLE  = st('T', fontName='Helvetica-Bold', fontSize=24, leading=30, textColor=NAVY, spaceAfter=4)
H1     = st('H1', fontName='Helvetica-Bold', fontSize=16, leading=22, textColor=NAVY, spaceBefore=18, spaceAfter=6)
H2     = st('H2', fontName='Helvetica-Bold', fontSize=13, leading=18, textColor=BLUE, spaceBefore=14, spaceAfter=4)
BODY   = st('BD', fontName='Helvetica', fontSize=10.5, leading=15, textColor=DGREY, spaceAfter=4)
CODE   = st('CD', fontName='Courier', fontSize=9.5, leading=14, textColor=NAVY, backColor=LGREY,
            leftIndent=12, rightIndent=12, spaceBefore=4, spaceAfter=4, borderPadding=6)
BULLET = st('BU', fontName='Helvetica', fontSize=10.5, leading=15, textColor=DGREY, leftIndent=16, spaceAfter=3)
NOTE   = st('NT', fontName='Helvetica-Oblique', fontSize=10, leading=14, textColor=MGREY, leftIndent=12, spaceAfter=6)
WARN   = st('WN', fontName='Helvetica-Bold', fontSize=10, leading=14, textColor=RED, leftIndent=12, spaceAfter=6)
HDR_W  = st('HW', fontName='Helvetica-Bold', fontSize=10, leading=14, textColor=WHITE)


def SP(n=8): return Spacer(1, n)
def HR(): return HRFlowable(width='100%', thickness=1, color=BLUE, spaceAfter=8, spaceBefore=4)
def B(t): return Paragraph(f'• {t}', BULLET)
def C(t): return Paragraph(t, CODE)
def P(t): return Paragraph(t, BODY)


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


def cmd_block(lines):
    """Render a command block with monospace font."""
    text = '<br/>'.join(lines)
    return Paragraph(text, CODE)


def build_pdf():
    doc = SimpleDocTemplate(OUT_PATH, pagesize=A4,
        leftMargin=M, rightMargin=M, topMargin=M, bottomMargin=M)

    s = []

    # ── Title ─────────────────────────────────────────────────────────────────
    s.append(Paragraph('Git Workflow Guide', TITLE))
    s.append(Paragraph('Betsybots · DIY Challenge Repo · Developer Reference', st('SUB',
        fontName='Helvetica', fontSize=11, leading=15, textColor=MGREY, spaceAfter=8)))
    s.append(HR())
    s.append(SP(4))

    # ── Overview ──────────────────────────────────────────────────────────────
    s.append(Paragraph('Overview', H1))
    s.append(P('This document defines how we manage code in the <b>DIY-Challenge-Repo</b> monorepo. '
               'All developers follow the same workflow to keep <b>main</b> stable and always launchable.'))
    s.append(SP(4))
    s.append(info_box('Key Principles', [
        'main is always launchable — never push broken code directly',
        'Every change goes through a Pull Request (PR) with at least 1 approval',
        'Small, focused PRs — one task per PR, easy to review',
        'Task-based branches — own the task end-to-end',
        'No folder-level ownership — touch whatever files the task needs',
    ], NAVY))

    # ── Repository Setup ──────────────────────────────────────────────────────
    s.append(Paragraph('Repository Setup', H1))
    s.append(Paragraph('Initial Clone', H2))
    s.append(P('Clone with submodules:'))
    s.append(cmd_block([
        '$ git clone --recurse-submodules \\',
        '    https://github.com/Betsybots/DIY-Challenge-Repo.git',
        '$ cd DIY-Challenge-Repo',
    ]))
    s.append(P('If you already cloned without submodules:'))
    s.append(cmd_block([
        '$ git submodule update --init --recursive',
    ]))
    s.append(SP(4))
    s.append(Paragraph('Verify Your Setup', H2))
    s.append(cmd_block([
        '$ git status              # should be on main, clean tree',
        '$ git submodule status    # all submodules show a commit SHA',
        '$ git remote -v           # origin → github.com/Betsybots/...',
    ]))

    # ── Branching Strategy ────────────────────────────────────────────────────
    s.append(Paragraph('Branching Strategy', H1))
    s.append(P('We use <b>short-lived feature branches</b> off main. No long-running develop branch.'))
    s.append(SP(4))

    s.append(Paragraph('Branch Naming Convention', H2))
    naming = [
        ['<b>Type</b>', '<b>Pattern</b>', '<b>Example</b>'],
        ['New feature', 'feature/description', 'feature/lidar-cluster-classifier'],
        ['Calibration work', 'calib/description', 'calib/imu-noise-params'],
        ['Bug fix', 'fix/description', 'fix/ekf-divergence-on-startup'],
        ['Config tuning', 'tune/description', 'tune/nav2-inflation-radius'],
        ['Documentation', 'docs/description', 'docs/race-day-runbook'],
    ]
    tbl = Table(naming, colWidths=[3.2*cm, 4.5*cm, CW - 7.7*cm - 0.6*cm])
    tbl.setStyle(TableStyle([
        ('BACKGROUND',(0,0),(-1,0),BLUE),
        ('TEXTCOLOR',(0,0),(-1,0),WHITE),
        ('FONTNAME',(0,0),(-1,0),'Helvetica-Bold'),
        ('FONTSIZE',(0,0),(-1,-1),10),
        ('ROWBACKGROUNDS',(0,1),(-1,-1),[WHITE, LGREY]),
        ('TOPPADDING',(0,0),(-1,-1),6),('BOTTOMPADDING',(0,0),(-1,-1),6),
        ('LEFTPADDING',(0,0),(-1,-1),10),('RIGHTPADDING',(0,0),(-1,-1),10),
        ('GRID',(0,0),(-1,-1),0.4,colors.HexColor('#E2E8F0')),
    ]))
    s.append(tbl)

    s.append(SP(8))
    s.append(Paragraph('Creating a Branch', H2))
    s.append(cmd_block([
        '# Always start from latest main',
        '$ git checkout main',
        '$ git pull origin main',
        '',
        '# Create your branch',
        '$ git checkout -b feature/my-task-name',
    ]))

    # ── Daily Workflow ────────────────────────────────────────────────────────
    s.append(Paragraph('Daily Workflow', H1))

    s.append(Paragraph('1. Start of Session', H2))
    s.append(cmd_block([
        '# Make sure you have latest main merged into your branch',
        '$ git checkout feature/my-task',
        '$ git fetch origin',
        '$ git rebase origin/main',
        '',
        '# If rebase has conflicts:',
        '$ git status                    # see conflicted files',
        '# ... fix conflicts in editor ...',
        '$ git add &lt;fixed-files&gt;',
        '$ git rebase --continue',
    ]))

    s.append(Paragraph('2. Making Changes', H2))
    s.append(P('Commit often with clear messages. Each commit should be a logical unit of work.'))
    s.append(cmd_block([
        '$ git add -p                    # stage hunks interactively (preferred)',
        '$ git add src/my_package/',
        '',
        '$ git commit -m "feat(localization): add RTK fusion to EKF config"',
    ]))
    s.append(SP(4))
    s.append(Paragraph('Commit Message Format', H2))
    s.append(P('Use conventional commit prefixes:'))
    prefixes = [
        ['<b>Prefix</b>', '<b>Use For</b>'],
        ['feat:', 'New functionality or capability'],
        ['fix:', 'Bug fix or correction'],
        ['calib:', 'Calibration data or config update'],
        ['tune:', 'Parameter tuning (Nav2, EKF, etc.)'],
        ['docs:', 'Documentation changes'],
        ['refactor:', 'Code restructuring (no behavior change)'],
        ['chore:', 'Build, CI, or repo maintenance'],
    ]
    tbl2 = Table(prefixes, colWidths=[3.0*cm, CW - 3.0*cm - 0.3*cm])
    tbl2.setStyle(TableStyle([
        ('BACKGROUND',(0,0),(-1,0),TEAL),
        ('TEXTCOLOR',(0,0),(-1,0),WHITE),
        ('FONTNAME',(0,0),(-1,0),'Helvetica-Bold'),
        ('FONTSIZE',(0,0),(-1,-1),10),
        ('ROWBACKGROUNDS',(0,1),(-1,-1),[WHITE, LGREY]),
        ('TOPPADDING',(0,0),(-1,-1),5),('BOTTOMPADDING',(0,0),(-1,-1),5),
        ('LEFTPADDING',(0,0),(-1,-1),10),('RIGHTPADDING',(0,0),(-1,-1),10),
        ('GRID',(0,0),(-1,-1),0.4,colors.HexColor('#E2E8F0')),
    ]))
    s.append(tbl2)

    s.append(SP(4))
    s.append(Paragraph('Good commit message examples:', BODY))
    s.append(cmd_block([
        'feat(perception): add PCL cluster size filter for obstacle detection',
        'calib(imu): update noise density from calibration session 2026-06-03',
        'fix(cmd_vel_mux): prevent zero-velocity output when e-stop releases',
        'tune(nav2): reduce inflation radius from 0.55 to 0.40 for narrow pass',
    ]))

    s.append(Paragraph('3. Pushing Your Branch', H2))
    s.append(cmd_block([
        '$ git push origin feature/my-task',
        '',
        '# First push — set upstream:',
        '$ git push -u origin feature/my-task',
    ]))

    # ── Pull Requests ─────────────────────────────────────────────────────────
    s.append(PageBreak())
    s.append(Paragraph('Pull Requests (PRs)', H1))
    s.append(P('Every change reaches main via a PR on GitHub. No direct pushes to main are allowed.'))

    s.append(Paragraph('Opening a PR', H2))
    s.append(B('Go to GitHub → Pull Requests → New Pull Request'))
    s.append(B('Base: <b>main</b> ← Compare: <b>your-branch</b>'))
    s.append(B('Fill in the PR template (see below)'))
    s.append(B('Assign at least 1 reviewer'))
    s.append(B('Link any related issues if applicable'))

    s.append(SP(8))
    s.append(Paragraph('PR Description Template', H2))
    s.append(P('Copy this into your PR description:'))
    s.append(cmd_block([
        '## What Changed',
        '- Brief list of changes',
        '',
        '## Why',
        '- Motivation / context',
        '',
        '## How I Tested',
        '- What you ran to validate (launch command, bag replay, etc.)',
        '- Include relevant terminal output or screenshots if helpful',
        '',
        '## Known Risks',
        '- Any areas of concern or things to watch after merge',
        '',
        '## Checklist',
        '- [ ] Builds clean: colcon build --packages-select &lt;pkg&gt;',
        '- [ ] Tested on robot / in sim / bag replay (pick one)',
        '- [ ] No TF errors or topic mismatches in logs',
        '- [ ] Config changes documented in commit message',
    ]))

    s.append(Paragraph('Review Process', H2))
    s.append(B('Reviewer has <b>24 hours</b> to review (48h max if weekend)'))
    s.append(B('Reviewer checks: does it build? does the description make sense? any obvious issues?'))
    s.append(B('Reviewer can <b>Approve</b>, <b>Request Changes</b>, or <b>Comment</b>'))
    s.append(B('If changes requested → push fixes to the same branch → re-request review'))
    s.append(B('Once approved → the PR author merges'))

    s.append(SP(8))
    s.append(Paragraph('Merging', H2))
    s.append(P('Use <b>Squash and Merge</b> for most PRs (keeps main history clean). '
               'Use <b>Merge Commit</b> only for large multi-commit PRs where individual commits matter.'))
    s.append(cmd_block([
        '# After merge on GitHub, clean up locally:',
        '$ git checkout main',
        '$ git pull origin main',
        '$ git branch -d feature/my-task         # delete local branch',
    ]))
    s.append(SP(4))
    s.append(Paragraph('<b>⚠ Never force-push to main or rewrite main history.</b>', WARN))

    # ── Branch Protection Rules ───────────────────────────────────────────────
    s.append(Paragraph('Branch Protection Rules', H1))
    s.append(P('These rules are configured on GitHub for the <b>main</b> branch:'))
    s.append(SP(4))

    rules = [
        ['<b>Rule</b>', '<b>Setting</b>'],
        ['Require PR before merge', 'Yes — no direct pushes'],
        ['Required approvals', '1 minimum'],
        ['Dismiss stale reviews on new push', 'Yes'],
        ['Require branches up-to-date', 'Yes — must rebase before merge'],
        ['Force push', 'Blocked for everyone'],
        ['Branch deletion', 'Blocked (main only)'],
    ]
    tbl3 = Table(rules, colWidths=[5.5*cm, CW - 5.5*cm - 0.3*cm])
    tbl3.setStyle(TableStyle([
        ('BACKGROUND',(0,0),(-1,0),RED),
        ('TEXTCOLOR',(0,0),(-1,0),WHITE),
        ('FONTNAME',(0,0),(-1,0),'Helvetica-Bold'),
        ('FONTSIZE',(0,0),(-1,-1),10),
        ('ROWBACKGROUNDS',(0,1),(-1,-1),[WHITE, LGREY]),
        ('TOPPADDING',(0,0),(-1,-1),6),('BOTTOMPADDING',(0,0),(-1,-1),6),
        ('LEFTPADDING',(0,0),(-1,-1),10),('RIGHTPADDING',(0,0),(-1,-1),10),
        ('GRID',(0,0),(-1,-1),0.4,colors.HexColor('#E2E8F0')),
    ]))
    s.append(tbl3)

    # ── Handling Conflicts ────────────────────────────────────────────────────
    s.append(Paragraph('Handling Merge Conflicts', H1))
    s.append(P('Conflicts happen when two branches edit the same lines. Here\'s how to resolve:'))
    s.append(SP(4))

    s.append(Paragraph('Option A: Rebase (preferred)', H2))
    s.append(cmd_block([
        '$ git fetch origin',
        '$ git rebase origin/main',
        '',
        '# For each conflict:',
        '$ git status                     # see which files conflict',
        '# Edit the files — remove &lt;&lt;&lt;/===/&gt;&gt;&gt; markers',
        '$ git add &lt;resolved-file&gt;',
        '$ git rebase --continue',
        '',
        '# If it gets messy, abort and start over:',
        '$ git rebase --abort',
    ]))

    s.append(Paragraph('Option B: Merge (if rebase is complex)', H2))
    s.append(cmd_block([
        '$ git fetch origin',
        '$ git merge origin/main',
        '# Resolve conflicts same way, then:',
        '$ git commit                     # creates merge commit',
    ]))

    s.append(SP(6))
    s.append(info_box('When to Ask for Help', [
        'If a conflict touches files you didn\'t write — ping that developer',
        'If rebase produces more than 3 conflicted files — discuss in sync meeting',
        'Never force-push over someone else\'s commits without telling them',
    ], ORANGE))

    # ── Submodules & Patches ─────────────────────────────────────────────────
    s.append(PageBreak())
    s.append(Paragraph('Submodule Management & Patches', H1))
    s.append(P('The <b>third_party_ws/src/</b> folder contains 6 git submodules — third-party ROS 2 packages '
               'that we use as-is or with small modifications applied via patch files.'))
    s.append(SP(4))

    # Submodule table
    submods = [
        ['<b>Submodule</b>', '<b>Purpose</b>', '<b>Patched?</b>'],
        ['FAST_LIO', 'Real-time LiDAR-inertial odometry', 'No'],
        ['LIO-SAM', 'Offline prior map generation', 'No'],
        ['imu_utils_ros2_humble', 'IMU noise calibration tool', 'No'],
        ['lidar_imu_calib', 'LiDAR-IMU extrinsic calibration', 'Yes'],
        ['livox_ros_driver2', 'Hesai QT64 LiDAR driver', 'Yes'],
        ['ndt_omp_ros2', 'NDT scan matching (localization)', 'Yes'],
    ]
    tbl_sub = Table(submods, colWidths=[4.5*cm, 6.5*cm, CW - 11.0*cm - 0.6*cm])
    tbl_sub.setStyle(TableStyle([
        ('BACKGROUND',(0,0),(-1,0),PURPLE),
        ('TEXTCOLOR',(0,0),(-1,0),WHITE),
        ('FONTNAME',(0,0),(-1,0),'Helvetica-Bold'),
        ('FONTSIZE',(0,0),(-1,-1),10),
        ('ROWBACKGROUNDS',(0,1),(-1,-1),[WHITE, LGREY]),
        ('TOPPADDING',(0,0),(-1,-1),5),('BOTTOMPADDING',(0,0),(-1,-1),5),
        ('LEFTPADDING',(0,0),(-1,-1),10),('RIGHTPADDING',(0,0),(-1,-1),10),
        ('GRID',(0,0),(-1,-1),0.4,colors.HexColor('#E2E8F0')),
    ]))
    s.append(tbl_sub)

    s.append(SP(8))
    s.append(Paragraph('How Submodules Work in This Repo', H2))
    s.append(B('Each submodule points to a <b>specific upstream commit SHA</b> — a known-good version'))
    s.append(B('The SHA is tracked in the parent repo (like a bookmark to that exact version)'))
    s.append(B('When you clone with <b>--recurse-submodules</b>, git checks out that exact commit'))
    s.append(B('We <b>never modify code directly inside submodule directories</b>'))
    s.append(B('If we need changes, we use <b>patch files</b> applied at build time'))

    s.append(SP(8))
    s.append(info_box('Why Not Just Edit the Submodule Directly?', [
        'Commits inside a submodule only exist on YOUR machine until pushed to the upstream remote',
        'If you commit inside a submodule, the parent repo records a SHA that nobody else has',
        'Result: teammates clone the repo and git fails to find the submodule commit → broken checkout',
        'Patches avoid this entirely — the modification lives in OUR repo as a .patch file',
    ], RED))

    # ── Patch System ──────────────────────────────────────────────────────────
    s.append(SP(8))
    s.append(Paragraph('The Patch System', H1))
    s.append(P('Patches are small diff files stored in the <b>patches/</b> directory. '
               'They record changes we need on top of the upstream submodule code.'))
    s.append(SP(4))

    s.append(Paragraph('Current Patches', H2))
    patches = [
        ['<b>Patch File</b>', '<b>Applied To</b>', '<b>What It Does</b>'],
        ['lidar_imu_calib.patch', 'lidar_imu_calib', 'Build fixes for ROS 2 Humble compatibility'],
        ['livox_ros_driver2.patch', 'livox_ros_driver2', 'Config adjustments for Hesai QT64'],
        ['ndt_omp_ros2.patch', 'ndt_omp_ros2', 'Build fixes and parameter changes for our setup'],
    ]
    tbl_p = Table(patches, colWidths=[4.5*cm, 3.8*cm, CW - 8.3*cm - 0.6*cm])
    tbl_p.setStyle(TableStyle([
        ('BACKGROUND',(0,0),(-1,0),TEAL),
        ('TEXTCOLOR',(0,0),(-1,0),WHITE),
        ('FONTNAME',(0,0),(-1,0),'Helvetica-Bold'),
        ('FONTSIZE',(0,0),(-1,-1),10),
        ('ROWBACKGROUNDS',(0,1),(-1,-1),[WHITE, LGREY]),
        ('TOPPADDING',(0,0),(-1,-1),5),('BOTTOMPADDING',(0,0),(-1,-1),5),
        ('LEFTPADDING',(0,0),(-1,-1),10),('RIGHTPADDING',(0,0),(-1,-1),10),
        ('GRID',(0,0),(-1,-1),0.4,colors.HexColor('#E2E8F0')),
    ]))
    s.append(tbl_p)

    s.append(SP(8))
    s.append(Paragraph('Applying Patches (After Clone)', H2))
    s.append(P('Patches are applied by <b>setup.sh</b> or manually:'))
    s.append(cmd_block([
        '# Apply a single patch:',
        '$ cd third_party_ws/src/livox_ros_driver2',
        '$ git apply ../../../patches/livox_ros_driver2.patch',
        '',
        '# Or apply all patches at once (setup.sh does this):',
        '$ cd /path/to/DIY-Challenge-Repo',
        '$ bash setup.sh',
    ]))
    s.append(SP(4))
    s.append(Paragraph('<i>Note: After patches are applied, the submodule directory is "dirty" (has uncommitted changes). '
                       'This is expected and .gitmodules has ignore = dirty to suppress the noise.</i>', NOTE))

    s.append(SP(8))
    s.append(Paragraph('Creating a New Patch', H2))
    s.append(P('When you need to modify a submodule (e.g., fix a build issue, change a default param):'))
    s.append(cmd_block([
        '# 1. Make your edits inside the submodule',
        '$ cd third_party_ws/src/FAST_LIO',
        '$ nano src/laserMapping.cpp     # or whatever file needs changing',
        '',
        '# 2. Generate the patch file',
        '$ git diff &gt; ../../../patches/fast_lio.patch',
        '',
        '# 3. Go back to the parent repo and commit the PATCH (not the submodule)',
        '$ cd ../../..',
        '$ git add patches/fast_lio.patch',
        '$ git commit -m "fix(submodule): patch FAST_LIO for &lt;reason&gt;"',
    ]))
    s.append(SP(4))
    s.append(Paragraph('<b>⚠ Important:</b> Do NOT run <b>git add third_party_ws/src/FAST_LIO</b> — '
                       'that would change the submodule pointer to a local-only commit.', WARN))

    s.append(SP(8))
    s.append(Paragraph('Updating a Patch', H2))
    s.append(P('If a patch needs changes (e.g., you need an additional fix in the same submodule):'))
    s.append(cmd_block([
        '# 1. Reset the submodule to upstream state',
        '$ cd third_party_ws/src/livox_ros_driver2',
        '$ git checkout .',
        '',
        '# 2. Apply the existing patch',
        '$ git apply ../../../patches/livox_ros_driver2.patch',
        '',
        '# 3. Make your additional edits',
        '$ nano src/some_file.cpp',
        '',
        '# 4. Regenerate the patch (captures ALL changes vs upstream)',
        '$ git diff &gt; ../../../patches/livox_ros_driver2.patch',
        '',
        '# 5. Commit the updated patch',
        '$ cd ../../..',
        '$ git add patches/livox_ros_driver2.patch',
        '$ git commit -m "fix(submodule): update livox_ros_driver2 patch for &lt;reason&gt;"',
    ]))

    s.append(SP(8))
    s.append(Paragraph('Updating a Submodule to a Newer Upstream Version', H2))
    s.append(P('If upstream publishes a bug fix or new feature we need:'))
    s.append(cmd_block([
        '# 1. Enter the submodule and fetch upstream',
        '$ cd third_party_ws/src/FAST_LIO',
        '$ git fetch origin',
        '$ git log --oneline origin/main -5      # see recent commits',
        '',
        '# 2. Move to the new commit',
        '$ git checkout &lt;new-commit-sha&gt;',
        '',
        '# 3. Check if existing patch still applies (if one exists)',
        '$ git apply --check ../../../patches/fast_lio.patch',
        '# If it fails: regenerate or fix the patch manually',
        '',
        '# 4. Go to parent repo and commit the pointer update',
        '$ cd ../../..',
        '$ git add third_party_ws/src/FAST_LIO',
        '$ git commit -m "chore(submodule): update FAST_LIO to &lt;sha&gt; (reason)"',
    ]))

    s.append(SP(8))
    s.append(Paragraph('.gitmodules and ignore = dirty', H2))
    s.append(P('Our <b>.gitmodules</b> file includes <b>ignore = dirty</b> for patched submodules:'))
    s.append(cmd_block([
        '[submodule "third_party_ws/src/livox_ros_driver2"]',
        '    path = third_party_ws/src/livox_ros_driver2',
        '    url = https://github.com/...',
        '    ignore = dirty',
    ]))
    s.append(B('This tells <b>git status</b> to ignore uncommitted changes inside the submodule'))
    s.append(B('Without this, every dev would see "modified: third_party_ws/src/..." after applying patches'))
    s.append(B('It does <b>NOT</b> ignore submodule pointer changes — those still show up correctly'))

    s.append(SP(8))
    s.append(info_box('Submodule Golden Rules', [
        'Never git add a submodule directory unless you intend to update its pointer',
        'Never git commit inside a submodule — your teammates can\'t access those commits',
        'Always use patch files for modifications — commit the .patch, not the submodule',
        'After cloning, run setup.sh to apply all patches automatically',
        'If a patch fails to apply after a submodule update, fix the patch and commit the fix',
    ], GREEN))

    # ── Common Scenarios ──────────────────────────────────────────────────────
    s.append(Paragraph('Common Scenarios', H1))

    s.append(Paragraph('Scenario: Wrong branch — committed to main', H2))
    s.append(cmd_block([
        '# Move the commit to a new branch (if not pushed yet)',
        '$ git branch feature/oops          # create branch at current HEAD',
        '$ git reset --hard HEAD~1          # move main back one commit',
        '$ git checkout feature/oops        # continue on new branch',
    ]))

    s.append(Paragraph('Scenario: Need to undo last commit (not pushed)', H2))
    s.append(cmd_block([
        '$ git reset --soft HEAD~1          # undo commit, keep changes staged',
        '# or',
        '$ git reset --mixed HEAD~1         # undo commit, unstage changes',
    ]))

    s.append(Paragraph('Scenario: Stash work to switch branches', H2))
    s.append(cmd_block([
        '$ git stash                        # save current changes',
        '$ git checkout other-branch',
        '# ... do something ...',
        '$ git checkout feature/my-task',
        '$ git stash pop                    # restore changes',
    ]))

    s.append(Paragraph('Scenario: PR is behind main', H2))
    s.append(P('GitHub will show "This branch is behind main" — you need to rebase:'))
    s.append(cmd_block([
        '$ git fetch origin',
        '$ git rebase origin/main',
        '$ git push --force-with-lease      # safe force push (only your branch)',
    ]))
    s.append(Paragraph('<i>Note: --force-with-lease is safe for your own feature branch. '
                       'Never use --force on main or shared branches.</i>', NOTE))

    # ── Quick Reference ───────────────────────────────────────────────────────
    s.append(PageBreak())
    s.append(Paragraph('Quick Reference Card', H1))
    s.append(SP(4))

    qr = [
        ['<b>Action</b>', '<b>Command</b>'],
        ['Start new task', 'git checkout main &amp;&amp; git pull &amp;&amp; git checkout -b feature/name'],
        ['Stage changes', 'git add -p  (interactive) or  git add &lt;files&gt;'],
        ['Commit', 'git commit -m "type(scope): description"'],
        ['Push branch', 'git push -u origin feature/name'],
        ['Update from main', 'git fetch origin &amp;&amp; git rebase origin/main'],
        ['Resolve conflicts', 'Edit files → git add → git rebase --continue'],
        ['Force push (own branch)', 'git push --force-with-lease'],
        ['After PR merged', 'git checkout main &amp;&amp; git pull &amp;&amp; git branch -d feature/name'],
        ['Stash work', 'git stash  /  git stash pop'],
        ['View log', 'git log --oneline -20'],
        ['See what changed', 'git diff --stat origin/main'],
    ]
    tbl4 = Table(qr, colWidths=[4.0*cm, CW - 4.0*cm - 0.3*cm])
    tbl4.setStyle(TableStyle([
        ('BACKGROUND',(0,0),(-1,0),NAVY),
        ('TEXTCOLOR',(0,0),(-1,0),WHITE),
        ('FONTNAME',(0,0),(-1,0),'Helvetica-Bold'),
        ('FONTNAME',(0,1),(1,-1),'Courier'),
        ('FONTSIZE',(0,0),(-1,-1),9.5),
        ('ROWBACKGROUNDS',(0,1),(-1,-1),[WHITE, LGREY]),
        ('TOPPADDING',(0,0),(-1,-1),6),('BOTTOMPADDING',(0,0),(-1,-1),6),
        ('LEFTPADDING',(0,0),(-1,-1),10),('RIGHTPADDING',(0,0),(-1,-1),10),
        ('GRID',(0,0),(-1,-1),0.4,colors.HexColor('#E2E8F0')),
    ]))
    s.append(tbl4)

    s.append(SP(12))
    s.append(info_box('Golden Rules', [
        'Never push directly to main',
        'Never force-push to main or someone else\'s branch',
        'Always rebase before merge — keep history linear',
        'One task per branch, one outcome per PR',
        'If in doubt, ask in the weekly sync',
    ], GREEN))

    s.append(SP(16))
    s.append(Paragraph('— Betsybots Team · DIY Challenge 2026', st('FT', fontName='Helvetica-Oblique',
        fontSize=9, textColor=MGREY, alignment=TA_CENTER)))

    doc.build(s)
    print(f'[OK] PDF written to: {OUT_PATH}')


if __name__ == '__main__':
    build_pdf()
