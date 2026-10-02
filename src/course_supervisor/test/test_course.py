import math
import os

import numpy as np
import pytest

from course_supervisor.course import Course, Progress, pure_pursuit

COURSE = os.path.join(os.path.dirname(__file__), '..', 'config', 'obstacle_course.yaml')


@pytest.fixture(scope='module')
def course():
    return Course.load(COURSE)


def test_sections_ordered_and_cover_course(course):
    starts = [s.s_start for s in course.sections]
    assert starts[0] == 0.0
    assert starts == sorted(starts)
    assert starts[-1] < course.length


def test_progress_follows_line_in_order(course):
    prog = Progress(course)
    rng = np.random.default_rng(0)
    seen = []
    for s in np.arange(0.0, course.length, 0.1):
        x, y, yaw = course.point_at(s)
        n = rng.normal(0, 0.1, 2)
        prog.update(x + n[0], y + n[1])
        idx = course.section_index(prog.s)
        if not seen or seen[-1] != idx:
            seen.append(idx)
    assert seen == list(range(len(course.sections)))
    assert prog.s > course.length - 0.5


def test_progress_not_captured_by_other_lanes(course):
    # Sitting at progress s, a pose on a different part of the course must not
    # move progress anywhere.
    prog = Progress(course)
    prog.s = 50.0
    for s_other in (5.0, 16.0, 20.0, 70.0):
        x, y, _ = course.point_at(s_other)
        prog.update(x, y)
        assert 49.5 <= prog.s <= 52.5


def test_path_tracker_closed_loop(course):
    # Kinematic unicycle with the tracker in the loop over the whole course,
    # starting 0.2 m off the line.
    x, y, yaw = course.point_at(0.0)
    y += 0.2
    prog = Progress(course)
    dt, worst = 0.05, 0.0
    for step in range(int(course.length / 0.3 / dt * 1.5)):
        s = prog.update(x, y)
        if s >= course.length - 0.3:
            break
        v, w = pure_pursuit(course, x, y, yaw, s, 0.3, 0.6, 0.371)
        assert abs(w) <= v / 0.371 + 1e-9
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw += w * dt
        if step * dt > 5.0:
            worst = max(worst, prog.offset)
    assert prog.s >= course.length - 0.3, 'tracker did not reach the end'
    assert worst < 0.15, f'cross-track error {worst:.3f} m'


def test_lap_wrap_restarts_progress(course):
    # Drive to the end of the line, reset (as the supervisor does for the next lap) and
    # check progress picks up again from the start lane without jumping.
    prog = Progress(course)
    for s in np.arange(0.0, course.length, 0.1):
        x, y, _ = course.point_at(s)
        prog.update(x, y)
    assert prog.s > course.length - 0.3
    x, y, _ = course.point_at(course.length)
    prog.reset()
    assert prog.update(x, y) < 0.6                     # finish is next to the start
    for s in np.arange(0.0, 10.0, 0.1):
        x, y, _ = course.point_at(s)
        prog.update(x, y)
    assert abs(prog.s - 9.9) < 0.3
