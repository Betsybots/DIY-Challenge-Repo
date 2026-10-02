import os

import pytest
import yaml

from course_supervisor.course import Course
from course_supervisor.plan_tags import GridMap, plan_tag

HERE = os.path.dirname(__file__)
CFG = os.path.join(HERE, '..', 'config')
MAP = os.path.join(HERE, '..', '..', 'diy_localization', 'map', 'Obstacle_course', 'obstacle_course_clean.yaml')


@pytest.mark.skipif(not os.path.exists(MAP), reason='obstacle course map not in this checkout')
def test_planned_tags_are_visible_and_match_tag_map():
    course = Course.load(os.path.join(CFG, 'obstacle_course.yaml'))
    grid = GridMap(MAP)
    with open(os.path.join(CFG, 'tag_plan_obstacle_course.yaml')) as f:
        plan = yaml.safe_load(f)
    with open(os.path.join(CFG, 'tag_map_obstacle_course.yaml')) as f:
        tag_map = yaml.safe_load(f)['tags']
    sections = {s.name: s for s in course.sections}
    for spec in plan['tags']:
        r = plan_tag(course, grid, spec, float(plan.get('max_range', 1.0)), 80.0, float(plan.get('inset', 0.03)))
        assert r['seen'] and r['seen'][-1] - r['seen'][0] >= 1.0, f"tag {r['id']} barely visible"
        assert spec.get('trigger_range', 0.0) <= 1.0
        entry = tag_map[r['id']]
        assert abs(entry['x'] - r['x']) < 0.02 and abs(entry['y'] - r['y']) < 0.02
        if spec.get('section'):
            sec = sections[spec['section']]
            assert sec.trigger_label == spec['label']
            rng = float(spec.get('trigger_range', spec['s'] - sec.s_start))
            assert abs(sec.trigger_range - rng) < 0.1
            fire_s = spec['s'] - rng
            assert r['seen'][0] - 0.05 <= fire_s <= r['seen'][-1] + 0.05, 'trigger could never fire'
