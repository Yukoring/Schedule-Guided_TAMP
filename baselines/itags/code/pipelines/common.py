"""World construction (environment + PRM) and plan loading (plan file -> trajectories)."""
import shutil
import time
from dataclasses import dataclass
from pathlib import Path

from world.environment import Environment
from world.prm import PRMPlanning
from common.visualize import (Animation, plot_best_plan, plot_environment,
                              plot_prm)

PROBLEM_BASENAME = 'problem.pddl'


@dataclass
class World:
    """Static per-trial context: environment, roadmap and goal samples."""
    env: object
    prm: object
    init_samples: object
    init_roadmap: object
    goal_samples: list
    construct_time: float


def build_world(map_file, debug, plot, trial_idx, prm_cls=PRMPlanning):
    start = time.time()
    env = Environment(map_file, debug)
    if plot:
        plot_environment(env, trial_idx)

    prm = prm_cls(env)
    init_samples, init_roadmap, goal_samples = prm.ConstructPhase(goals_map=env.goals_map)
    construct_time = time.time() - start

    if plot:
        plot_prm(env, init_samples, init_roadmap, goal_samples)

    return World(env, prm, init_samples, init_roadmap,
                 goal_samples, construct_time)


@dataclass
class LoadedPlan:
    """A task plan turned into robot trajectories by the motion planner."""
    path: dict
    total_cost: float
    motion_cost: float  # makespan


def load_plan(generator, plan_file, label, count, debug):
    """Parse a temporal plan and map it to trajectories. None if no plan file."""
    if not Path(plan_file).is_file():
        return None
    try:
        dispatch, task_cost, cal_time = generator.parse_pddl_plan(plan_file)
    except Exception as exc:
        # keep the plan and problem files of a malformed plan
        dump = Path('crash_dumps')
        dump.mkdir(exist_ok=True)
        stamp = '%s_%s_c%d' % (Path(plan_file).stem, label.replace(' ', ''), count)
        shutil.copy(plan_file, dump / Path(plan_file).name)
        for prob in Path('pddl_prob_plan').glob('problem*_round%d_*.pddl' % count):
            shutil.copy(prob, dump / ('%s__%s' % (stamp, prob.name)))
        print('PARSE FAILURE (%s, count %d) -> variant dropped, evidence in %s/: %s'
              % (label, count, dump, exc))
        return None
    path, total_cost, motion_cost = generator.generate_path(dispatch)
    if debug:
        print('%s (count %d)' % (label, count))
        print('Task Plan Cost: ', task_cost, ' Calculation Time: ', cal_time,
              ' Motion Planner Path Total Cost: ', total_cost,
              ' Motion Planner Path Max Cost: ', motion_cost)
    return LoadedPlan(path, total_cost, motion_cost)
