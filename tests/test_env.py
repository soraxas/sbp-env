from unittest import TestCase

import numpy as np

from sbp_env import env, visualiser
from sbp_env.planners.basePlanner import Planner
from sbp_env.samplers.baseSampler import Sampler
from sbp_env import generate_args
from sbp_env.utils import planner_registry
from tests.common_vars import DummyPlannerClass


class TestGenerateArgs(TestCase):
    def test_missing_argunment(self):
        with self.assertRaises(TypeError):
            generate_args()
        with self.assertRaises(TypeError):
            generate_args(planner_id="rrt")
        with self.assertRaises(TypeError):
            generate_args(map_fname="maps/4d.png")

        # should not raise error
        generate_args(planner_id="rrt", map_fname="maps/4d.png")

        # test error if the planner id has not been registered yet
        with self.assertRaises(ValueError):
            generate_args(planner_id="my_planner_id", map_fname="maps/4d.png")

        # test that after the planner id is registered, it will work.
        planner_registry.register_planner(
            planner_id="my_planner_id",
            planner_class=DummyPlannerClass,
            sampler_id="random",
        )
        generate_args(planner_id="my_planner_id", map_fname="maps/4d.png")

    def test_actual_planning(self):
        visualiser.VisualiserSwitcher.choose_visualiser("base")
        args = generate_args(
            planner_id="rrt",
            map_fname="maps/test.png",
            start_pt=np.array([25, 123]),
            goal_pt=np.array([225, 42]),
        )
        args.no_display = True

        e = env.Env(args, fixed_seed=0)
        ori_method = e.planner.run_once

        # prepare an exception to escape from the planning loop
        class PlanningSuccess(Exception):
            pass

        def planner_run_once_with_side_effect(*args, **kwargs):
            # pass through to planner
            ori_method(*args, **kwargs)
            if e.planner.c_max < float("inf"):
                raise PlanningSuccess()

        # patch the planner run_once such that it will terminates as soon as the
        # planning problem is finished.
        e.planner.run_once = planner_run_once_with_side_effect
        with self.assertRaises(PlanningSuccess):
            e.run()

    def test_get_attribute(self):
        visualiser.VisualiserSwitcher.choose_visualiser("pygame")
        args = generate_args(
            planner_id="rrt",
            map_fname="maps/test.png",
            start_pt=np.array([25, 123]),
            goal_pt=np.array([225, 42]),
        )
        args.no_display = True

        e = env.Env(args, fixed_seed=0)

        # test get planner
        assert isinstance(e.planner, Planner)

        # test get sampler
        assert isinstance(e.sampler, Sampler)
