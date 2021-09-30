from unittest import TestCase

from planners.basePlanner import Planner
from samplers.baseSampler import Sampler
from utils.planner_registry import register_planner, register_sampler


class DummyClass:
    pass


class MyPlanner(Planner):
    pass


class MySampler(Sampler):
    pass


class TestRegisterPlanner(TestCase):
    def test_register(self):
        register_planner(
            "my_id",
            planner_class=MyPlanner,
            sampler_id="my_sampler_id",
        )

    def test_register_not_derived(self):
        with self.assertRaises(TypeError):
            register_planner(
                "my_non_derived_id",
                planner_class=DummyClass,
                sampler_id="my_sampler_id",
            )

    def test_register_duplicate_id(self):
        register_planner(
            "my_duplicate_id",
            planner_class=MyPlanner,
            sampler_id="my_sampler_id",
        )
        # cannot have duplicate id
        with self.assertRaises(ValueError):
            register_planner(
                "my_duplicate_id",
                planner_class=MyPlanner,
                sampler_id="my_sampler_id",
            )


class TestRegisterSampler(TestCase):
    def test_register(self):
        register_sampler(
            "my_sampler_id",
            sampler_class=MySampler,
        )

    def test_register_not_derived(self):
        with self.assertRaises(TypeError):
            register_sampler(
                "my_non_derived_sampler_id",
                sampler_class=DummyClass,
            )

    def test_register_duplicate_id(self):
        register_sampler(
            "my_duplicate_sampler_id",
            sampler_class=MySampler,
        )
        # cannot have duplicate id
        with self.assertRaises(ValueError):
            register_sampler(
                "my_duplicate_sampler_id",
                sampler_class=MySampler,
            )
