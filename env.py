#!/usr/bin/env python
import logging
import math
import random

import numpy as np
from scipy import spatial
from tqdm import tqdm

import collisionChecker
from utils.common import Node, MagicDict, Stats
from visualiser import VisualiserSwitcher

LOGGER = logging.getLogger(__name__)


class Env(VisualiserSwitcher.env_clname):
    """Represents the planning environment. The main loop happens inside this class"""

    def __init__(self, writer=None, fname=None, fixed_seed=None, **kwargs):
        self.writer = writer
        self.fname = fname
        self.started = False

        if fixed_seed is not None:
            np.random.seed(fixed_seed)
            random.seed(fixed_seed)
            print(f"Fixed random seed: {fixed_seed}")

        # initialize and prepare screen
        self.args = MagicDict(kwargs)
        self.stats = Stats(showSampledPoint=self.args.showSampledPoint)

        if kwargs["engine"] == "image":
            self.cc = collisionChecker.ImgCollisionChecker(self.args.image)
            self.dist = self.euclidean_dist
        elif kwargs["engine"] == "4d":
            self.cc = collisionChecker.RobotArm4dCollisionChecker(self.args.image)
            self.dist = self.euclidean_dist
        elif kwargs["engine"] == "klampt":
            self.cc = collisionChecker.KlamptCollisionChecker(
                self.args.image, self.stats
            )
            self.dist = self.radian_dist

        kwargs["num_dim"] = self.cc.get_dimension()
        kwargs["image_shape"] = self.cc.get_image_shape()
        self.dim = kwargs["image_shape"]
        kwargs["cc"] = self.cc

        def parse_input_pt(pt_as_str):
            if pt_as_str is None:
                return None
            pt = pt_as_str.split(",")
            if len(pt) != kwargs["num_dim"]:
                raise RuntimeError(
                    f"Expected to have number of dimension = {kwargs['num_dim']}, but "
                    f"was n={len(pt)} from input '{pt_as_str}'"
                )
            return tuple(map(float, pt))

        kwargs["start_pt"] = parse_input_pt(kwargs["start_pt"])
        kwargs["goal_pt"] = parse_input_pt(kwargs["goal_pt"])

        self.args.planner = kwargs["planner_type"](**kwargs)
        kwargs["planner"] = self.args.planner

        self.planner = self.args.planner
        self.planner.args.env = self

        super().__init__(**kwargs)

        self.visualiser_init(no_display=kwargs["no_display"])
        start_pt, goal_pt = self.set_start_goal_points(
            start=kwargs["start_pt"], goal=kwargs["goal_pt"]
        )

        self.start_pt = self.goal_pt = None
        if start_pt:
            self.start_pt = Node(start_pt)
        if goal_pt:
            self.goal_pt = Node(goal_pt)
        self.start_pt.is_start = True
        self.goal_pt.is_goal = True
        self.planner.add_newnode(self.start_pt)
        self.update_screen(update_all=True)
        # update the string pt to object
        kwargs["start_pt"] = self.start_pt
        kwargs["goal_pt"] = self.goal_pt

        self.planner.init(env=self, **kwargs)
        if kwargs["engine"] == "klampt":
            self.args.sampler.set_use_radian(True)

    @staticmethod
    def radian_dist(p1: np.ndarray, p2: np.ndarray):
        """Return the cosine similarity for radian

        :param p1: first configuration :math:`q_1`
        :param p2: second configuration :math:`q_2`

        """
        # distance metric for angles
        # https://en.wikipedia.org/wiki/Cosine_similarity#Angular_distance_and_similarity

        cosine_similarity = 1 - spatial.distance.cosine(p1, p2)
        return np.arccos(cosine_similarity) / np.pi
        # return spatial.distance.cosine(p1, p2)

    @staticmethod
    def euclidean_dist(p1: np.ndarray, p2: np.ndarray):
        """Return the Euclidean distance between p1 and p2

        :param p1: first configuration :math:`q_1`
        :param p2: second configuration :math:`q_2`

        """
        # THIS IS MUCH SLOWER for small array
        # return np.linalg.norm(p1 - p2)

        p = p1 - p2
        return math.sqrt(p[0] ** 2 + p[1] ** 2)

    def step_from_to(self, p1: np.ndarray, p2: np.ndarray):
        """Get a new point from p1 to p2, according to step size.

        :param p1: first configuration :math:`q_1`
        :param p2: second configuration :math:`q_2`

        """
        if self.args.ignore_step_size:
            return p2
        if np.all(p1 == p2):
            # p1 is at the same point as p2
            return p2
        unit_vector = p2 - p1
        unit_vector = unit_vector / np.linalg.norm(unit_vector)
        step_size = self.dist(p1, p2)
        step_size = min(step_size, self.args.epsilon)
        return p1 + step_size * unit_vector

    def run(self):
        """Run until we reached the specified max nodes"""
        self.started = True

        with tqdm(
                total=self.args.max_number_nodes, desc=self.args.sampler.name
        ) as pbar:
            while self.stats.valid_sample < self.args.max_number_nodes:
                self.update_screen()
                self.planner.run_once()

                pbar.n = self.stats.valid_sample

                pbar.set_postfix(
                    {
                        # 'i.con' : self.stats.invalid_samples_connections,
                        # 'i.obs' : self.stats.invalid_samples_obstacles,
                        # 'su' : self.stats.sampler_success,
                        # 'sua' : self.stats.sampler_success_all,
                        # 'sf' : self.stats.sampler_fail,
                        "cc_fe": self.stats.feasible_cnt,
                        "cc_vi": self.stats.visible_cnt,
                        "fe": self.stats.invalid_samples_obstacles,
                        "vi": self.stats.invalid_samples_connections,
                        "c_max": self.planner.c_max,
                    }
                )
                pbar.refresh()

        self.planner.terminates_hook()
