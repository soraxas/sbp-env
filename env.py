#!/usr/bin/env python
import logging
import math
import random

import numpy as np
from scipy import spatial

import collisionChecker
from utils.helpers import Node, MagicDict, Stats
from visualiser import VisualiserSwitcher

LOGGER = logging.getLogger(__name__)


############################################################


class Env(VisualiserSwitcher.env_clname):
    def __init__(self, writer=None, fname=None, fixed_seed=None, **kwargs):
        self.writer = writer
        self.fname = fname

        if fixed_seed is not None:
            np.random.seed(fixed_seed)
            random.seed(fixed_seed)
            print(f"Fixed random seed: {fixed_seed}")

        # initialize and prepare screen
        self.args = MagicDict(kwargs)
        self.stats = Stats(showSampledPoint=self.args.showSampledPoint)

        if kwargs["engine"] == "image":
            self.cc = collisionChecker.ImgCollisionChecker(self.args.image)
            if self.args.image == "maps/4d.png":
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

    ############################################################

    @staticmethod
    def radian_dist(p1, p2):
        # distance metric for angles
        # https://en.wikipedia.org/wiki/Cosine_similarity#Angular_distance_and_similarity

        cosine_similarity = 1 - spatial.distance.cosine(p1, p2)
        return np.arccos(cosine_similarity) / np.pi
        # return spatial.distance.cosine(p1, p2)

    @staticmethod
    def euclidean_dist(p1, p2):
        # THIS IS MUCH SLOWER for small array
        # return np.linalg.norm(p1 - p2)

        p = p1 - p2
        return math.sqrt(p[0] ** 2 + p[1] ** 2)

    def step_from_to(self, p1, p2):
        """Get a new point from p1 to p2, according to step size."""
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
        from tqdm import tqdm
        import csv
        from timeit import default_timer

        starttime = default_timer()

        cur = -1
        # with open(f'out_stats-{self.args.sampler}.csv', 'a') as f:
        with open(f"out_stats.csv", "a") as f:
            writer = csv.writer(f)
            writer.writerow(
                [self.args.sampler.__class__.__name__, self.start_pt, self.goal_pt]
            )
            writer.writerow(
                [
                    "n_node",
                    "time",
                    "n_samp_cnt",
                    "i.con",
                    "i.obs",
                    "n_fe",
                    "n_vi",
                    "c_max",
                ]
            )
            mark_at = 0
            TERMINATED = False
            with tqdm(
                total=self.args.max_number_nodes, desc=type(self.args.sampler).__name__
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
                    if self.stats.valid_sample >= mark_at:
                        mark_at += 25
                        # writer.writerow(
                        #     [
                        #         self.stats.valid_sample,
                        #         default_timer() - starttime,
                        #         self.stats.invalid_samples_connections
                        #         + self.stats.invalid_samples_obstacles
                        #         + self.stats.valid_sample,
                        #         self.stats.invalid_samples_connections,
                        #         self.stats.invalid_samples_obstacles,
                        #         self.stats.feasible_cnt,
                        #         self.stats.visible_cnt,
                        #         self.planner.c_max,
                        #     ]
                        # )
                    pbar.refresh()

                    # write basic information
                    # if self.stats.valid_sample > cur:
                    #     curtime = default_timer()
                    #     self.writer.writerow([
                    #         curtime - starttime,
                    #         self.stats.valid_sample,
                    #         self.stats.invalid_samples_connections,
                    #         self.stats.invalid_samples_obstacles,
                    #         self.stats.sampler_success,
                    #         self.stats.sampler_success_all,
                    #         self.stats.sampler_fail,
                    #         self.stats.feasible_cnt,
                    #         self.stats.visible_cnt,
                    #         self.planner.c_max,
                    #     ])
                    #     cur = self.stats.valid_sample

        self.planner.terminates_hook()
        # self.write_solution_to_file(self.fname)

    # def write_solution_to_file(self, file_name):
    #
    #     with open(f"{file_name}.xml", 'w') as f:
    #         if self.planner.c_max == float('inf'):
    #             f.write('nope')
    #             return
    #         for n in self.planner.get_solution_path():
    #             f.write('12 ')
    #             f.write(" ".join(map(str, self.cc._translate_to_klampt(n.pos))))
    #             f.write("\n")
