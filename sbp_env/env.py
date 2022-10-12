#!/usr/bin/env python
import logging
import math
import random
import time
import re
from typing import Optional, List, Union

import numpy as np
from tqdm import tqdm

from .utils.common import Node, MagicDict, Stats
from .utils.csv_stats_logger import setup_csv_stats_logger, get_non_existing_filename
from .visualiser import VisualiserSwitcher

from . import collisionChecker

LOGGER = logging.getLogger(__name__)


class Env:
    """Represents the planning environment.
    The main planning loop happens inside this class.
    """

    def __init__(self, args: MagicDict, fixed_seed: int = None):
        """
        :param args: the dictionary of arguments to config the planning problem
        :param fixed_seed: if given, fix the random seed
        """
        self.started = False

        if fixed_seed is not None:
            np.random.seed(fixed_seed)
            random.seed(fixed_seed)
            print(f"Fixed random seed: {fixed_seed}")

        # initialize and prepare screen
        self.args = args
        self.args.stats = Stats(showSampledPoint=self.args.showSampledPoint)

        # cc_type, self.dist = {
        #     "image": (collisionChecker.ImgCollisionChecker, self.euclidean_dist),
        #     "4d": (collisionChecker.RobotArm4dCollisionChecker, self.euclidean_dist),
        #     "klampt": (collisionChecker.KlamptCollisionChecker, self.radian_dist),
        # }[self.args.engine]
        # self.cc = cc_type(self.args.image, stats=self.args.stats, args=self.args)
        from . import engine

        # setup visualiser
        if self.args.no_display:
            # use pass-through visualiser
            VisualiserSwitcher.choose_visualiser("base")

        # self.args.engine.get_dimension() = self.cc.get_dimension()
        # self.dim = self.cc.get_image_shape()
        # self.args["cc"] = self.cc

        args["sampler"] = args.sampler_data_pack.sampler_class(
            # sanitise keyword arguments by removing prefix -- and replacing - as _
            **{re.sub(r"^--", "", k).replace("-", "_"): v for (k, v) in args.items()},
        )

        def parse_input_pt(pt_as_str):
            if pt_as_str is None:
                return None
            pt = pt_as_str.split(",")
            if len(pt) != self.args.engine.get_dimension():
                raise RuntimeError(
                    f"Expected to have number of dimension = {self.args.engine.get_dimension()}, "
                    f"but "
                    f"was n={len(pt)} from input '{pt_as_str}'"
                )
            return tuple(map(float, pt))

        if type(self.args["start_pt"]) is str:
            self.args["start_pt"] = parse_input_pt(self.args["start_pt"])
        if type(self.args["goal_pt"]) is str:
            self.args["goal_pt"] = parse_input_pt(self.args["goal_pt"])

        self.args.planner = self.args.planner_data_pack.planner_class(**self.args)
        self.args["planner"] = self.args.planner

        self.planner = self.args.planner
        self.planner.args.env = self

        self.visualiser = VisualiserSwitcher.env_clname(env_instance=self)
        self.visualiser.visualiser_init(no_display=self.args["no_display"])
        start_pt, goal_pt = self.visualiser.set_start_goal_points(
            start=self.args["start_pt"], goal=self.args["goal_pt"]
        )
        if not self.args.engine.cc.feasible(start_pt):
            raise ValueError(f"The given start conf. is not feasible {start_pt}.")
        if not self.args.engine.cc.feasible(goal_pt):
            raise ValueError(f"The given goal conf. is not feasible {goal_pt}.")

        self.start_pt = self.goal_pt = None
        if start_pt is not None:
            self.start_pt = Node(start_pt)
        if goal_pt is not None:
            self.goal_pt = Node(goal_pt)
        self.start_pt.is_start = True
        self.goal_pt.is_goal = True
        self.planner.add_newnode(self.start_pt)
        self.visualiser.update_screen(update_all=True)
        # update the string pt to object
        self.args["start_pt"] = self.start_pt
        self.args["goal_pt"] = self.goal_pt

        self.planner.init(env=self, **self.args)
        if isinstance(self.args.engine, engine.KlamptEngine):
            self.args.sampler.set_use_radian(True)
            if self.args["epsilon"] > 1:
                import warnings

                warnings.warn(
                    f"Epsilon value is very high at {self.args['epsilon']} ("
                    f">than 1.0). It might not work well as klampt uses "
                    f"radian for joints value"
                )
            if self.args["radius"] > 2:
                import warnings

                warnings.warn(
                    f"Radius value is very high at {self.args['radius']} ("
                    f">than 2.0). It might not work well as klampt uses "
                    f"radian for joints value"
                )

    def __getattr__(self, attr):
        """This is called what self.attr doesn't exist.
        Forward the call to the visualiser instance
        """
        return object.__getattribute__(self.visualiser, attr)

    @property
    def sampler(self):
        """Pass through attribute access to sampler."""
        return self.args.sampler

    @staticmethod
    def radian_dist(p1: np.ndarray, p2: np.ndarray):
        """Return the (possibly wrapped) distance between two vector of angles in
        radians.

        :param p1: first configuration :math:`q_1`
        :param p2: second configuration :math:`q_2`

        """
        # https://stackoverflow.com/questions/28036652/finding-the-shortest-distance-between-two-angles/28037434
        diff = (p2 - p1 + np.pi) % (2 * np.pi) - np.pi
        diff = np.where(diff < -np.pi, diff + (2 * np.pi), diff)
        return np.linalg.norm(diff)

    def step_from_to(self, p1: np.ndarray, p2: np.ndarray):
        """Get a new point from p1 to p2, according to step size.

        :param p1: first configuration :math:`q_1`
        :param p2: second configuration :math:`q_2`

        """
        if self.args.ignore_step_size:
            return p2
        if np.isclose(p1, p2).all():
            # p1 is at the same point as p2
            return p2
        unit_vector = p2 - p1
        unit_vector = unit_vector / np.linalg.norm(unit_vector)
        step_size = self.args.engine.dist(p1, p2)
        step_size = min(step_size, self.args.epsilon)
        return p1 + step_size * unit_vector

    def run(self):
        """Run until we reached the specified max nodes"""
        self.started = True

        if self.args.save_output:
            setup_csv_stats_logger(
                get_non_existing_filename(
                    self.args.output_dir + "/%Y-%m-%d_%H-%M{}.csv"
                )
            )
            csv_logger = logging.getLogger("CSV_STATS")
            csv_logger.info(
                [
                    "nodes",
                    "time",
                    "cc_feasibility",
                    "cc_visibility",
                    "invalid_feasibility",
                    "invalid_visibility",
                    "c_max",
                ]
            )
            start_time = time.time()

        with tqdm(
            total=self.args.max_number_nodes, desc=self.args.sampler.name
        ) as pbar:
            while self.args.stats.valid_sample < self.args.max_number_nodes:
                self.visualiser.update_screen()
                self.planner.run_once()

                pbar.n = self.args.stats.valid_sample

                pbar.set_postfix(
                    {
                        "cc_fe": self.args.stats.feasible_cnt,
                        "cc_vi": self.args.stats.visible_cnt,
                        "fe": self.args.stats.invalid_samples_obstacles,
                        "vi": self.args.stats.invalid_samples_connections,
                        "c_max": self.planner.c_max,
                    }
                )
                if self.args.save_output:
                    csv_logger = logging.getLogger("CSV_STATS")
                    csv_logger.info(
                        [
                            self.args.stats.valid_sample,
                            time.time() - start_time,
                            self.args.stats.feasible_cnt,
                            self.args.stats.visible_cnt,
                            self.args.stats.invalid_samples_obstacles,
                            self.args.stats.invalid_samples_connections,
                            self.planner.c_max,
                        ]
                    )
                pbar.refresh()
                if self.args.first_solution and self.planner.c_max < float("inf"):
                    break

        self.visualiser.terminates_hook()

    def get_solution_path(
        self, as_array: bool = False
    ) -> Optional[Union[np.ndarray, List[Node]]]:
        if self.planner.c_max >= float("inf"):
            return None
        nn = self.planner.goal_pt
        path = []
        while True:
            path.append(nn)
            if nn.is_start:
                break
            nn = nn.parent
        path = reversed(path)
        if as_array:
            path = np.array([n.pos for n in path])
        else:
            path = list(path)
        return path
