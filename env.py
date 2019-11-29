#!/usr/bin/env python
import logging
import sys
from math import atan2, cos, sin

from collisionChecker import *
from helpers import Node, check_pygame_enabled, MagicDict, Stats
from pygamevisualiser import PygameEnvVisualiser, VisualiserSwitcher

LOGGER = logging.getLogger(__name__)

############################################################


class Env(VisualiserSwitcher.env_clname):
    def __init__(self,
                 startPt=None,
                 goalPt=None,
                 **kwargs):
        # initialize and prepare screen
        self.args = MagicDict(kwargs)
        self.cc = ImgCollisionChecker(self.args.image)
        self.dim = self.cc.get_dimension()
        kwargs['num_dim'] = len(self.dim)
        self.stats = Stats(showSampledPoint=self.args.showSampledPoint)

        self.planner = self.args.planner
        self.planner.args.env = self

        super().__init__(**kwargs)

        self.visualiser_init(kwargs['enable_pygame'])
        self.startPt = self.goalPt = None
        if startPt:
            self.startPt = Node(startPt)
        if goalPt:
            self.goalPt = Node(goalPt)
        self.set_start_goal_points()
        self.planner.add_newnode(self.startPt)

        self.planner.init(
            env=self,
            startPt=self.startPt,
            goalPt=self.goalPt,
            **kwargs)

    ############################################################

    @staticmethod
    def dist(p1, p2):
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
        while self.stats.valid_sample < self.args.max_number_nodes:
            self.update_screen()
            self.planner.run_once()
        self.planner.terminates_hook()
