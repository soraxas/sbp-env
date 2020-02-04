#!/usr/bin/env python
import logging
import math

from scipy import spatial

from collisionChecker import *
from helpers import Node, MagicDict, Stats
from pygamevisualiser import VisualiserSwitcher

LOGGER = logging.getLogger(__name__)

############################################################


class Env(VisualiserSwitcher.env_clname):
    def __init__(self,
                 startPt=None,
                 goalPt=None,
                 writer=None,
                 fname=None,
                 **kwargs):
        self.writer = writer
        self.fname = fname
        # initialize and prepare screen
        self.args = MagicDict(kwargs)
        self.stats = Stats(showSampledPoint=self.args.showSampledPoint)

        if kwargs['engine'] == 'image':
            self.cc = ImgCollisionChecker(self.args.image)
            self.dist = self.euclidean_dist
        elif kwargs['engine'] == 'klampt':
            self.cc = KlamptCollisionChecker(self.args.image, self.stats)
            self.dist = self.radian_dist

        kwargs['num_dim'] = self.cc.get_dimension()
        kwargs['image_shape'] = self.cc.get_image_shape()
        kwargs['cc'] = self.cc

        self.args.planner = kwargs['planner_type'](**kwargs)
        kwargs['planner'] = self.args.planner

        self.planner = self.args.planner
        self.planner.args.env = self

        super().__init__(**kwargs)

        self.visualiser_init(no_display=kwargs['no_display'])
        startPt, goalPt = self.set_start_goal_points(start=startPt,
                                                     goal=goalPt)

        self.startPt = self.goalPt = None
        if startPt:
            self.startPt = Node(startPt)
        if goalPt:
            self.goalPt = Node(goalPt)
        self.planner.add_newnode(self.startPt)
        self.update_screen(update_all=True)

        self.planner.init(
            env=self,
            startPt=self.startPt,
            goalPt=self.goalPt,
            **kwargs)
        if kwargs['engine'] == 'klampt':
            self.args.sampler.set_use_radian(True)

    ############################################################

    @staticmethod
    def radian_dist(p1, p2):
        # distance metric for angles
        # https://en.wikipedia.org/wiki/Cosine_similarity#Angular_distance_and_similarity

        cosine_similarity = 1 - spatial.distance.cosine(p1, p2)
        return np.arccos(cosine_similarity) / np.pi

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
        pbar = tqdm(total=self.args.max_number_nodes)

        from timeit import default_timer
        starttime = default_timer()

        cur = -1
        while self.stats.valid_sample < self.args.max_number_nodes:
            self.update_screen()
            self.planner.run_once()

            pbar.n = self.stats.valid_sample

            pbar.set_postfix({
                # 'i.con' : self.stats.invalid_samples_connections,
                # 'i.obs' : self.stats.invalid_samples_obstacles,
                # 'su' : self.stats.sampler_success,
                # 'sua' : self.stats.sampler_success_all,
                # 'sf' : self.stats.sampler_fail,
                'feasible' : self.stats.feasible_cnt,
                'visible' : self.stats.visible_cnt,
                'c_max' : self.planner.c_max,
            })

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
        self.write_solution_to_file(self.fname)


    def write_solution_to_file(self, file_name):

        with open(f"{file_name}.xml", 'w') as f:
            if self.planner.c_max == float('inf'):
                f.write('nope')
                return
            for n in self.planner.get_solution_path():
                f.write('12 ')
                f.write(" ".join(map(str, self.cc._translate_to_klampt(n.pos))))
                f.write("\n")