import random
from overrides import overrides
from baseSampler import Sampler
from randomPolicySampler import RandomPolicySampler
from randomness import SUPPORTED_RANDOM_METHODS, RandomnessManager
import numpy as np
import networkx as nx


volume_of_unit_ball = {
    1: 2,
    2: 3.142,
    3: 4.189,
    4: 4.935,
    5: 5.264,
    6: 5.168,
    7: 4.725,
    8: 4.059,
    9: 3.299,
    10: 2.550
}


class PRMSampler(Sampler):

    def get_free_area(self):
        area = 0
        for i in range(self.rrt.XDIM):
            for j in range(self.rrt.YDIM):
                color = self.rrt.img.get_at((i, j))
                if color != (255, 255, 255) and color != (255, 255, 255, 255):
                    area += 1
        return area


    @overrides
    def init(self, **kwargs):
        super().init(**kwargs)
        self.randomSampler = RandomPolicySampler()
        kwargs['goalBias'] = 0
        self.randomSampler.init(**kwargs)

        # Monkey patch the RRT for this smapler's specific stuff
        import types
        self.rrt.run = types.MethodType(prm_patched_run, self.rrt)
        self.rrt.run_once = types.MethodType(prm_patched_run_once, self.rrt)
        self.rrt.redraw_paths = types.MethodType(prm_patched_redraw_paths, self.rrt)

        self.rrt.stats.invalid_samples_connections = '-- '

        self.space_dim = 2
        self.d_threshold = self.rrt.EPSILON ###############################################################
        # self.gamma = 1 + np.power(2, self.space_dim) * (1 + 1.0 / self.space_dim) * 10000
        self.gamma = 2 * np.power((1 + 1 / self.space_dim), 1/self.space_dim) * np.power((self.get_free_area() / volume_of_unit_ball[self.space_dim]), 1/self.space_dim)

        self.tree = nx.DiGraph()
        self.tree.add_node(self.rrt.startPt)
        self.rrt.end_state = None

    def clear_graph(self):
        self.tree = nx.DiGraph()
        self.tree.add_node(self.rrt.startPt)
        self.rrt.end_state = None


    # def update_shortest_path(self):
    #     solution_path = nx.shortest_path(self.tree, self.rrt.startPt, self.rrt.end_state)
    #     solution_path[0].cost = 0
    #     for i in range(1, len(solution_path)):
    #         solution_path[i].parent = solution_path[i-1]
    #         solution_path[i].cost = solution_path[i-1].cost + dist(solution_path[i].pos, solution_path[i-1].pos)
    #     self.rrt.c_max = self.rrt.end_state.cost
    #     self.rrt.draw_solution_path()

    def build_graph(self):
        for v in self.rrt.nodes:

            n = len(self.rrt.nodes)
            radius = self.gamma * np.power( np.log(n + 1) / (n + 1), 1 / self.space_dim)

            m_near = nearest_neighbours(self.rrt.nodes, self.rrt.poses, v.pos, radius)

            for m_g in m_near:
                if m_g is v:
                    continue
                # check if path between(m_g,m_new) defined by motion-model is collision free
                if not self.rrt.cc.path_is_free(m_g.pos, v.pos):
                    continue
                self.tree.add_weighted_edges_from([(m_g, v, dist(m_g.pos, v.pos))])

    def get_nearest_free(self, node, neighbours):
        nn = None
        min_cost = 999999
        for n in neighbours:
            if n is self.rrt.startPt or n is self.rrt.goalPt or  n is node:
                continue
            if not self.rrt.cc.path_is_free(node.pos, n.pos):
                continue
            if nn is None:
                nn = n
                min_cost = dist(node.pos, n.pos)
            else:
                _cost = dist(node.pos, n.pos)
                if _cost < min_cost:
                    min_cost = _cost
                    nn = n
        return nn


    def get_solution(self):
        # get two nodes that is cloest to start/goal and are free routes
        m_near = nearest_neighbours(self.rrt.nodes, self.rrt.poses, self.startPt, self.rrt.EPSILON)
        start = self.get_nearest_free(self.rrt.startPt, m_near)
        m_near = nearest_neighbours(self.rrt.nodes, self.rrt.poses, self.goalPt, self.rrt.EPSILON)
        goal = self.get_nearest_free(self.rrt.goalPt, m_near)

        if start is None or goal is None or not nx.has_path(self.tree, start, goal):
            return False

        solution_path = nx.shortest_path(self.tree, start, goal)
        solution_path[0].cost = dist(solution_path[0].pos, start.pos)
        for i in range(1, len(solution_path)):
            solution_path[i].parent = solution_path[i-1]
            solution_path[i].cost = solution_path[i-1].cost + dist(solution_path[i].pos, solution_path[i-1].pos)
        self.rrt.c_max = goal.cost
        self.rrt.goalPt.parent = goal
        start.parent = self.rrt.startPt
        self.rrt.draw_solution_path()
        return self.rrt.c_max


    @overrides
    def get_next_pos(self):
        # Random pathx`
        while True:
            p = self.randomSampler.get_next_pos()[0]
            return p, self.report_success, self.report_fail



from rrtstar import Node, dist, GOAL_RADIUS

def nearest_neighbours(nodes, poses, pos, radius):
    distances = np.linalg.norm(poses[:len(nodes)] - pos, axis=1)
    neighbours = []
    for i, d in enumerate(distances):
        if d < radius:
            neighbours.append(nodes[i])
    return neighbours

def prm_patched_run(self):
    """Run until we reached the specified max nodes"""
    while self.stats.valid_sample < self.NUMNODES:
        self.process_pygame_event()
        self.update_screen()
        self.run_once()
    self.sampler.build_graph()
    self.sampler.get_solution()
    self.update_screen()

    import time
    time.sleep(30)

def prm_patched_run_once(self):
    rand_pos, _, _ = self.sampler.get_valid_next_pos()
    self.stats.add_free()
    self.add_newnode(Node(rand_pos))


import pygame
def prm_patched_redraw_paths(self):
    drawn_nodes_pairs = set()
    edges = list(self.sampler.tree.edges)
    # print(edges)
    for n in self.nodes:
        self.draw_circle(pos=n.pos, colour=(0,0,255), radius=1.4, layer=self.path_layers)
    for edge in edges:
        edge = np.array(edge).transpose()
        self.draw_path(edge[0], edge[1])

    #                 self.draw_path(n, n.parent)
