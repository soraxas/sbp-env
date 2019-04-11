import networkx as nx
import numpy as np
from overrides import overrides

from env import Node, dist
from planners.baseSampler import Sampler
from planners.randomPolicySampler import RandomPolicySampler
from planners.rrtPlanner import RRTPlanner

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
    @overrides
    def init(self, **kwargs):
        super().init(**kwargs)
        self.randomSampler = RandomPolicySampler()
        kwargs['goalBias'] = 0
        self.randomSampler.init(**kwargs)


def nearest_neighbours(nodes, poses, pos, radius):
    distances = np.linalg.norm(poses[:len(nodes)] - pos, axis=1)
    neighbours = []
    for i, d in enumerate(distances):
        if d < radius:
            neighbours.append(nodes[i])
    return neighbours


class PRMPlanner(RRTPlanner):
    @overrides
    def init(self, *argv, **kwargs):
        super().init(*argv, **kwargs)
        self.args.env.stats.invalid_samples_connections = '-- '

        self.space_dim = 2
        self.d_threshold = self.args.epsilon
        ###############################################################
        # self.gamma = 1 + np.power(2, self.space_dim) * (1 + 1.0 / self.space_dim) * 10000
        self.gamma = 2 * np.power(
            (1 + 1 / self.space_dim), 1 / self.space_dim) * np.power(
                (self.get_free_area() / volume_of_unit_ball[self.space_dim]),
                1 / self.space_dim)

        self.tree = nx.DiGraph()
        self.tree.add_node(self.args.env.startPt)
        self.args.env.end_state = None

    @overrides
    def terminates_hook(self):
        """Run until we reached the specified max nodes"""
        self.build_graph()
        self.get_solution()
        self.args.env.update_screen()

        import time
        time.sleep(30)

    @overrides
    def run_once(self):
        rand_pos, _, _ = self.args.sampler.get_valid_next_pos()
        self.args.env.stats.add_free()
        self.add_newnode(Node(rand_pos))

    def get_free_area(self):
        area = 0
        for i in range(self.args.env.XDIM):
            for j in range(self.args.env.YDIM):
                color = self.args.env.img.get_at((i, j))
                if color != (255, 255, 255) and color != (255, 255, 255, 255):
                    area += 1
        return area

    def clear_graph(self):
        self.tree = nx.DiGraph()
        self.tree.add_node(self.args.env.startPt)
        self.args.env.end_state = None

    def build_graph(self):
        for v in self.nodes:

            n = len(self.nodes)
            radius = self.gamma * np.power(
                np.log(n + 1) / (n + 1), 1 / self.space_dim)

            m_near = nearest_neighbours(self.nodes, self.poses, v.pos, radius)
            for m_g in m_near:
                if m_g is v:
                    continue
                # check if path between(m_g,m_new) defined by motion-model is collision free
                if not self.args.env.cc.path_is_free(m_g.pos, v.pos):
                    continue
                self.tree.add_weighted_edges_from([(m_g, v, dist(
                    m_g.pos, v.pos))])

    def get_nearest_free(self, node, neighbours):
        nn = None
        min_cost = 999999
        for n in neighbours:
            if (n is self.args.env.startPt or n is self.args.env.goalPt
                    or n is node):
                continue
            if not self.args.env.cc.path_is_free(node.pos, n.pos):
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
        m_near = nearest_neighbours(self.nodes, self.poses, self.args.sampler.start_pos,
                                    self.args.epsilon)
        start = self.get_nearest_free(self.args.env.startPt, m_near)
        m_near = nearest_neighbours(self.nodes, self.poses, self.args.sampler.goal_pos,
                                    self.args.epsilon)
        goal = self.get_nearest_free(self.args.env.goalPt, m_near)

        if start is None or goal is None or not nx.has_path(
                self.tree, start, goal):
            return False

        solution_path = nx.shortest_path(self.tree, start, goal)
        solution_path[0].cost = dist(solution_path[0].pos, start.pos)
        for i in range(1, len(solution_path)):
            solution_path[i].parent = solution_path[i - 1]
            solution_path[i].cost = solution_path[i - 1].cost + dist(
                solution_path[i].pos, solution_path[i - 1].pos)
        self.c_max = goal.cost
        self.args.env.goalPt.parent = goal
        start.parent = self.args.env.startPt
        self.draw_solution_path()
        return self.c_max

    @overrides
    def paint(self):
        drawn_nodes_pairs = set()
        edges = list(self.tree.edges)
        # print(edges)
        for n in self.nodes:
            self.args.env.draw_circle(
                pos=n.pos,
                colour=(0, 0, 255),
                radius=1.4,
                layer=self.args.env.path_layers)
        for edge in edges:
            edge = np.array(edge).transpose()
            self.args.env.draw_path(edge[0], edge[1])
