import networkx as nx
import numpy as np
from overrides import overrides

from env import Node
from planners.randomPolicySampler import RandomPolicySampler
from planners.rrtPlanner import RRTPlanner
from utils import planner_registry

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
    10: 2.550,
}


class PRMSampler(RandomPolicySampler):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    @overrides
    def init(self, **kwargs):
        kwargs["goalBias"] = 0
        super().init(**kwargs)


def nearest_neighbours(nodes, poses, pos, radius):
    distances = np.linalg.norm(poses[: len(nodes)] - pos, axis=1)
    neighbours = []
    for i, d in enumerate(distances):
        if d < radius:
            neighbours.append(nodes[i])
    return neighbours


class PRMPlanner(RRTPlanner):
    @overrides
    def init(self, *argv, **kwargs):
        super().init(*argv, **kwargs)
        self.args.env.stats.invalid_samples_connections = "-- "

        self.d_threshold = self.args.epsilon
        ###############################################################
        self.gamma = (
            1 + np.power(2, kwargs["num_dim"]) * (1 + 1.0 / kwargs["num_dim"]) * 10000
        )
        # self.gamma = 2 * np.power(
        #     (1 + 1 / self.args.num_dim), 1 / self.args.num_dim) * np.power(
        #         (self.get_free_area() / volume_of_unit_ball[self.args.num_dim]),
        #         1 / self.args.num_dim)

        self.graph = nx.DiGraph()
        self.graph.add_node(self.args.env.start_pt)
        self.args.env.end_state = None

    @overrides
    def run_once(self):
        rand_pos, _, _ = self.args.sampler.get_valid_next_pos()
        self.args.env.stats.add_free()
        self.add_newnode(Node(rand_pos))

    def get_free_area(self):
        area = 0
        for i in range(self.args.env.dim[0]):
            for j in range(self.args.env.dim[1]):
                color = self.args.env.img.get_at((i, j))
                if color != (255, 255, 255) and color != (255, 255, 255, 255):
                    area += 1
        return area

    def clear_graph(self):
        self.graph = nx.DiGraph()
        self.graph.add_node(self.args.env.start_pt)
        self.args.env.end_state = None

    def build_graph(self):
        for v in self.nodes:

            n = len(self.nodes)
            radius = self.gamma * np.power(
                np.log(n + 1) / (n + 1), 1 / self.args.num_dim
            )

            m_near = nearest_neighbours(self.nodes, self.poses, v.pos, radius)
            for m_g in m_near:
                if m_g is v:
                    continue
                # check if path between(m_g,m_new) defined by motion-model is collision free
                if not self.args.env.cc.visible(m_g.pos, v.pos):
                    continue
                self.graph.add_weighted_edges_from(
                    [(m_g, v, self.args.env.dist(m_g.pos, v.pos))]
                )

    def get_nearest_free(self, node, neighbours):
        nn = None
        min_cost = 999999
        for n in neighbours:
            if n is self.args.env.start_pt or n is self.args.env.goal_pt or n is node:
                continue
            if not self.args.env.cc.visible(node.pos, n.pos):
                continue
            if nn is None:
                nn = n
                min_cost = self.args.env.dist(node.pos, n.pos)
            else:
                _cost = self.args.env.dist(node.pos, n.pos)
                if _cost < min_cost:
                    min_cost = _cost
                    nn = n
        return nn

    def get_solution(self):
        # get two nodes that is cloest to start/goal and are free routes
        m_near = nearest_neighbours(
            self.nodes, self.poses, self.args.sampler.start_pos, self.args.epsilon
        )
        start = self.get_nearest_free(self.args.env.start_pt, m_near)
        m_near = nearest_neighbours(
            self.nodes, self.poses, self.args.sampler.goal_pos, self.args.epsilon
        )
        goal = self.get_nearest_free(self.args.env.goal_pt, m_near)

        if start is None or goal is None or not nx.has_path(self.graph, start, goal):
            return False

        solution_path = nx.shortest_path(self.graph, start, goal)
        solution_path[0].cost = self.args.env.dist(solution_path[0].pos, start.pos)
        for i in range(1, len(solution_path)):
            solution_path[i].parent = solution_path[i - 1]
            solution_path[i].cost = solution_path[i - 1].cost + self.args.env.dist(
                solution_path[i].pos, solution_path[i - 1].pos
            )
        self.c_max = goal.cost
        self.args.env.goal_pt.parent = goal
        start.parent = self.args.env.start_pt
        self.draw_solution_path()
        return self.c_max


def pygame_prm_planner_paint(planner):
    from utils.helpers import Colour

    for n in planner.nodes:
        planner.args.env.draw_circle(
            pos=n.pos,
            colour=(0, 0, 255),
            radius=1.4,
            layer=planner.args.env.path_layers,
        )


def pygame_prm_planner_paint_when_terminate(planner):
    from utils.helpers import Colour

    planner.build_graph()
    # draw all edges
    for n1, n2 in planner.graph.edges():
        planner.args.env.draw_path(n1, n2, Colour.path_blue)
    planner.get_solution()
    planner.args.env.update_screen()

    input("\nPress Enter to quit...")


planner_registry.register_sampler(
    "prm_sampler", sampler_class=PRMSampler,
)

planner_registry.register_planner(
    "prm",
    planner_class=PRMPlanner,
    visualise_pygame_paint=pygame_prm_planner_paint,
    visualise_pygame_paint_terminate=pygame_prm_planner_paint_when_terminate,
    sampler_id="prm_sampler",
)
