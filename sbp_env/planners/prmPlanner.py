from typing import List

import networkx as nx
import numpy as np
from overrides import overrides
import tqdm

from ..utils.common import Node
from ..planners.rrtPlanner import RRTPlanner
from ..samplers import prmSampler
from ..utils import planner_registry
from ..utils.common import Colour, Stats

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


def nearest_neighbours(
    nodes: List[Node], poses: np.ndarray, pos: np.ndarray, radius: float
):
    """A helper function to find the nearest neighbours from a roadmap

    :param nodes: the list of nodes to search against
    :param poses: array of positions
    :param pos: the position of interest
    :param radius: the maximum radius of distance

    """
    distances = np.linalg.norm(poses[: len(nodes)] - pos, axis=1)
    neighbours = []
    for i, d in enumerate(distances):
        if d < radius:
            neighbours.append(nodes[i])
    return neighbours


class PRMPlanner(RRTPlanner):
    """
    Probabilistic Roadmap motion planner, the multi-query sampling-based planner.
    """

    @overrides
    def init(self, *argv, **kwargs):
        super().init(*argv, **kwargs)

        self.d_threshold = self.args.epsilon
        self.gamma = (
            1
            + np.power(2, self.args.engine.get_dimension())
            * (1 + 1.0 / self.args.engine.get_dimension())
            * 10000
        )

        self.graph = nx.DiGraph()
        self.graph.add_node(self.args.env.start_pt)
        self.args.env.end_state = None

    @overrides
    def run_once(self):
        rand_pos, _, _ = self.args.sampler.get_valid_next_pos()
        Stats.get_instance().add_free()
        self.add_newnode(Node(rand_pos))

    def clear_graph(self):
        """Clear the current roadmap graph"""
        self.graph = nx.DiGraph()
        self.graph.add_node(self.args.env.start_pt)
        self.args.env.end_state = None

    def build_graph(self):
        """Performs the graph building process where

        .. math::
            G = (V, E).
        """
        n = len(self.nodes)
        radius = self.gamma * np.power(
            np.log(n + 1) / (n + 1), 1 / self.args.engine.get_dimension()
        )

        for v in tqdm.tqdm(self.nodes, desc="Building graph"):
            m_near = nearest_neighbours(self.nodes, self.poses, v.pos, radius)
            for m_g in m_near:
                if m_g is v:
                    continue
                # check if path between(m_g,m_new) defined by motion-model is collision free
                if not self.args.engine.cc.visible(m_g.pos, v.pos):
                    continue
                self.graph.add_weighted_edges_from(
                    [(m_g, v, self.args.engine.dist(m_g.pos, v.pos))]
                )

    def get_nearest_free(self, node: Node, neighbours: List[Node]):
        """Internal method to get the closest existing node that is free to connects
        to the given node.

        :param node: the node of interest
        :param neighbours: the list of nodes to search against

        """
        nn = None
        min_cost = 999999
        for n in neighbours:
            if n is self.args.env.start_pt or n is self.args.env.goal_pt or n is node:
                continue
            if not self.args.engine.cc.visible(node.pos, n.pos):
                continue
            if nn is None:
                nn = n
                min_cost = self.args.engine.dist(node.pos, n.pos)
            else:
                _cost = self.args.engine.dist(node.pos, n.pos)
                if _cost < min_cost:
                    min_cost = _cost
                    nn = n
        return nn

    def get_solution(self):
        """Build the solution path"""
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
            return float("inf")

        solution_path = nx.shortest_path(self.graph, start, goal)
        solution_path[0].cost = self.args.engine.dist(solution_path[0].pos, start.pos)
        for i in range(1, len(solution_path)):
            solution_path[i].parent = solution_path[i - 1]
            solution_path[i].cost = solution_path[i - 1].cost + self.args.engine.dist(
                solution_path[i].pos, solution_path[i - 1].pos
            )
        self.c_max = goal.cost
        self.args.env.goal_pt.parent = goal
        start.parent = self.args.env.start_pt
        self.visualiser.draw_solution_path()
        return self.c_max


def pygame_prm_planner_paint(planner):
    """Visualisation function to paint for planner

    :param planner: the planner to visualise

    """
    for n in planner.nodes:
        planner.args.env.draw_circle(
            pos=n.pos,
            colour=(0, 0, 255),
            radius=1.4,
            layer=planner.args.env.path_layers,
        )


def pygame_prm_planner_paint_when_terminate(planner):
    """Visualisation function to paint for planner when termiante

    :param planner: the planner to visualise

    """

    planner.build_graph()
    # draw all edges
    for n1, n2 in planner.graph.edges():
        planner.args.env.draw_path(n1, n2, Colour.path_blue)
    planner.get_solution()
    planner.args.env.update_screen()

    input("\nPress Enter to quit...")


def klampt_prm_paint(planner) -> None:
    """Visualiser paint function for PRM

    :param planner: the planner to be visualised

    """

    colour = (1, 0, 0, 1)
    for n in planner.nodes:
        planner.args.env.draw_node(
            planner.args.engine.cc.get_eef_world_pos(n.pos), colour=colour
        )
    for edge in planner.graph.edges:
        edge = np.array(edge).transpose()
        planner.args.env.draw_path(
            planner.args.engine.cc.get_eef_world_pos(n.pos),
            planner.args.engine.cc.get_eef_world_pos(n.parent.pos),
            colour=colour,
        )


def klampt_prm_planner_paint_when_terminate(planner):
    """Visualisation function to paint for planner when termiante

    :param planner: the planner to visualise

    """
    planner.build_graph()
    # draw all edges
    for n1, n2 in planner.tree.edges():
        planner.args.env.draw_path(n1, n2, Colour.path_blue)
    planner.get_solution()
    planner.args.env.update_screen()

    input("\nPress Enter to quit...")


# start register
planner_registry.register_planner(
    "prm",
    planner_class=PRMPlanner,
    visualise_pygame_paint=pygame_prm_planner_paint,
    visualise_pygame_paint_terminate=pygame_prm_planner_paint_when_terminate,
    visualise_klampt_paint=klampt_prm_paint,
    visualise_klampt_paint_terminate=klampt_prm_planner_paint_when_terminate,
    sampler_id=prmSampler.sampler_id,
)
# finish register
