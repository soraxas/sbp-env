"""Represent a planner."""
import numpy as np
import pygame

from checkCollision import *
from helpers import *


class RRTPlanner:
    """This planner is largely a RRT planner, though with extra features."""

    def __init__(self, **kwargs):
        self.args = MagicDict(kwargs)
        self.poses = np.empty((self.args.max_number_nodes + 50,
                               2))  # +50 to prevent over flow
        self.c_max = float('inf')
        # this dict is to temparily store distance of a new node to all others
        # so that in the next run other functions (eg choose_least_cost_parent and rwire)
        # and take advantage to the already computed values
        self._new_node_dist_to_all_others = {}
        self.nodes = []
        self.args.env = None  # will be set by env itself

    def init(self, *argv, **kwargs):
        # self.args.env = kwargs['RRT']
        self.args.sampler.init(*argv, **kwargs)
        self.startPt = kwargs['startPt']
        self.goalPt = kwargs['goalPt']

    def run_once(self):
        # Get an sample that is free (not in blocked space)
        rand_pos, report_success, report_fail = self.args[
            'sampler'].get_valid_next_pos()
        # Found a node that is not in X_obs
        idx = self.find_nearest_neighbour_idx(rand_pos,
                                              self.poses[:len(self.nodes)])
        nn = self.nodes[idx]
        # get an intermediate node according to step-size
        newpos = self.args.env.step_from_to(nn.pos, rand_pos)
        # check if it has a free path to nn or not
        if not self.args.env.cc.path_is_free(nn.pos, newpos):
            self.args.env.stats.add_invalid(obs=False)
            report_fail(pos=rand_pos, free=False)
        else:
            newnode = Node(newpos)
            self.args.env.stats.add_free()
            self.args.sampler.add_tree_node(newnode.pos)
            report_success(pos=newnode.pos, nn=nn, rand_pos=rand_pos)
            ######################
            newnode, nn = self.choose_least_cost_parent(
                newnode, nn, nodes=self.nodes)
            self.add_newnode(newnode)
            # rewire to see what the newly added node can do for us
            self.rewire(newnode, self.nodes)
            self.args.env.draw_path(nn, newnode)

            if dist(newnode.pos, self.goalPt.pos) < self.args.goal_radius:
                if newnode.cost < self.c_max:
                    self.c_max = newnode.cost
                    self.goalPt.parent = newnode
                    newnode.children.append(self.goalPt.parent)
                    self.draw_solution_path()

    def add_newnode(self, node):
        self.poses[len(self.nodes)] = node.pos
        self.nodes.append(node)

    def choose_least_cost_parent(self, newnode, nn=None, nodes=None):
        """Given a new node, a node from root, return a node from root that
        has the least cost (toward the newly added node)"""
        if nn is not None:
            _newnode_to_nn_cost = dist(newnode.pos, nn.pos)
        self._new_node_dist_to_all_others = {}
        for p in nodes:
            _newnode_to_p_cost = dist(newnode.pos, p.pos)
            self._new_node_dist_to_all_others[(newnode,
                                               p)] = _newnode_to_p_cost
            if _newnode_to_p_cost <= self.args[
                    'radius'] and self.args.env.cc.path_is_free(newnode.pos, p.pos):
                # This is another valid parent. Check if it's better than our current one.
                if nn is None or (p.cost + _newnode_to_p_cost <
                                  nn.cost + _newnode_to_nn_cost):
                    nn = p
                    _newnode_to_nn_cost = _newnode_to_p_cost
        if nn is None:
            raise LookupError(
                "ERROR: Provided nn=None, and cannot find any valid nn by this function. This newnode is not close to the root tree...?"
            )
        newnode.cost = nn.cost + dist(nn.pos, newnode.pos)
        newnode.parent = nn
        nn.children.append(newnode)

        return newnode, nn

    def rewire(self, newnode, nodes, already_rewired=None):
        """Reconsider parents of nodes that had change, so that the optimiality would change instantly"""
        if len(nodes) < 1:
            return
        if already_rewired is None:
            already_rewired = {newnode}
        for n in (x for x in nodes if x not in already_rewired):
            if len(already_rewired) <= 1:
                _newnode_to_n_cost = self._new_node_dist_to_all_others[newnode,
                                                                       n]
            else:
                _newnode_to_n_cost = dist(newnode.pos, n.pos)
            if (n != newnode.parent
                    and _newnode_to_n_cost <= self.args.radius
                    and self.args.env.cc.path_is_free(n.pos, newnode.pos)
                    and newnode.cost + _newnode_to_n_cost < n.cost):
                # draw over the old wire
                self.args.env.draw_path(n, n.parent, Colour.white)
                reconsider = (n.parent, *n.children)
                n.parent.children.remove(n)
                n.parent = newnode
                newnode.children.append(n)
                n.cost = newnode.cost + _newnode_to_n_cost
                already_rewired.add(n)
                self.args.env.draw_path(n, newnode, Colour.path_blue)
                self.rewire(n, reconsider, already_rewired=already_rewired)

    def terminates_hook(self):
        """For planner to process anything when planning terminates.
        RRT does nothing."""
        pass

    @staticmethod
    def find_nearest_neighbour_idx(pos, poses):
        # Make use of numpy fast parallel operation to find all distance with one operation.
        distances = np.linalg.norm(poses - pos, axis=1)
        return np.argmin(distances)


############################################################
##                    DRAWING RELATED                     ##
############################################################

    def paint(self):
        # these had already been drawn
        drawn_nodes_pairs = set()
        self.args.env.path_layers.fill(Colour.ALPHA_CK)
        # Draw path trees
        for n in self.nodes:
            if n.parent is not None:
                new_set = frozenset({n, n.parent})
                if new_set not in drawn_nodes_pairs:
                    drawn_nodes_pairs.add(new_set)
                    self.args.env.draw_path(n, n.parent)
        self.draw_solution_path()

    @check_pygame_enabled
    def draw_solution_path(self):
        if self.c_max == float('inf'):
            # nothing to d
            return

        # redraw new path
        self.args.env.solution_path_screen.fill(Colour.ALPHA_CK)
        nn = self.goalPt.parent
        self.c_max = nn.cost
        while nn != self.startPt:
            self.args.env.draw_path(
                nn,
                nn.parent,
                colour=Colour.blue,
                line_modifier=5,
                layer=self.args.env.solution_path_screen)
            nn = nn.parent
        self.args.env.window.blit(self.args.env.path_layers, (0, 0))
        self.args.env.window.blit(self.args.env.solution_path_screen, (0, 0))
