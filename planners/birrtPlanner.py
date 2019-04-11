import random

import numpy as np
from overrides import overrides

from env import Node
from planners.baseSampler import Sampler
from planners.randomPolicySampler import RandomPolicySampler
from planners.rrtPlanner import RRTPlanner


class BiRRTSampler(Sampler):
    @overrides
    def init(self, **kwargs):
        super().init(**kwargs)
        self.randomSampler = RandomPolicySampler()
        self.randomSampler.init(**kwargs)

    @overrides
    def get_next_pos(self):
        # Random path
        while True:
            if random.random() < self.args.goalBias:
                # init/goal bias
                if self.args.planner.goal_tree_turn:
                    p = self.start_pos
                else:
                    p = self.goal_pos
            else:
                p = self.randomSampler.get_next_pos()[0]
            return p, self.report_success, self.report_fail


class BiRRTPlanner(RRTPlanner):
    @overrides
    def init(self, *argv, **kwargs):
        super().init(*argv, **kwargs)
        self.goal_tree_nodes = []
        self.goal_tree_poses = np.empty((self.args.max_number_nodes + 50,
                                         2))  # +50 to prevent over flow
        self.goal_tree_nodes.append(self.args.env.goalPt)
        self.goal_tree_poses[0] = self.args.env.goalPt.pos

        self.found_solution = False
        self.goal_tree_turn = False


    @overrides
    def run_once(self):
        if self.goal_tree_turn and not self.found_solution:
            # extend from goal tree
            poses = self.goal_tree_poses
            nodes = self.goal_tree_nodes
        else:
            # extend from init tree
            poses = self.poses
            nodes = self.nodes
        self.goal_tree_turn = not self.goal_tree_turn
        # check two tree join together

        ###################################################################
        ###################################################################
        # Get an sample that is free (not in blocked space)
        rand_pos, _, _ = self.args.sampler.get_valid_next_pos()
        # Found a node that is not in X_obs

        idx = self.find_nearest_neighbour_idx(rand_pos, poses[:len(nodes)])
        nn = nodes[idx]
        # get an intermediate node according to step-size
        newpos = self.args.env.step_from_to(nn.pos, rand_pos)
        # check if it has a free path to nn or not
        if not self.args.env.cc.path_is_free(nn.pos, newpos):
            self.args.env.stats.add_invalid(obs=False)
        else:
            newnode = Node(newpos)
            self.args.env.stats.add_free()

            ######################
            newnode, nn = self.choose_least_cost_parent(
                newnode, nn, nodes=nodes)
            poses[len(nodes)] = newnode.pos

            nodes.append(newnode)
            # rewire to see what the newly added node can do for us
            self.rewire(newnode, nodes)
            self.args.env.draw_path(nn, newnode)

            ###################################################################
            # check if two tree joins
            if not self.found_solution:
                if nodes is self.nodes:
                    other_poses = self.goal_tree_poses
                    other_nodes = self.goal_tree_nodes
                else:
                    other_poses = self.poses
                    other_nodes = self.nodes
                distances = np.linalg.norm(
                    other_poses[:len(self.nodes)] - newpos, axis=1)
                if min(distances) < self.args.epsilon:
                    idx = np.argmin(distances)
                    if self.args.env.cc.path_is_free(other_poses[idx], newpos):

                        self.found_solution = True
                        # get the two closest nodes
                        if nodes is self.nodes:
                            init_tree_node = newnode
                            goal_tree_node = other_nodes[idx]
                        else:
                            init_tree_node = other_nodes[idx]
                            goal_tree_node = newnode
                        _nextnode = goal_tree_node  # keep track of original parent
                        _old_parent = _nextnode.parent

                        # trees joined! Flip all the parent as child
                        nn = init_tree_node

                        assert init_tree_node in self.nodes
                        assert goal_tree_node in self.goal_tree_nodes

                        to_be_removed = []
                        while _old_parent is not None:
                            _old_parent = _nextnode.parent

                            _nextnode, nn = self.choose_least_cost_parent(
                                _nextnode, nn=nn, nodes=self.nodes)
                            self.rewire(_nextnode, nodes=self.nodes)

                            self.poses[len(self.nodes)] = _nextnode.pos
                            self.nodes.append(_nextnode)
                            to_be_removed.append(_nextnode)

                            nn = _nextnode
                            _nextnode = _old_parent

            if self.goalPt.parent is not None:
                if self.goalPt.parent.cost < self.c_max:
                    self.c_max = self.goalPt.parent.cost
                    self.draw_solution_path()

    @overrides
    def paint(self):
        drawn_nodes_pairs = set()
        for nodes in (self.nodes, self.goal_tree_nodes):
            for n in nodes:
                if n.parent is not None:
                    new_set = frozenset({n, n.parent})
                    if new_set not in drawn_nodes_pairs:
                        drawn_nodes_pairs.add(new_set)
                        self.args.env.draw_path(n, n.parent)
