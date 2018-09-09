import random
from overrides import overrides
from baseSampler import Sampler
from randomPolicySampler import RandomPolicySampler
from randomness import SUPPORTED_RANDOM_METHODS, RandomnessManager
import numpy as np


class BiRRTSampler(Sampler):

    @overrides
    def init(self, **kwargs):
        super().init(**kwargs)
        self.randomSampler = RandomPolicySampler()
        self.randomSampler.init(**kwargs)

        # Monkey patch the RRT for this smapler's specific stuff
        import types
        self.rrt.run_once = types.MethodType(birrt_patched_run_once, self.rrt)
        self.rrt.redraw_paths = types.MethodType(patched_redraw_paths, self.rrt)

        self.goal_tree_nodes = []
        self.goal_tree_poses = np.empty((self.rrt.NUMNODES+50,2))  # +50 to prevent over flow

        self.goal_tree_nodes.append(self.rrt.goalPt)
        self.goal_tree_poses[0] = self.rrt.goalPt.pos

        self.goal_tree_turn = False
        self.found_solution = False


    @overrides
    def get_next_pos(self):
        # Random path
        while True:
            if random.random() < self.goalBias:
                # init/goal bias
                if self.goal_tree_turn:
                    p = self.startPt
                else:
                    p = self.goalPt
            else:
                p = self.randomSampler.get_next_pos()[0]
                # p[0] *= self.XDIM
                # p[1] *= self.YDIM
            return p, self.report_success, self.report_fail


from rrtstar import Node, dist, GOAL_RADIUS

def birrt_patched_run_once(self):
    if self.sampler.goal_tree_turn and not self.sampler.found_solution:
        # extend from goal tree
        poses = self.sampler.goal_tree_poses
        nodes = self.sampler.goal_tree_nodes
    else:
        # extend from init tree
        poses = self.poses
        nodes = self.nodes
    self.sampler.goal_tree_turn = not self.sampler.goal_tree_turn
    # check two tree join together

    ###################################################################
    ###################################################################
    # Get an sample that is free (not in blocked space)
    rand_pos, _, _ = self.sampler.get_valid_next_pos()
    # Found a node that is not in X_obs

    idx = self.find_nearest_neighbour_idx(rand_pos, poses[:len(nodes)])
    nn = nodes[idx]
    # get an intermediate node according to step-size
    newpos = self.step_from_to(nn.pos, rand_pos)
    # check if it has a free path to nn or not
    if not self.cc.path_is_free(nn.pos, newpos):
        self.stats.add_invalid(obs=False)
    else:
        newnode = Node(newpos)
        self.stats.add_free()

        ######################
        newnode, nn = self.choose_least_cost_parent(newnode, nn, nodes=nodes)
        poses[len(nodes)] = newnode.pos

        nodes.append(newnode)
        # rewire to see what the newly added node can do for us
        self.rewire(newnode, nodes)
        self.draw_path(nn, newnode)

        ###################################################################
        # check if two tree joins
        if not self.sampler.found_solution:
            if nodes is self.nodes:
                other_poses = self.sampler.goal_tree_poses
                other_nodes = self.sampler.goal_tree_nodes
            else:
                other_poses = self.poses
                other_nodes = self.nodes
            distances = np.linalg.norm(other_poses[:len(self.nodes)] - newpos, axis=1)
            if min(distances) < self.EPSILON:
                idx = np.argmin(distances)
                if self.cc.path_is_free(other_poses[idx], newpos):

                    self.sampler.found_solution = True
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
                    assert goal_tree_node in self.sampler.goal_tree_nodes


                    to_be_removed = []
                    while _old_parent is not None:
                        _old_parent = _nextnode.parent

                        _nextnode, nn = self.choose_least_cost_parent(_nextnode, nn=nn, nodes=self.nodes)
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


def patched_redraw_paths(self):
    drawn_nodes_pairs = set()
    for nodes in (self.nodes, self.sampler.goal_tree_nodes):
        for n in nodes:
            if n.parent is not None:
                new_set = frozenset({n, n.parent})
                if new_set not in drawn_nodes_pairs:
                    drawn_nodes_pairs.add(new_set)
                    self.draw_path(n, n.parent)
