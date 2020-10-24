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
        self.randomSampler.init(use_radian=self.use_radian, **kwargs)

    def set_use_radian(self, value=True):
        self.use_radian = value
        self.randomSampler.use_radian = value

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
        self.goal_tree_poses = np.empty((self.args.max_number_nodes + 50, # +50 to prevent over flow
                                         kwargs['num_dim']))
        self.goal_tree_nodes.append(self.args.env.goalPt)
        self.goal_tree_poses[0] = self.args.env.goalPt.pos

        self.found_solution = False
        self.goal_tree_turn = False


    def draw_potential(self, fac=.5):
        from sklearn.neighbors import KernelDensity

        def kde2D(x, y, bandwidth, xbins=100j, ybins=100j,
                  xmin=None, xmax=None, ymin=None, ymax=None,
                  **kwargs):
            from sklearn.neighbors import KernelDensity
            """Build 2D kernel density estimate (KDE)."""
            if xmin is None:
                xmin = x.min()
                xmax = x.max()
                ymin = y.min()
                ymax = y.max()
            # create grid of sample locations (default: 100x100)
            xx, yy = np.mgrid[xmin:xmax:xbins,
                     ymin:ymax:ybins]

            xy_sample = np.vstack([yy.ravel(), xx.ravel()]).T
            xy_train = np.vstack([y, x]).T

            kde_skl = KernelDensity(bandwidth=bandwidth, **kwargs)
            kde_skl.fit(xy_train)

            # score_samples() returns the log-likelihood of the samples
            z = np.exp(kde_skl.score_samples(xy_sample))
            return xx, yy, np.reshape(z, xx.shape)

        import numpy as np
        import matplotlib.pyplot as plt

        # plt.scatter(*np.array(self.bins).T)
        # plt.show()

        # m1 = np.random.normal(size=1000)
        # m2 = np.random.normal(scale=0.5, size=1000)
        #
        # x, y = m1 + m2, m1 - m2

        nodes_poses = np.vstack([self.poses[:len(self.nodes)],
                                 self.goal_tree_poses[:len(self.goal_tree_nodes)]])
        xx, yy, node_zz = kde2D(nodes_poses[:, 0], nodes_poses[:, 1], 20,
                                xmin=0, xmax=600, ymin=0, ymax=400)
        data = np.array(self.bins)
        xx, yy, samp_zz = kde2D(data[:, 0], data[:, 1], 30,
                                xmin=0, xmax=600, ymin=0, ymax=400)

        x = data[:, 0]
        y = data[:, 1]

        plt.pcolormesh(xx, yy, (samp_zz - fac * node_zz).clip(min=0))
        plt.colorbar()
        plt.gca().invert_yaxis()
        plt.scatter(x, y, s=2, facecolor='white', alpha=.2)

        plt.show()


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
        if not self.args.env.cc.visible(nn.pos, newpos):
            self.args.env.stats.add_invalid(obs=False)
            # try:
            #     self.bins
            # except AttributeError:
            #     self.bins = []
            # self.bins.append(rand_pos)
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
                    if self.args.env.cc.visible(other_poses[idx], newpos):

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
