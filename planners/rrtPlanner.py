"""Represent a planner."""
import operator

import numpy as np
from helpers import MagicDict, Node
from planners.baseSampler import Planner
from rtree import index

class Tree():

    def __init__(self, dimension):
        p = index.Property()
        p.dimension = dimension
        self.V = index.Index(interleaved=True, properties=p)
        # self.V_raw = []
        self.E = {}  # edges in form E[child] = parent

    def add_vertex(self, v, pos):
        if len(pos) == 2:
            # print(v)
            # print(pos)
            # print(np.tile(pos, 2))
            self.V.insert(0, tuple(pos), v)
        else:
            self.V.insert(0, np.tile(pos, 2), v)
        # self.V_raw.append(v)

    def add_edge(self, child, parent):
        self.E[child] = parent

    def nearby(self, x, n):
        return self.V.nearest(np.tile(x, 2), num_results=n, objects="raw")

    def get_nearest(self, x):
        return next(self.nearby(x, 1))

    # def connect_to_point(self, tree, x_a, x_b):
    #     """
    #     Connect vertex x_a in tree to vertex x_b
    #     :param tree: int, tree to which to add edge
    #     :param x_a: tuple, vertex
    #     :param x_b: tuple, vertex
    #     :return: bool, True if able to add edge, False if prohibited by an obstacle
    #     """
    #     if self.V.count(x_b) == 0 and self.X.collision_free(x_a, x_b, self.r):
    #         self.add_vertex(tree, x_b)
    #         self.add_edge(tree, x_b, x_a)
    #         return True
    #     return False

    # def can_connect_to_goal(self, tree):
    #     """
    #     Check if the goal can be connected to the graph
    #     :param tree: rtree of all Vertices
    #     :return: True if can be added, False otherwise
    #     """
    #     x_nearest = self.get_nearest(tree, self.x_goal)
    #     if self.x_goal in self.E and x_nearest in self.E[self.x_goal]:
    #         # tree is already connected to goal using nearest vertex
    #         return True
    #     if self.X.collision_free(x_nearest, self.x_goal, self.r):  # check if obstacle-free
    #         return True
    #     return False

class RRTPlanner(Planner):
    """This planner is largely a RRT planner, though with extra features."""

    def __init__(self, **kwargs):
        self.args = MagicDict(kwargs)
        self.poses = np.empty((self.args.max_number_nodes*2 + 50,
                               kwargs['num_dim']))  # +50 to prevent over flow
        self.c_max = float('inf')
        # this dict is to temparily store distance of a new node to all others
        # so that in the next run other functions (eg choose_least_cost_parent and rwire)
        # and take advantage to the already computed values
        self._new_node_dist_to_all_others = {}
        self.nodes = []
        self.args.env = None  # will be set by env itself
        self.tree = Tree(kwargs['num_dim'])
        self.found_solution = True

        if self.args['skip_optimality']:
            # respect option of skip planning for optimality
            # (i.e. reduce RRT* to RRT with the benefit of increased performance)
            def no_opt_choose_least_cost_parent(newnode, nn=None, **_):
                if nn is None:
                    raise RuntimeError("Not enough information")
                newnode.parent = nn
                newnode.cost = nn.cost + self.args.env.dist(nn.pos, newnode.pos)
                return newnode, nn

            def no_opt_rewire(*_, **__):
                pass

            self.choose_least_cost_parent = no_opt_choose_least_cost_parent
            self.rewire = no_opt_rewire


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
        if not self.args.env.cc.visible(nn.pos, newpos):
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
                # newnode, nn, nodes=self.nodes, skip_optimality=True)
            self.add_newnode(newnode)
            # rewire to see what the newly added node can do for us
            self.rewire(newnode, self.nodes)
            # self.rewire(newnode, self.nodes, skip_optimality=True)

            # newnode.parent = nn
            # newnode.cost = nn.cost + self.args.env.dist(nn.pos, newnode.pos)

            if self.args.env.cc.visible(newnode.pos, self.goalPt.pos):
                if self.args.env.dist(newnode.pos, self.goalPt.pos) < self.args.goal_radius:
                    if newnode.cost < self.c_max:
                        # print('finished at ', self.args.env.stats.valid_sample)
                        self.c_max = newnode.cost
                        self.goalPt.parent = newnode
                        newnode.children.append(self.goalPt.parent)
                        self.draw_solution_path()

    def add_newnode(self, node):
        self.tree.add_vertex(node, pos=node.pos)

        self.poses[len(self.nodes)] = node.pos
        self.nodes.append(node)

    def choose_least_cost_parent(self, newnode, nn=None, nodes=None,
                                 skip_optimality=False, use_rtree=True, poses=None):
        """Given a new node, a node from root, return a node from root that
        has the least cost (toward the newly added node)"""
        skip_optimality = False
        use_rtree = False
        
        if skip_optimality:
            if nn is None:
                raise RuntimeError("Not enough information")

            newnode.parent = nn
            newnode.cost = nn.cost + self.args.env.dist(nn.pos, newnode.pos)
            return newnode, nn
        ########################################################

        if use_rtree or poses is not None:
            nn2 = None
            if use_rtree:
                canidates = list(self.tree.nearby(newnode.pos, n=20))
            else:
                distances = np.linalg.norm(poses - newnode.pos, axis=1)
                canidates = [nodes[idx] for idx in np.argsort(distances)[:20]]

            canidates.sort(key=operator.attrgetter('cost'))

            for n in canidates:
                if (self.args.env.dist(newnode.pos, n.pos) <= self.args.radius and
                        self.args.env.cc.visible(newnode.pos, n.pos)):
                    nn2 = n
                    break
            if nn2 is None:
                if nn is None:
                    raise RuntimeError("Unable to find nn that is connectable")
                nn2 = nn
            newnode.cost = nn2.cost + self.args.env.dist(nn2.pos, newnode.pos)
            newnode.parent = nn2
            nn2.children.append(newnode)

            return newnode, nn2



        if nn is not None:
            _newnode_to_nn_cost = self.args.env.dist(newnode.pos, nn.pos)
        self._new_node_dist_to_all_others = {}
        for p in nodes:
            _newnode_to_p_cost = self.args.env.dist(newnode.pos, p.pos)
            self._new_node_dist_to_all_others[(newnode,
                                               p)] = _newnode_to_p_cost
            if _newnode_to_p_cost <= self.args.radius and self.args.env.cc.visible(newnode.pos, p.pos):
                # This is another valid parent. Check if it's better than our current one.
                if nn is None or (p.cost + _newnode_to_p_cost <
                                  nn.cost + _newnode_to_nn_cost):
                    nn = p
                    _newnode_to_nn_cost = _newnode_to_p_cost
        if nn is None:
            raise LookupError(
                "ERROR: Provided nn=None, and cannot find any valid nn by this function. This newnode is not close to the root tree...?"
            )
        newnode.cost = nn.cost + self.args.env.dist(nn.pos, newnode.pos)
        assert newnode is not nn
        newnode.parent = nn
        nn.children.append(newnode)

        return newnode, nn


    def rewire(self, newnode, nodes, already_rewired=None, skip_optimality=False,
               use_rtree=True, poses=None):
        """Reconsider parents of nodes that had change, so that the optimiality would change instantly"""
        skip_optimality = False
        use_rtree = False
        if skip_optimality:
            return
        if len(nodes) < 1:
            return

        if use_rtree or poses is not None:
            if already_rewired is None:
                already_rewired = {newnode}
            if use_rtree:
                canidates = list(self.tree.nearby(newnode.pos, n=20))
            else:
                distances = np.linalg.norm(poses - newnode.pos, axis=1)
                canidates = [nodes[idx] for idx in np.argsort(distances)[:20]]

            canidates.sort(key=operator.attrgetter('cost'))


            for n in canidates:
                if n in already_rewired:
                    continue

                _newnode_to_n_cost = self.args.env.dist(newnode.pos, n.pos)
                if (n != newnode.parent
                        and _newnode_to_n_cost <= self.args.radius
                        and self.args.env.cc.visible(n.pos, newnode.pos)
                        and newnode.cost + _newnode_to_n_cost < n.cost):
                    # draw over the old wire
                    # reconsider = (n.parent, *n.children)
                    reconsider = n.children
                    n.parent.children.remove(n)
                    n.parent = newnode
                    newnode.children.append(n)
                    n.cost = newnode.cost + _newnode_to_n_cost
                    already_rewired.add(n)
                    self.rewire(n, reconsider, already_rewired=already_rewired)
            return

            #     if n != newnode.parent and self.args.env.dist(newnode.pos, n.pos) <= self.args[
            #         'radius'] and self.args.env.cc.visible(newnode.pos, n.pos):
            #         nn2 = n
            #         break
            # if nn2 is None:
            #     nn2 = nn
            # return nn2


        if already_rewired is None:
            already_rewired = {newnode}
        for n in (x for x in nodes if x not in already_rewired):
            if len(already_rewired) <= 1:
                _newnode_to_n_cost = self._new_node_dist_to_all_others[newnode,
                                                                       n]
            else:
                _newnode_to_n_cost = self.args.env.dist(newnode.pos, n.pos)
            if (n != newnode.parent
                    and _newnode_to_n_cost <= self.args.radius
                    and self.args.env.cc.visible(n.pos, newnode.pos)
                    and newnode.cost + _newnode_to_n_cost < n.cost):
                # draw over the old wire
                reconsider = (n.parent, *n.children)
                n.parent.children.remove(n)
                n.parent = newnode
                newnode.children.append(n)
                n.cost = newnode.cost + _newnode_to_n_cost
                already_rewired.add(n)
                self.rewire(n, reconsider, already_rewired=already_rewired)

    @staticmethod
    def find_nearest_neighbour_idx(pos, poses):
        # Make use of numpy fast parallel operation to find all distance with one operation.
        distances = np.linalg.norm(poses - pos, axis=1)
        return np.argmin(distances)
