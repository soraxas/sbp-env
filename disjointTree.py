from checkCollision import *

JOIN_TREES_RADIUS = 10

class Node:
    def __init__(self, pos):
        self.pos = np.array(pos)
        self.cost = 0
        self.edges = []

class TreeRoot:
    def __init__(self):
        self.particle_handler = None
        self.nodes = []

class TreeDisjoint(TreeRoot):
    def __init__(self):
        super().__init__()


class TreesManager:
    def __init__(self):
        self.root = TreeRoot()
        self.disjointTrees = []
        pass

    def findNearestNode(self, node, parent_tree):
        """
        Find nearest node from given node
        """
        nn = parent_tree.nodes[0]
        for p in parent_tree.nodes:
            if dist(p.pos, node.pos) < dist(nn.pos, node.pos):
                nn = p
        return nn

    def findNearestNodeFromNeighbour(self, node, parent_tree, radius):
        """
        Given a tree, a node within that tree, and radius
        Return the cloest node (and its corresponding tree) within radius that is not from the given tree (from other neighbourhood trees)
        Return None if none exists
        """
        # TODO make it more general to return a list of nodes within radius..?
        cloest_node = cloest_tree = cloest_dist = None
        for tree in [*self.disjointTrees, self.root]:
            if tree is parent_tree:
                # skip self
                continue
            for n in tree.nodes:
                _dist = dist(node, n)
                if _dist < radius:
                    if cloest_dist is None:
                        cloest_dist = _dist
                    if _dist <= cloest_dist:
                        cloest_node = n
                        cloest_tree = tree
        return cloest_node, cloest_tree


    def joinTreeToRoot(self, tree, kill_particle=True):
        """It will join the given tree to the root"""
        # remove tree2 from disjoint tree list
        raise Exception("NOT implemented yet")

    def joinTrees(self, tree1, tree2, kill_particle=True):
        """
        Join the two given tree together (along with their nodes).
        It will delete the particle reference from the second tree.
        It will use RRT* method to add all nodes if one of the tree is the ROOT.
        """
        # TODO make it more general to return a list of nodes within radius..?
        if tree1 is self.root:
            self.joinTreeToRoot(tree1)
            return
        if tree2 is self.root:
            self.joinTreeToRoot(tree2)
            return
        tree1.nodes.extend(tree2.nodes)
        if kill_particle:
            tree2.particle_handler = None
        # remove tree2 from disjoint tree list
        self.disjointTrees.remove(tree2)



##########################################################################################
##########################################################################################
##########################################################################################
##########################################################################################

    def chooseLeastCostParent(self, nn, newnode):
        for p in self.nodes:
            if(self.cc.pathIsFree(p, newnode) and
               dist(p.pos, newnode.pos) < self.RADIUS and
               p.cost + dist(p.pos, newnode.pos) < nn.cost + dist(nn.pos, newnode.pos)):
                nn = p
        newnode.cost = nn.cost + dist(nn.pos, newnode.pos)
        newnode.parent = nn
        return newnode, nn

    def reWire(self, newnode):
        for i in range(len(self.nodes)):
            p = self.nodes[i]
            if(p != newnode.parent and self.cc.pathIsFree(p, newnode) and
               dist(p.pos, newnode.pos) < self.RADIUS and newnode.cost + dist(p.pos, newnode.pos) < p.cost):
                # draw over the old wire
                pygame.draw.line(self.path_layers, Colour.white, p.pos*self.SCALING, p.parent.pos*self.SCALING, self.SCALING)
                # update new parents (re-wire)
                p.parent = newnode
                p.cost = newnode.cost + dist(p.pos, newnode.pos)
                pygame.draw.line(self.path_layers, Colour.black, p.pos*self.SCALING, newnode.pos*self.SCALING, self.SCALING)
