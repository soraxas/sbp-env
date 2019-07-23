import math
import pygame
import numpy as np

from helpers import Colour


class Visualiser:
    def init(self, *args, **kwargs):
        if self.__class__.__name__ == 'RRdTSampler':
            self.particles_layer = pygame.Surface(
                (self.args.XDIM * self.args.scaling,
                 self.args.YDIM * self.args.scaling), pygame.SRCALPHA)
        elif self.__class__.__name__ == 'LikelihoodPolicySampler':
            # probability layer
            self.prob_layer = pygame.Surface(
                (self.PROB_BLOCK_SIZE * self.args.scaling,
                 self.PROB_BLOCK_SIZE * self.args.scaling), pygame.SRCALPHA)

    def paint(self, *args, **kwargs):
        def nothing_paint():
            pass

        def LikelihoodPolicySampler_paint():
            def get_vector_alpha_parameters(vector):
                max_prob = vector.max()
                min_prob = vector.min()
                denominator = max_prob - min_prob
                if denominator == 0:
                    denominator = 1  # prevent division by zero
                return max_prob, min_prob, denominator

            if self.prob_vector_normalized is not None:
                for i in range(self.prob_vector_normalized.shape[0]):
                    for j in range(self.prob_vector_normalized.shape[1]):
                        max_prob, min_prob, denominator = get_vector_alpha_parameters(
                            self.prob_vector_normalized)
                        alpha = 240 * (1 - (self.prob_vector_normalized[i][j] -
                                            min_prob) / denominator)
                        self.prob_layer.fill((255, 128, 255, alpha))
                        # print(self.prob_vector_normalized[i][j])
                        kwargs['window'].blit(
                            self.prob_layer,
                            (i * self.PROB_BLOCK_SIZE * self.args.scaling,
                             j * self.PROB_BLOCK_SIZE * self.args.scaling))

        def InformedRRTSampler_paint():
            # draw the ellipse
            if self.args.sampler.cBest < float('inf'):
                cBest = self.cBest * self.args.scaling
                cMin = self.cMin * self.args.scaling
                a = math.sqrt(cBest**2 - cMin**2)  # height
                b = cBest  # width
                # rectangle that represent the ellipse
                r = pygame.Rect(0, 0, b, a)
                angle = self.etheta
                # rotate via surface
                ellipse_surface = pygame.Surface((b, a), pygame.SRCALPHA,
                                                 32).convert_alpha()
                try:
                    pygame.draw.ellipse(ellipse_surface, (255, 0, 0, 80), r)
                    pygame.draw.ellipse(ellipse_surface, Colour.black, r,
                                        int(2 * self.args.scaling))
                except ValueError:
                    # sometime it will fail to draw due to ellipse being too narrow
                    pass
                # rotate
                ellipse_surface = pygame.transform.rotate(
                    ellipse_surface, -angle * 180 / math.pi)
                # we need to offset the blitz based on the surface ceenter
                rcx, rcy = ellipse_surface.get_rect().center
                ellipse_x = (self.xCenter[0] * self.args.scaling - rcx)
                ellipse_y = (self.xCenter[1] * self.args.scaling - rcy)
                kwargs['window'].blit(ellipse_surface, (ellipse_x, ellipse_y))

        def RRdTSampler_paint():
            def get_color_transists(value, max_prob, min_prob):
                denominator = max_prob - min_prob
                if denominator == 0:
                    denominator = 1  # prevent division by zero
                return 220 - 180 * (1 - (value - min_prob) / denominator)

            # if self._last_prob is None:
            #     return
            max_num = self._last_prob.max()
            min_num = self._last_prob.min()
            for i, p in enumerate(self.p_manager.particles):
                self.particles_layer.fill((255, 128, 255, 0))
                # get a transition from green to red
                c = get_color_transists(self._last_prob[i], max_num, min_num)
                c = max(min(255, c), 50)
                color = (c, c, 0)
                self.args.env.draw_circle(pos=p.pos,
                                          colour=color,
                                          radius=4,
                                          layer=self.particles_layer)
                kwargs['window'].blit(self.particles_layer, (0, 0))

        def RRTPlanner_paint():
            self.args.env.path_layers.fill(Colour.ALPHA_CK)
            drawn_nodes_pairs = set()
            for n in self.nodes:
                if n.parent is not None:
                    new_set = frozenset({n, n.parent})
                    if new_set not in drawn_nodes_pairs:
                        drawn_nodes_pairs.add(new_set)
                        self.args.env.draw_path(n, n.parent)
            if self.goalPt.parent is not None:
                self.draw_solution_path()

        def BiRRTPlanner_paint():
            self.args.env.path_layers.fill(Colour.ALPHA_CK)
            drawn_nodes_pairs = set()
            for nodes in (self.nodes, self.goal_tree_nodes):
                for n in nodes:
                    if n.parent is not None:
                        new_set = frozenset({n, n.parent})
                        if new_set not in drawn_nodes_pairs:
                            drawn_nodes_pairs.add(new_set)
                            self.args.env.draw_path(n, n.parent)
            if self.goalPt.parent is not None:
                self.draw_solution_path()

        def PRMPlanner_paint():
            self.args.env.path_layers.fill(Colour.ALPHA_CK)
            edges = list(self.tree.edges)
            # print(edges)
            for n in self.nodes:
                self.args.env.draw_circle(pos=n.pos,
                                          colour=(0, 0, 255),
                                          radius=1.4,
                                          layer=self.args.env.path_layers)
            for edge in edges:
                edge = np.array(edge).transpose()
                self.args.env.draw_path(edge[0], edge[1])

        def RRdTPlanner_paint():
            self.args.env.path_layers.fill(Colour.ALPHA_CK)
            from planners.rrdtPlanner import BFS
            drawn_nodes_pairs = set()
            # Draw disjointed trees
            for tree in self.args.sampler.tree_manager.disjointedTrees:
                bfs = BFS(tree.nodes[0], validNodes=tree.nodes)
                while bfs.has_next():
                    newnode = bfs.next()
                    for e in newnode.edges:
                        new_set = frozenset({newnode, e})
                        if new_set not in drawn_nodes_pairs:
                            drawn_nodes_pairs.add(new_set)
                            self.args.env.draw_path(newnode, e)
            # Draw root tree
            for n in self.args.sampler.tree_manager.root.nodes:
                if n.parent is not None:
                    new_set = frozenset({n, n.parent})
                    if new_set not in drawn_nodes_pairs:
                        drawn_nodes_pairs.add(new_set)
                        self.args.env.draw_path(n, n.parent, Colour.orange)
            self.draw_solution_path()

        def default_paint():
            raise NotImplementedError(f"{self.__class__.__name__} visualising "
                                      "is not not implemented!")

        implemented_classes = {
            'RandomPolicySampler': nothing_paint,
            'LikelihoodPolicySampler': LikelihoodPolicySampler_paint,
            'InformedRRTSampler': InformedRRTSampler_paint,
            'RRdTSampler': RRdTSampler_paint,
            'RRdTPlanner': RRdTPlanner_paint,
            'PRMSampler': nothing_paint,
            'PRMPlanner': PRMPlanner_paint,
            'RRdTPlanner': RRdTPlanner_paint,
            'RRTSampler': nothing_paint,
            'RRTPlanner': RRTPlanner_paint,
            'BiRRTSampler': nothing_paint,
            'BiRRTPlanner': BiRRTPlanner_paint,
        }

        implemented_classes.get(self.__class__.__name__, default_paint)()

    def draw_solution_path(self):
        if self.c_max == float('inf'):
            # nothing to d
            return
        # redraw new path
        self.args.env.solution_path_screen.fill(Colour.ALPHA_CK)
        nn = self.goalPt.parent
        self.c_max = nn.cost
        while nn != self.startPt:
            self.args.env.draw_path(nn,
                                    nn.parent,
                                    colour=Colour.blue,
                                    line_modifier=5,
                                    layer=self.args.env.solution_path_screen)
            nn = nn.parent
        self.args.env.window.blit(self.args.env.path_layers, (0, 0))
        self.args.env.window.blit(self.args.env.solution_path_screen, (0, 0))

    def terminates_hook(self):
        if self.__class__.__name__ == 'PRMPlanner':
            self.build_graph()
            # draw all edges
            for n1, n2 in self.tree.edges():
                self.args.env.draw_path(n1, n2, Colour.path_blue)
            self.get_solution()
            self.args.env.update_screen()
            import time
            time.sleep(30)
