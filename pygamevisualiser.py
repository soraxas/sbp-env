import math
import numpy as np
import logging
import sys

import pygame
from pygame.locals import *
from helpers import Colour

LOGGER = logging.getLogger(__name__)


class VisualiserSwitcher():
    def choose_planner_vis(clname):
         VisualiserSwitcher.planner_clname = clname

    def choose_env_vis(clname):
         VisualiserSwitcher.env_clname = clname


class BasePlannerVisualiser:
    def __init__(self, *args, **kwargs):
        pass

    def init(self, *args, **kwargs):
        pass

    def paint(self, *args, **kwargs):
        pass

    def terminates_hook(self):
        pass

    def draw_solution_path(self):
        pass

class BaseEnvVisualiser:
    def __init__(self, *args, **kwargs):
        pass

    def visualiser_init(self, *args, **kwargs):
        pass

    def update_screen(self, *args, **kwargs):
        pass

    def set_start_goal_points(self, *args, **kwargs):
        if kwargs['start'] is None or kwargs['goal'] is None:
            raise Exception("start/goal is not set yet")
        return kwargs['start'], kwargs['goal']

class PygamePlannerVisualiser:

    def init(self, *args, **kwargs):
        if self.__class__.__name__ == 'RRdTSampler':
            self.particles_layer = pygame.Surface(
                (self.args.env.dim[0] * self.args.scaling,
                 self.args.env.dim[1] * self.args.scaling), pygame.SRCALPHA)
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
            for tree in self.disjointedTrees:
                bfs = BFS(tree.nodes[0], validNodes=tree.nodes)
                while bfs.has_next():
                    newnode = bfs.next()
                    for e in newnode.edges:
                        new_set = frozenset({newnode, e})
                        if new_set not in drawn_nodes_pairs:
                            drawn_nodes_pairs.add(new_set)
                            self.args.env.draw_path(newnode, e)
            # Draw root tree
            for n in self.root.nodes:
                if n.parent is not None:
                    new_set = frozenset({n, n.parent})
                    if new_set not in drawn_nodes_pairs:
                        drawn_nodes_pairs.add(new_set)
                        self.args.env.draw_path(n, n.parent, Colour.orange)
            self.draw_solution_path()

        def RRFPlanner_paint():
            self.args.env.path_layers.fill(Colour.ALPHA_CK)
            from planners.rrdtPlanner import BFS
            drawn_nodes_pairs = set()
            # Draw disjointed trees
            for tree in self.disjointedTrees:
                bfs = BFS(tree.nodes[0], validNodes=tree.nodes)
                while bfs.has_next():
                    newnode = bfs.next()
                    for e in newnode.edges:
                        new_set = frozenset({newnode, e})
                        if new_set not in drawn_nodes_pairs:
                            drawn_nodes_pairs.add(new_set)
                            self.args.env.draw_path(newnode, e)
            # Draw root tree
            for nodes, colour in zip((self.root.nodes, self.goal_root.nodes), (Colour.orange, Colour.red)):
                # for n in self.nodes:
                for n in nodes:
                    if n.parent is not None:
                        new_set = frozenset({n, n.parent})
                        if new_set not in drawn_nodes_pairs:
                            drawn_nodes_pairs.add(new_set)
                            self.args.env.draw_path(n, n.parent, colour)
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
            'RRFSampler': RRdTSampler_paint,
            'RRFPlanner': RRFPlanner_paint,
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
        if not self.found_solution or self.c_max == float('inf'):
            # nothing to d
            return
        # redraw new path
        self.args.env.solution_path_screen.fill(Colour.ALPHA_CK)
        nn = self.goalPt.parent
        self.c_max = nn.cost
        while not nn.is_start:
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


class PygameEnvVisualiser:

    def __init__(self, *args, **kwargs):
        self.extra = 25
        self.img = pygame.image.load(self.args.image)
        self.dim = kwargs['image_shape']

    def visualiser_init(self, no_display=False):
        self.args.no_display = no_display
        if self.args.no_display:
            return
        pygame.init()
        self.fpsClock = pygame.time.Clock()
        # self.fpsClock.tick(10)
        self.fpsClock.tick(10000)
        pygame.display.set_caption('RRTstar')
        # screen.fill(white)
        ################################################################################
        # text
        pygame.font.init()
        self.myfont = pygame.font.SysFont('Arial',
                                          int(self.dim[0] * 0.04 * self.args.scaling))
        ################################################################################
        # main window
        self.window = pygame.display.set_mode([
            int(self.dim[0] * self.args.scaling),
            int((self.dim[1] + self.extra) * self.args.scaling)
        ])
        ################################################################################
        # background aka the room
        self.background = pygame.Surface([self.dim[0], (self.dim[1] + self.extra)])
        self.background.blit(self.img, (0, 0))
        # resize background to match windows
        self.background = pygame.transform.scale(self.background, [
            int(self.dim[0] * self.args.scaling),
            int((self.dim[1] + self.extra) * self.args.scaling)
        ])
        ################################################################################
        # path of RRT*
        self.path_layers = pygame.Surface([
            self.dim[0] * self.args.scaling, (self.dim[1] + self.extra) * self.args.scaling
        ])
        self.path_layers.fill(Colour.ALPHA_CK)
        self.path_layers.set_colorkey(Colour.ALPHA_CK)
        ################################################################################
        # layers to store the solution path
        self.solution_path_screen = pygame.Surface([
            self.dim[0] * self.args.scaling, (self.dim[1] + self.extra) * self.args.scaling
        ])
        self.solution_path_screen.fill(Colour.ALPHA_CK)
        self.solution_path_screen.set_colorkey(Colour.ALPHA_CK)
        ################################################################################
        # layers to store the sampled points
        self.sampledPoint_screen = pygame.Surface([
            self.dim[0] * self.args.scaling, (self.dim[1] + self.extra) * self.args.scaling
        ])
        self.sampledPoint_screen.fill(Colour.ALPHA_CK)
        self.sampledPoint_screen.set_colorkey(Colour.ALPHA_CK)
        ################################################################################

    def set_start_goal_points(self, start, goal):
        """A function that query user for start/goal and set them accordingly."""
        from helpers import Node
        ##################################################
        # Get starting and ending point
        LOGGER.info('Select Starting Point and then Goal Point')
        if start is not None and goal is not None:
            return start, goal
        self.startPt = self.goalPt = None
        self.update_screen(update_all=True)
        while start is None or goal is None:
            mousePos = None
            for e in pygame.event.get():
                if e.type == MOUSEBUTTONDOWN:
                    mousePos = np.array(e.pos) / self.args.scaling
                    from collisionChecker import RobotArm4dCollisionChecker
                    if type(self.cc) == RobotArm4dCollisionChecker:
                        mousePos = np.array([*mousePos, *np.random.uniform(-np.pi, np.pi, 2)])
                    if not self.cc.feasible(mousePos):
                        # failed to pass collision check
                        mousePos = None
                    elif e.type == QUIT or (e.type == KEYUP
                                            and e.key == K_ESCAPE):
                        LOGGER.info("Leaving.")
                        return
            # convert mouse pos to Node
            if mousePos is not None:
                if start is None:
                    start = Node(mousePos)
                    self.startPt = start
                    LOGGER.info(f'starting point set: {mousePos}')
                elif goal is None:
                    goal = Node(mousePos)
                    self.goalPt = goal
                    LOGGER.info(f'goal point set: {mousePos}')
            self.update_screen(update_all=True)
        return start, goal

    def process_pygame_event(self):
        for e in pygame.event.get():
            if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                LOGGER.info("Leaving.")
                sys.exit(0)

    def pygame_show(self):
        self.args.no_display = False

    def pygame_hide(self):
        self.args.no_display = True
        pygame.display.iconify()
        # pygame.quit()

    def draw_stick_robot(self,
                  node,
                  colour1=Colour.cAlpha(Colour.orange, 128),
                  colour2=Colour.cAlpha(Colour.cyan, 128),
                  line_modifier=2.5,
                  layer=None):

        # draw config for node 1
        pt1 = node.pos[:2]
        pt2 = self.cc._get_pt_from_angle_and_length(pt1, node.pos[2], self.cc.stick_robot_length_config[0])
        pt3 = self.cc._get_pt_from_angle_and_length(pt2, node.pos[3], self.cc.stick_robot_length_config[1])

        pt2 = np.array(pt2)
        pt3 = np.array(pt3)

        pygame.draw.line(layer, colour1,
                         pt1 * self.args.scaling,
                         pt2 * self.args.scaling,
                         int(line_modifier * self.args.scaling))
        pygame.draw.line(layer, colour2,
                         pt2 * self.args.scaling,
                         pt3 * self.args.scaling,
                         int(line_modifier * self.args.scaling))

        # pygame.draw.circle(layer, Colour.green, (pt1 * self.args.scaling).astype(int) , int(2 * self.args.scaling))


    def draw_path(self,
                  node1,
                  node2,
                  colour=Colour.path_blue,
                  line_modifier=1,
                  layer=None):
        if layer is None:
            layer = self.path_layers
        if node1 is not None and node2 is not None:
            pygame.draw.line(layer, colour, node1.pos[:2] * self.args.scaling,
                             node2.pos[:2] * self.args.scaling,
                             int(line_modifier * self.args.scaling))

        # nodes_to_draw = self.nodes
        # # cap the number of robot to draw
        # max_draw = 30
        # if len(nodes_to_draw) > max_draw:
        #     nodes_to_draw = random.sample(nodes_to_draw, max_draw)
        #
        # for n in nodes_to_draw:
        #     self.args.env.draw_stick_robot(n, layer=self.args.env.path_layers)

        import collisionChecker
        if type(self.cc) == collisionChecker.RobotArm4dCollisionChecker:
            self.draw_stick_robot(node1, layer=self.path_layers)
        else:
            pygame.draw.line(layer, colour, node1.pos * self.args.scaling,
                            node2.pos * self.args.scaling,
                            int(line_modifier * self.args.scaling))

    def draw_circle(self, pos, colour, radius, layer):
        draw_pos = int(pos[0] * self.args.scaling), int(pos[1] * self.args.scaling)
        pygame.draw.circle(layer, colour, draw_pos, int(radius * self.args.scaling))

    def update_screen(self, update_all=False):
        self.process_pygame_event()
        if 'refresh_cnt' not in self.__dict__:
            # INIT (this section will only run when this function is first called)
            self.refresh_cnt = 0

        if update_all or self.args.always_refresh:
            count = 0  #FORCE UPDATE
        else:
            count = self.refresh_cnt
            self.refresh_cnt += 1

###################################################################################
        def draw_start_goal_pt():
            if self.startPt is not None:
                self.draw_circle(
                    pos=self.startPt.pos,
                    colour=Colour.red,
                    radius=self.args.goal_radius,
                    layer=self.path_layers)
            if self.goalPt is not None:
                self.draw_circle(
                    pos=self.goalPt.pos,
                    colour=Colour.green,
                    radius=self.args.goal_radius,
                    layer=self.path_layers)

        # limites the screen update
        if count % 20 == 0:
            self.window.blit(self.background, (0, 0))

        if count % 60 == 0:
            try:
                self.planner.paint(window=self.window)
            except AttributeError as e:
                # print(e)
                pass
            draw_start_goal_pt()

        ##### Tree paths
        if count % 20 == 0:
            self.window.blit(self.path_layers, (0, 0))
            self.window.blit(self.solution_path_screen, (0, 0))
            draw_start_goal_pt()

        ##### Sampler hook
        if count % 20 == 0:
            try:
                self.args.sampler.paint(window=self.window)
            except AttributeError as e:
                # print(e)
                pass

        ##### Sampled points
        if count % 4 == 0:
            self.sampledPoint_screen.fill(Colour.ALPHA_CK)
            # Draw sampled nodes
            for sampledPos in self.stats.sampledNodes:
                self.draw_circle(
                    pos=sampledPos,
                    colour=Colour.red,
                    radius=2,
                    layer=self.sampledPoint_screen)
            self.window.blit(self.sampledPoint_screen, (0, 0))
            # remove them from list
            del self.stats.sampledNodes[:]

        ##### Texts
        if count % 10 == 0:
            _cost = 'INF' if self.planner.c_max == float('inf') else round(
                self.planner.c_max, 2)
            if 'RRdTSampler' in self.args.sampler.__str__() and count > 0:
                num_nodes = sum(
                    len(tree.nodes) for tree in (
                        *self.planner.disjointedTrees,
                        self.planner.root))
            else:
                num_nodes = len(self.planner.nodes)
            # text = 'Cost_min: {}  | Nodes: {}'.format(_cost, num_nodes)
            # self.window.blit(self.myfont.render(text, False, Colour.black, Colour.white), (20,self.dim[1] * self.args.scaling * 0.88))
            text = 'Cost: {} | Inv.Samples: {}(con) {}(obs)'.format(
                _cost, self.stats.invalid_samples_connections,
                self.stats.invalid_samples_obstacles)
            self.window.blit(
                self.myfont.render(text, False, Colour.white, Colour.black),
                (10, (self.dim[1] + self.extra) * self.args.scaling * 0.95))

        pygame.display.update()

class KlamptPlannerVisualiser:

    def init(self, *args, **kwargs):
        return
        if self.__class__.__name__ == 'RRdTSampler':
            self.particles_layer = pygame.Surface(
                (self.args.env.dim[0] * self.args.scaling,
                 self.args.env.dim[1] * self.args.scaling), pygame.SRCALPHA)

    def paint(self, *args, **kwargs):

        def get_world_pos(config):
            self.args.env.cc.robot.setConfig(
                self.args.env.cc._translate_to_klampt(config))
            link = self.args.env.cc.robot.link(11)
            pos = link.getWorldPosition([0, 0, 0])
            return pos

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
            # return
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
                # get a transition from green to red
                c = get_color_transists(self._last_prob[i], max_num, min_num)
                c = max(min(255, c), 50)
                color = (c, c, 0, 1)
                self.args.env.draw_node(get_world_pos(p.pos), colour=color, size=25,
                                        label=id(p))

        def _helper_draw_nodes_edges(nodes, colour):
            drawn_nodes_pairs = set()
            for n in nodes:
                self.args.env.draw_node(get_world_pos(n.pos), colour=colour)
                if n.parent is not None:
                    new_set = frozenset({n, n.parent})
                    if new_set not in drawn_nodes_pairs:
                        drawn_nodes_pairs.add(new_set)
                        self.args.env.draw_path(get_world_pos(n.pos),
                                                get_world_pos(n.parent.pos),
                                                colour=colour)
            if self.goalPt.parent is not None:
                self.draw_solution_path()

        def RRTPlanner_paint():
            # self.args.env.path_layers.fill(Colour.ALPHA_CK)
            _helper_draw_nodes_edges(nodes=self.nodes, colour=(1,0,0,1))

        def BiRRTPlanner_paint():
            drawn_nodes_pairs = set()
            for c, nodes in (((1,0,0,1), self.nodes),
                             ((0,0,1,1),self.goal_tree_nodes)):
                _helper_draw_nodes_edges(nodes=nodes, colour=c)


        def PRMPlanner_paint():
            edges = list()
            colour = (1,0,0,1)
            for n in self.nodes:
                self.args.env.draw_node(get_world_pos(n.pos), colour=colour)
            for edge in self.graph.edges:
                edge = np.array(edge).transpose()
                self.args.env.draw_path(get_world_pos(n.pos),
                                        get_world_pos(n.parent.pos),
                                        colour=colour)

        def RRdTPlanner_paint():
            from planners.rrdtPlanner import BFS
            drawn_nodes_pairs = set()

            def generate_random_colors():
                """Return a generator that generate distinct colour."""
                import colorsys
                import ghalton
                perms = ghalton.EA_PERMS[:1]
                sequencer = ghalton.GeneralizedHalton(perms)
                while True:
                    x = sequencer.get(1)[0][0]
                    HSV_tuple = (x, 0.5, 0.5)
                    HSV_tuple = (x, 1, 0.6)
                    rgb_colour = colorsys.hsv_to_rgb(*HSV_tuple)
                    yield (*rgb_colour, 1.0)  # add alpha channel

            color_gen = generate_random_colors()


            # Draw disjointed trees
            for tree in self.disjointedTrees:
                c = next(color_gen)
                # draw nodes
                for node in tree.nodes:
                    self.args.env.draw_node(get_world_pos(node.pos), colour=c)
                # draw edges
                bfs = BFS(tree.nodes[0], validNodes=tree.nodes)
                while bfs.has_next():
                    newnode = bfs.next()
                    for e in newnode.edges:
                        new_set = frozenset({newnode, e})
                        if new_set not in drawn_nodes_pairs:
                            drawn_nodes_pairs.add(new_set)


                            self.args.env.draw_path(get_world_pos(newnode.pos),
                                                    get_world_pos(e.pos),
                                                    colour=c)
            # Draw root tree
            c = next(color_gen)
            # override to red
            c = (1, 0, 0, 1)
            # draw nodes
            for node in self.root.nodes:
                self.args.env.draw_node(get_world_pos(node.pos), colour=c)
            # draw edges
            for n in self.root.nodes:
                if n.parent is not None:
                    new_set = frozenset({n, n.parent})
                    if new_set not in drawn_nodes_pairs:
                        drawn_nodes_pairs.add(new_set)
                        self.args.env.draw_path(get_world_pos(n.pos),
                                                    get_world_pos(n.parent.pos)
                                                , colour=c)

        def RRFPlanner_paint():
            from planners.rrdtPlanner import BFS
            drawn_nodes_pairs = set()

            def generate_random_colors():
                """Return a generator that generate distinct colour."""
                import colorsys
                import ghalton
                perms = ghalton.EA_PERMS[:1]
                sequencer = ghalton.GeneralizedHalton(perms)
                while True:
                    x = sequencer.get(1)[0][0]
                    HSV_tuple = (x, 0.5, 0.5)
                    HSV_tuple = (x, 1, 0.6)
                    rgb_colour = colorsys.hsv_to_rgb(*HSV_tuple)
                    yield (*rgb_colour, 1.0)  # add alpha channel

            color_gen = generate_random_colors()


            # Draw disjointed trees
            for tree in self.disjointedTrees:
                c = next(color_gen)
                # draw nodes
                for node in tree.nodes:
                    self.args.env.draw_node(get_world_pos(node.pos), colour=c)
                # draw edges
                bfs = BFS(tree.nodes[0], validNodes=tree.nodes)
                while bfs.has_next():
                    newnode = bfs.next()
                    for e in newnode.edges:
                        new_set = frozenset({newnode, e})
                        if new_set not in drawn_nodes_pairs:
                            drawn_nodes_pairs.add(new_set)


                            self.args.env.draw_path(get_world_pos(newnode.pos),
                                                    get_world_pos(e.pos),
                                                    colour=c)
            # Draw root tree
            c = next(color_gen)
            # override to red
            c = (1, 0, 0, 1)
            # draw nodes
            for root, c in zip((self.root, self.goal_root), ((1, 0, 0, 1), (0, 0, 1, 1))):
                for node in root.nodes:
                    self.args.env.draw_node(get_world_pos(node.pos), colour=c)
                # draw edges
                for n in root.nodes:
                    if n.parent is not None:
                        new_set = frozenset({n, n.parent})
                        if new_set not in drawn_nodes_pairs:
                            drawn_nodes_pairs.add(new_set)
                            self.args.env.draw_path(get_world_pos(n.pos),
                                                        get_world_pos(n.parent.pos)
                                                    , colour=c)

            # self.draw_solution_path()

        def default_paint():
            raise NotImplementedError(f"{self.__class__.__name__} visualising "
                                      "is not not implemented!")

        implemented_classes = {
            'RandomPolicySampler': nothing_paint,
            'LikelihoodPolicySampler': LikelihoodPolicySampler_paint,
            'InformedRRTSampler': InformedRRTSampler_paint,
            'RRdTSampler': RRdTSampler_paint,
            'RRdTPlanner': RRdTPlanner_paint,
            'RRFSampler': RRdTSampler_paint,
            'RRFPlanner': RRFPlanner_paint,
            'PRMSampler': nothing_paint,
            'PRMPlanner': PRMPlanner_paint,
            'RRTSampler': nothing_paint,
            'RRTPlanner': RRTPlanner_paint,
            'BiRRTSampler': nothing_paint,
            'BiRRTPlanner': BiRRTPlanner_paint,
        }

        implemented_classes.get(self.__class__.__name__, default_paint)()

    def draw_solution_path(self):
        return
        if self.c_max == float('inf'):
            # nothing to d
            return
        # redraw new path
        self.args.env.solution_path_screen.fill(Colour.ALPHA_CK)
        nn = self.goalPt.parent
        self.c_max = nn.cost
        while not nn.is_start:
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

class KlamptEnvVisualiser:

    def __init__(self, *args, **kwargs):
        self.drawn_label = set()
        self.kwargs = kwargs

    def visualiser_init(self, no_display=False):
        if not no_display:
            from klampt import vis

            vis.add("world", self.kwargs['cc'].world)
            vis.show()
        return

    def set_start_goal_points(self, start, goal):
        if start:
            start = self.cc._translate_to_klampt(start)
        if goal:
            goal = self.cc._translate_to_klampt(goal)
        from klampt.io import resource

        def user_set_config(q, type_str="<..>"):
            save = None
            # it's worthwhile to make sure that it's feasible
            while q is None or not self.cc.space.feasible(q):
                print(type_str + " configuration isn't feasible")
                save, q = resource.edit(type_str + " config", q, "Config", world=self.cc.world)
            return save, q
        # start = [0.0, -1.5800000000000005, 1.2, -0.37, -1.57, -1.57, -1.57, 0.0, 0.048, 0.048, -0.048, 0.048]
        # goal = [0.0, -1.5800000000000005, 1.2, -0.37, -1.57, -1.57, -1.57, 0.0, 0.048, 0.048, -0.048, 0.048]
        _, start = user_set_config(start, "Start")
        _, goal = user_set_config(goal, "Goal")


        return tuple(map(self.cc._translate_from_klampt, (start, goal)))

    def draw_node(self, pos, colour=(1, 0, 0, 1), size=15, label=None):
        from klampt import vis
        from klampt.model.coordinates import Point, Direction
        if label is None:
            label = repr(pos)
        vis.add(label, Point(pos), keepAppearance=True)
        vis.setAttribute(label, "size", size)
        vis.setAttribute(label, "color", colour)
        vis.hideLabel(label)

    def draw_path(self, pos1, pos2, colour=None):
        from klampt import vis
        from klampt.model.coordinates import Point, Direction
        from klampt.model.trajectory import Trajectory

        unique_label = repr((pos1, pos2))

        label = repr((pos1, pos2))
        if label not in self.drawn_label:
            vis.add(label, Trajectory(milestones=[pos1, pos2]), keepAppearance=True)
            if colour is not None:
                vis.setAttribute(label, "color", colour)
            vis.hideLabel(label)
            # self.drawn_label.add(label)



    def draw_circle(self, pos, colour, radius, layer):
        draw_pos = int(pos[0] * self.args.scaling), int(pos[1] * self.args.scaling)
        pygame.draw.circle(layer, colour, draw_pos, int(radius * self.args.scaling))

    def update_screen(self, update_all=False):
        if 'refresh_cnt' not in self.__dict__:
            # INIT (this section will only run when this function is first called)
            self.refresh_cnt = 0

        if update_all or self.args.always_refresh:
            count = 0  #FORCE UPDATE
        else:
            count = self.refresh_cnt
            self.refresh_cnt += 1

###################################################################################
        def draw_start_goal_pt():
            if self.startPt is not None:
                self.draw_circle(
                    pos=self.startPt.pos,
                    colour=Colour.red,
                    radius=self.args.goal_radius,
                    layer=self.path_layers)
            if self.goalPt is not None:
                self.draw_circle(
                    pos=self.goalPt.pos,
                    colour=Colour.green,
                    radius=self.args.goal_radius,
                    layer=self.path_layers)

        if count % 60 == 0:
            try:
                self.planner.paint()
            except AttributeError as e:
                # print(e)
                pass

        ##### Sampler hook
        if count % 20 == 0:
            try:
                self.args.sampler.paint()
            except AttributeError as e:
                # print(e)
                pass

        # ##### Sampled points
        # if count % 4 == 0:
        #     self.sampledPoint_screen.fill(Colour.ALPHA_CK)
        #     # Draw sampled nodes
        #     for sampledPos in self.stats.sampledNodes:
        #         self.draw_circle(
        #             pos=sampledPos,
        #             colour=Colour.red,
        #             radius=2,
        #             layer=self.sampledPoint_screen)
        #     self.window.blit(self.sampledPoint_screen, (0, 0))
        #     # remove them from list
        #     del self.stats.sampledNodes[:]
