from __future__ import annotations

import logging
import os
import sys
from abc import ABC
from typing import Optional
from typing import TYPE_CHECKING

import numpy as np

from utils.common import Colour

os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"  # disable pygame prompt

import pygame
from pygame.locals import *

if TYPE_CHECKING:
    from utils import planner_registry
    from env import Env
    from planners.basePlanner import Planner

LOGGER = logging.getLogger(__name__)


def nothing_to_paint(*args):
    """Paint function that does nothing.

    :param *args: unused
    """
    pass


############################################################
#              Base without visualisation                  #
############################################################


class BasePlannerVisualiser(ABC):
    """
    Abstract base Planner Visualiser
    """

    def __init__(self, **kwargs):
        pass

    def init(self, **kwargs):
        """
        Abstract method, delayed *initialisation* method for planner visualiser.
        """
        pass

    def paint(self, **kwargs):
        """
        Abstract method, paint method for planner visualiser.
        """
        pass

    def terminates_hook(self):
        """
        Abstract method, terminates hook for planner visualiser that is executed at
        the end of loop.
        """
        pass

    def draw_solution_path(self):
        """
        Abstract method, method to draw solution path for planner visualiser.
        """
        pass


class BaseSamplerVisualiser(ABC):
    """
    Abstract base Sampler Visualiser
    """

    def __init__(self, **kwargs):
        pass

    def init(self, **kwargs):
        """
        Abstract method, delayed *initialisation* method for sampler visualiser.
        """
        pass

    def paint(self, **kwargs):
        """
        Abstract method, paint method for sampler visualiser.
        """
        pass

    def terminates_hook(self):
        """
        Abstract method, terminates hook for sampler visualiser that is executed at
        the end of loop.
        """
        pass


class BaseEnvVisualiser(ABC):
    """
    Abstract base environment visualiser
    """

    def __init__(self, **kwargs):
        pass

    def visualiser_init(self, **kwargs):
        """
        Abstract method, delayed *initialisation* method for environment visualiser.
        """
        pass

    def update_screen(self, **kwargs):
        """
        Abstract method, when call this method will update the screen.
        """
        pass

    def set_start_goal_points(
        self,
        start: Optional[np.ndarray] = None,
        goal: Optional[np.ndarray] = None,
        **kwargs,
    ):
        """
        Ask the visualiser to query both start and goal configuration from the user.
        """
        if start is None or goal is None:
            raise Exception("start/goal is not set yet")
        return start, goal


############################################################
#              Pygame (2D) visualisation                   #
############################################################


class PygamePlannerVisualiser(BasePlannerVisualiser):
    """
    Planner Visualiser with the Pygame engine
    """

    def __init__(self, planner_data_pack: planner_registry.PlannerDataPack, **kwargs):
        """
        :param planner_data_pack: a planner data pack that stores the implemented
            paint function or init function.
        """
        super().__init__(**kwargs)
        self.found_solution = None
        self.planner_data_pack = planner_data_pack

        # retrieve function for painting
        self.paint_func = planner_data_pack.visualise_pygame_paint

        if self.paint_func is None:
            # paint function has not been provided. Do nothing in paint function
            self.paint_func = nothing_to_paint

    def init(self, **kwargs):
        """
        The delayed *initialisation* method for planner visualiser.
        """
        super().init(**kwargs)

        if self.planner_data_pack is not None:
            if self.planner_data_pack.visualise_pygame_paint_init is not None:
                self.planner_data_pack.visualise_pygame_paint_init(self)

    def paint(self):
        """
        Paint method for planner visualiser.
        """
        self.paint_func(self)

    def draw_solution_path(self):
        """
        Method to draw solution path for planner visualiser.
        """
        if not self.found_solution or self.c_max == float("inf"):
            return
        # redraw new path
        self.args.env.solution_path_screen.fill(Colour.ALPHA_CK)
        nn = self.goal_pt.parent
        self.c_max = nn.cost
        while not nn.is_start:
            self.args.env.draw_path(
                nn,
                nn.parent,
                colour=Colour.blue,
                line_modifier=5,
                layer=self.args.env.solution_path_screen,
            )
            nn = nn.parent
        self.args.env.window.blit(self.args.env.path_layers, (0, 0))
        self.args.env.window.blit(self.args.env.solution_path_screen, (0, 0))

    def terminates_hook(self):
        """
        Terminates hook for planner visualiser that is executed at the end of loop.
        """
        if self.planner_data_pack is not None:
            if self.planner_data_pack.visualise_pygame_paint_terminate is not None:
                self.planner_data_pack.visualise_pygame_paint_terminate(self)


class PygameSamplerVisualiser(BaseSamplerVisualiser):
    """
    Visualisation of the sampler with Pygame engine
    """

    def __init__(
        self,
        sampler_data_pack: Optional[planner_registry.SamplerDataPack] = None,
        **kwargs,
    ):
        """
        :param sampler_data_pack: a sampler data pack that stores the implemented
        paint function or init function.
        """
        super().__init__(**kwargs)
        self.sampler_data_pack = sampler_data_pack
        self.paint_func = None

        # sampler can be nested, so sampler_data_pack is optional
        if sampler_data_pack is not None:
            # retrieve function for painting
            self.paint_func = sampler_data_pack.visualise_pygame_paint

        if self.paint_func is None:
            # paint function has not been provided. Do nothing in paint function
            self.paint_func = nothing_to_paint

    def init(self, **kwargs):
        """

        :param \**kwargs:

        """
        super().init(**kwargs)

        if self.sampler_data_pack is not None:
            if self.sampler_data_pack.visualise_pygame_paint_init is not None:
                self.sampler_data_pack.visualise_pygame_paint_init(self)

    def paint(self):
        """ """
        self.paint_func(self)

    def terminates_hook(self):
        """ """
        if self.sampler_data_pack is not None:
            if self.sampler_data_pack.visualise_pygame_paint_terminate is not None:
                self.sampler_data_pack.visualise_pygame_paint_terminate(self)


# noinspection PyAttributeOutsideInit
class PygameEnvVisualiser(BaseEnvVisualiser):
    """
    Environment Visualiser with the Pygame engine
    """

    def __init__(self: Env, **kwargs):
        super().__init__(**kwargs)
        self.extra = 25
        self.img = pygame.image.load(self.args.image)
        self.dim = kwargs["image_shape"]

    def visualiser_init(self: Env, no_display: bool = False):
        """Delayed *initialisation* method for environment visualiser.

        :param no_display: Controls whether turn on the
            visualisation or not, defaults to False. This option is deprecated,
            and instead, the environment should derived directly from the base
            visualisation to turn off visualisation.

        """
        self.args.no_display = no_display
        if self.args.no_display:
            return
        pygame.init()
        self.fpsClock = pygame.time.Clock()
        # self.fpsClock.tick(10)
        self.fpsClock.tick(10000)
        pygame.display.set_caption(self.args.planner.__class__.__name__)
        # screen.fill(white)
        ################################################################################
        # text
        pygame.font.init()
        self.myfont = pygame.font.SysFont(
            "Arial", int(self.dim[0] * 0.04 * self.args.scaling)
        )
        ################################################################################
        # main window
        self.window = pygame.display.set_mode(
            [
                int(self.dim[0] * self.args.scaling),
                int((self.dim[1] + self.extra) * self.args.scaling),
            ]
        )
        ################################################################################
        # background aka the room
        self.background = pygame.Surface([self.dim[0], (self.dim[1] + self.extra)])
        self.background.blit(self.img, (0, 0))
        # resize background to match windows
        self.background = pygame.transform.scale(
            self.background,
            [
                int(self.dim[0] * self.args.scaling),
                int((self.dim[1] + self.extra) * self.args.scaling),
            ],
        )
        ################################################################################
        # path of RRT*
        self.path_layers = pygame.Surface(
            [
                self.dim[0] * self.args.scaling,
                (self.dim[1] + self.extra) * self.args.scaling,
            ]
        )
        self.path_layers.fill(Colour.ALPHA_CK)
        self.path_layers.set_colorkey(Colour.ALPHA_CK)
        ################################################################################
        # layers to store the solution path
        self.solution_path_screen = pygame.Surface(
            [
                self.dim[0] * self.args.scaling,
                (self.dim[1] + self.extra) * self.args.scaling,
            ]
        )
        self.solution_path_screen.fill(Colour.ALPHA_CK)
        self.solution_path_screen.set_colorkey(Colour.ALPHA_CK)
        ################################################################################
        # layers to store the sampled points
        self.sampledPoint_screen = pygame.Surface(
            [
                self.dim[0] * self.args.scaling,
                (self.dim[1] + self.extra) * self.args.scaling,
            ]
        )
        self.sampledPoint_screen.fill(Colour.ALPHA_CK)
        self.sampledPoint_screen.set_colorkey(Colour.ALPHA_CK)
        ################################################################################

    def set_start_goal_points(
        self: Env, start: Optional[np.ndarray] = None, goal: Optional[np.ndarray] = None
    ):
        """A function that query user for start/goal and set them accordingly.

        :param start: the start configuration of the motion planning problem
        :param goal: the goal configuration of the motion planning problem

        """
        from utils.common import Node

        ##################################################
        # Get starting and ending point
        LOGGER.info("Select Starting Point and then Goal Point")
        if start is not None and goal is not None:
            return start, goal
        self.start_pt = self.goal_pt = None
        self.update_screen(update_all=True)
        while start is None or goal is None:
            mouse_pos = None
            for e in pygame.event.get():
                if e.type == MOUSEBUTTONDOWN:
                    mouse_pos = np.array(e.pos) / self.args.scaling
                    from collisionChecker import RobotArm4dCollisionChecker

                    if type(self.cc) == RobotArm4dCollisionChecker:
                        mouse_pos = np.array(
                            [*mouse_pos, *np.random.uniform(-np.pi, np.pi, 2)]
                        )
                    if not self.cc.feasible(mouse_pos):
                        # failed to pass collision check
                        mouse_pos = None
                    elif e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                        LOGGER.info("Leaving.")
                        return
            # convert mouse pos to Node
            if mouse_pos is not None:
                if start is None:
                    start = Node(mouse_pos)
                    self.start_pt = start
                    LOGGER.info(f"starting point set: {mouse_pos}")
                elif goal is None:
                    goal = Node(mouse_pos)
                    self.goal_pt = goal
                    LOGGER.info(f"goal point set: {mouse_pos}")
            self.update_screen(update_all=True)
        return start, goal

    @staticmethod
    def process_pygame_event():
        """ """
        for e in pygame.event.get():
            if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                LOGGER.info("Leaving.")
                sys.exit(0)

    def pygame_show(self: Env):
        """

        :param self: Env:

        """
        self.args.no_display = False

    def pygame_hide(self: Env):
        """

        :param self: Env:

        """
        self.args.no_display = True
        pygame.display.iconify()
        # pygame.quit()

    def draw_stick_robot(
        self: Env,
        node,
        colour1=Colour.cAlpha(Colour.orange, 128),
        colour2=Colour.cAlpha(Colour.cyan, 128),
        line_modifier=2.5,
        layer=None,
    ):
        """

        :param self: Env:
        :param node:
        :param colour1:  (Default value = Colour.cAlpha(Colour.orange)
        :param 128):
        :param colour2:  (Default value = Colour.cAlpha(Colour.cyan)
        :param line_modifier:  (Default value = 2.5)
        :param layer:  (Default value = None)

        """

        # draw config for node 1
        pt1 = node.pos[:2]
        pt2 = self.cc.get_pt_from_angle_and_length(
            pt1, node.pos[2], self.cc.stick_robot_length_config[0]
        )
        pt3 = self.cc.get_pt_from_angle_and_length(
            pt2, node.pos[3], self.cc.stick_robot_length_config[1]
        )

        pt2 = np.array(pt2)
        pt3 = np.array(pt3)

        pygame.draw.line(
            layer,
            colour1,
            pt1 * self.args.scaling,
            pt2 * self.args.scaling,
            int(line_modifier * self.args.scaling),
        )
        pygame.draw.line(
            layer,
            colour2,
            pt2 * self.args.scaling,
            pt3 * self.args.scaling,
            int(line_modifier * self.args.scaling),
        )

        # pygame.draw.circle(
        #     layer,
        #     Colour.green,
        #     (pt1 * self.args.scaling).astype(int),
        #     int(2 * self.args.scaling),
        # )

    def draw_path(
        self: Env, node1, node2, colour=Colour.path_blue, line_modifier=1, layer=None
    ):
        """

        :param self: Env:
        :param node1:
        :param node2:
        :param colour:  (Default value = Colour.path_blue)
        :param line_modifier:  (Default value = 1)
        :param layer:  (Default value = None)

        """
        if layer is None:
            layer = self.path_layers
        if node1 is not None and node2 is not None:
            pygame.draw.line(
                layer,
                colour,
                node1.pos[:2] * self.args.scaling,
                node2.pos[:2] * self.args.scaling,
                int(line_modifier * self.args.scaling),
            )

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
            pygame.draw.line(
                layer,
                colour,
                node1.pos * self.args.scaling,
                node2.pos * self.args.scaling,
                int(line_modifier * self.args.scaling),
            )

    def draw_circle(self: Env, pos, colour, radius, layer):
        """

        :param self: Env:
        :param pos:
        :param colour:
        :param radius:
        :param layer:

        """
        draw_pos = int(pos[0] * self.args.scaling), int(pos[1] * self.args.scaling)
        pygame.draw.circle(layer, colour, draw_pos, int(radius * self.args.scaling))

    def update_screen(self: Env, update_all=False):
        """

        :param self: Env:
        :param update_all:  (Default value = False)

        """
        self.process_pygame_event()
        if "refresh_cnt" not in self.__dict__:
            # INIT (this section will only run when this function is first called)
            self.refresh_cnt = 0

        if update_all or self.args.always_refresh:
            count = 0  # FORCE UPDATE
        else:
            count = self.refresh_cnt
            self.refresh_cnt += 1

        ##################################################
        def draw_start_goal_pt():
            """ """
            if self.start_pt is not None:
                self.draw_circle(
                    pos=self.start_pt.pos,
                    colour=Colour.red,
                    radius=self.args.goal_radius,
                    layer=self.path_layers,
                )
            if self.goal_pt is not None:
                self.draw_circle(
                    pos=self.goal_pt.pos,
                    colour=Colour.green,
                    radius=self.args.goal_radius,
                    layer=self.path_layers,
                )

        # limits the screen update
        if count % 20 == 0:
            self.window.blit(self.background, (0, 0))

        if count % 60 == 0:
            try:
                self.planner.paint()
            except AttributeError as e:
                # print(e)
                pass
            draw_start_goal_pt()

        # Tree paths
        if count % 20 == 0:
            self.window.blit(self.path_layers, (0, 0))
            self.window.blit(self.solution_path_screen, (0, 0))
            draw_start_goal_pt()

        # Sampler hook
        if count % 20 == 0:
            try:
                self.args.sampler.paint()
            except AttributeError as e:
                print(e)
                pass

        # Sampled points
        if count % 4 == 0:
            self.sampledPoint_screen.fill(Colour.ALPHA_CK)
            # Draw sampled nodes
            for sampledPos in self.stats.sampledNodes:
                self.draw_circle(
                    pos=sampledPos,
                    colour=Colour.red,
                    radius=2,
                    layer=self.sampledPoint_screen,
                )
            self.window.blit(self.sampledPoint_screen, (0, 0))
            # remove them from list
            del self.stats.sampledNodes[:]

        # Texts
        if count % 10 == 0:
            _cost = (
                "INF"
                if self.planner.c_max == float("inf")
                else round(self.planner.c_max, 2)
            )
            text = "Cost: {} | Inv.Samples: {}(con) {}(obs)".format(
                _cost,
                self.stats.invalid_samples_connections,
                self.stats.invalid_samples_obstacles,
            )
            self.window.blit(
                self.myfont.render(text, False, Colour.white, Colour.black),
                (10, (self.dim[1] + self.extra) * self.args.scaling * 0.95),
            )

        pygame.display.update()


############################################################
#              Klampt (3D) visualisation                   #
############################################################


class KlamptPlannerVisualiser(BasePlannerVisualiser):
    """ """

    def init(self: Planner, **kwargs):
        """

        :param self: Planner: 
        :param \**kwargs:

        """
        return

    def paint(self: Planner, **kwargs):
        """

        :param self: Planner: 
        :param \**kwargs:

        """

        def get_world_pos(config):
            """

            :param config: 

            """
            self.args.env.cc.robot.setConfig(
                self.args.env.cc.translate_to_klampt(config)
            )
            link = self.args.env.cc.robot.link(11)
            pos = link.getWorldPosition([0, 0, 0])
            return pos

        def nothing_paint():
            """ """
            pass

        def RRdTSampler_paint():
            """ """

            # return
            def get_color_transists(value, max_prob, min_prob):
                """

                :param value: 
                :param max_prob: 
                :param min_prob: 

                """
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
                self.args.env.draw_node(
                    get_world_pos(p.pos), colour=color, size=25, label=id(p)
                )

        def _helper_draw_nodes_edges(nodes, colour):
            """

            :param nodes: 
            :param colour: 

            """
            drawn_nodes_pairs = set()
            for n in nodes:
                self.args.env.draw_node(get_world_pos(n.pos), colour=colour)
                if n.parent is not None:
                    new_set = frozenset({n, n.parent})
                    if new_set not in drawn_nodes_pairs:
                        drawn_nodes_pairs.add(new_set)
                        self.args.env.draw_path(
                            get_world_pos(n.pos),
                            get_world_pos(n.parent.pos),
                            colour=colour,
                        )
            if self.goal_pt.parent is not None:
                self.draw_solution_path()

        def RRTPlanner_paint():
            """ """
            # self.args.env.path_layers.fill(Colour.ALPHA_CK)
            _helper_draw_nodes_edges(nodes=self.nodes, colour=(1, 0, 0, 1))

        def BiRRTPlanner_paint():
            """ """
            drawn_nodes_pairs = set()
            for c, nodes in (
                ((1, 0, 0, 1), self.nodes),
                ((0, 0, 1, 1), self.goal_tree_nodes),
            ):
                _helper_draw_nodes_edges(nodes=nodes, colour=c)

        def PRMPlanner_paint():
            """ """
            edges = list()
            colour = (1, 0, 0, 1)
            for n in self.nodes:
                self.args.env.draw_node(get_world_pos(n.pos), colour=colour)
            for edge in self.graph.edges:
                edge = np.array(edge).transpose()
                self.args.env.draw_path(
                    get_world_pos(n.pos), get_world_pos(n.parent.pos), colour=colour
                )

        def RRdTPlanner_paint():
            """ """
            from planners.rrdtPlanner import BFS

            drawn_nodes_pairs = set()

            def generate_random_colors():
                """ """
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

                            self.args.env.draw_path(
                                get_world_pos(newnode.pos),
                                get_world_pos(e.pos),
                                colour=c,
                            )
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
                        self.args.env.draw_path(
                            get_world_pos(n.pos), get_world_pos(n.parent.pos), colour=c
                        )

        def RRFPlanner_paint():
            """ """
            from planners.rrdtPlanner import BFS

            drawn_nodes_pairs = set()

            def generate_random_colors():
                """ """
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

                            self.args.env.draw_path(
                                get_world_pos(newnode.pos),
                                get_world_pos(e.pos),
                                colour=c,
                            )
            # Draw root tree
            c = next(color_gen)
            # override to red
            c = (1, 0, 0, 1)
            # draw nodes
            for root, c in zip(
                (self.root, self.goal_root), ((1, 0, 0, 1), (0, 0, 1, 1))
            ):
                for node in root.nodes:
                    self.args.env.draw_node(get_world_pos(node.pos), colour=c)
                # draw edges
                for n in root.nodes:
                    if n.parent is not None:
                        new_set = frozenset({n, n.parent})
                        if new_set not in drawn_nodes_pairs:
                            drawn_nodes_pairs.add(new_set)
                            self.args.env.draw_path(
                                get_world_pos(n.pos),
                                get_world_pos(n.parent.pos),
                                colour=c,
                            )

            # self.draw_solution_path()

        def default_paint():
            """ """
            raise NotImplementedError(
                f"{self.__class__.__name__} visualising " "is not not implemented!"
            )

        implemented_classes = {
            "RandomPolicySampler": nothing_paint,
            "InformedRRTSampler": InformedRRTSampler_paint,
            "RRdTSampler": RRdTSampler_paint,
            "RRdTPlanner": RRdTPlanner_paint,
            "RRFSampler": RRdTSampler_paint,
            "RRFPlanner": RRFPlanner_paint,
            "PRMSampler": nothing_paint,
            "PRMPlanner": PRMPlanner_paint,
            "RRTSampler": nothing_paint,
            "RRTPlanner": RRTPlanner_paint,
            "BiRRTSampler": nothing_paint,
            "BiRRTPlanner": BiRRTPlanner_paint,
        }

        implemented_classes.get(self.__class__.__name__, default_paint)()

    def draw_solution_path(self):
        """ """
        # not implemented
        return

    def terminates_hook(self):
        """ """
        if self.__class__.__name__ == "PRMPlanner":
            self.build_graph()
            # draw all edges
            for n1, n2 in self.tree.edges():
                self.args.env.draw_path(n1, n2, Colour.path_blue)
            self.get_solution()
            self.args.env.update_screen()
            import time

            time.sleep(30)


class KlamptEnvVisualiser(BaseEnvVisualiser):
    """ """

    def __init__(self, **kwargs):
        self.drawn_label = set()
        self.kwargs = kwargs

    def visualiser_init(self, no_display=False):
        """

        :param no_display:  (Default value = False)

        """
        if not no_display:
            from klampt import vis

            vis.add("world", self.kwargs["cc"].world)
            vis.show()
        return

    def set_start_goal_points(
        self, start: Optional[np.ndarray] = None, goal: Optional[np.ndarray] = None
    ):
        """

        :param start: 
        :param goal: 

        """
        if start:
            start = self.cc.translate_to_klampt(start)
        if goal:
            goal = self.cc.translate_to_klampt(goal)
        from klampt.io import resource

        def user_set_config(q, type_str="<..>"):
            """

            :param q: 
            :param type_str:  (Default value = "<..>")

            """
            save = None
            # it's worthwhile to make sure that it's feasible
            while q is None or not self.cc.space.feasible(q):
                print(type_str + " configuration isn't feasible")
                save, q = resource.edit(
                    type_str + " config", q, "Config", world=self.cc.world
                )
            return save, q

        # start = [0.0, -1.5800000000000005, 1.2, -0.37, -1.57, -1.57, -1.57, 0.0, 0.048, 0.048, -0.048, 0.048]
        # goal = [0.0, -1.5800000000000005, 1.2, -0.37, -1.57, -1.57, -1.57, 0.0, 0.048, 0.048, -0.048, 0.048]
        _, start = user_set_config(start, "Start")
        _, goal = user_set_config(goal, "Goal")

        return tuple(map(self.cc.translate_from_klampt, (start, goal)))

    def draw_node(self, pos, colour=(1, 0, 0, 1), size=15, label=None):
        """

        :param pos: 
        :param colour:  (Default value = (1)
        :param 0: 
        :param 1): 
        :param size:  (Default value = 15)
        :param label:  (Default value = None)

        """
        from klampt import vis
        from klampt.model.coordinates import Point

        if label is None:
            label = repr(pos)
        vis.add(label, Point(pos), keepAppearance=True)
        vis.setAttribute(label, "size", size)
        vis.setAttribute(label, "color", colour)
        vis.hideLabel(label)

    def draw_path(self, pos1, pos2, colour=None):
        """

        :param pos1: 
        :param pos2: 
        :param colour:  (Default value = None)

        """
        from klampt import vis
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
        """

        :param pos: 
        :param colour: 
        :param radius: 
        :param layer: 

        """
        draw_pos = int(pos[0] * self.args.scaling), int(pos[1] * self.args.scaling)
        pygame.draw.circle(layer, colour, draw_pos, int(radius * self.args.scaling))

    def update_screen(self, update_all=False):
        """

        :param update_all:  (Default value = False)

        """
        if "refresh_cnt" not in self.__dict__:
            # INIT (this section will only run when this function is first called)
            self.refresh_cnt = 0

        if update_all or self.args.always_refresh:
            count = 0  # FORCE UPDATE
        else:
            count = self.refresh_cnt
            self.refresh_cnt += 1

        ###################################################################################
        def draw_start_goal_pt():
            """ """
            if self.start_pt is not None:
                self.draw_circle(
                    pos=self.start_pt.pos,
                    colour=Colour.red,
                    radius=self.args.goal_radius,
                    layer=self.path_layers,
                )
            if self.goal_pt is not None:
                self.draw_circle(
                    pos=self.goal_pt.pos,
                    colour=Colour.green,
                    radius=self.args.goal_radius,
                    layer=self.path_layers,
                )

        if count % 60 == 0:
            try:
                self.planner.paint()
            except AttributeError as e:
                # print(e)
                pass

        # Sampler hook
        if count % 20 == 0:
            try:
                self.args.sampler.paint()
            except AttributeError as e:
                # print(e)
                pass

        # # Sampled points
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


class KlamptSamplerVisualiser(BaseSamplerVisualiser):
    """ """

    def __init__(
        self,
        sampler_data_pack: Optional[planner_registry.SamplerDataPack] = None,
        **kwargs,
    ):
        super().__init__(**kwargs)
        self.sampler_data_pack = sampler_data_pack
        self.paint_func = None

        # # sampler can be nested, so sampler_data_pack is optional
        # if sampler_data_pack is not None:
        #     # retrieve function for painting
        #     self.paint_func = sampler_data_pack.visualise_pygame_paint
        #
        # if self.paint_func is None:
        #     # paint function has not been provided. Do nothing in paint function
        #     self.paint_func = nothing_to_paint

    def init(self, **kwargs):
        """

        :param \**kwargs:

        """
        super().init(**kwargs)

    def paint(self):
        """ """
        self.paint_func(self)

    def terminates_hook(self):
        """ """
        pass
        # if self.sampler_data_pack is not None:
        #     if self.sampler_data_pack.visualise_pygame_paint_terminate is not None:
        #         self.sampler_data_pack.visualise_pygame_paint_terminate(self)


class VisualiserSwitcher:
    """Default to Pygame visualiser"""

    env_clname = PygameEnvVisualiser
    planner_clname = PygamePlannerVisualiser
    sampler_clname = PygameSamplerVisualiser

    @staticmethod
    def choose_visualiser(visualiser_type: str):
        """

        :param visualiser_type: str: 

        """
        if visualiser_type == "base":
            VisualiserSwitcher.env_clname = BaseEnvVisualiser
            VisualiserSwitcher.planner_clname = BasePlannerVisualiser
            VisualiserSwitcher.sampler_clname = BaseSamplerVisualiser
        elif visualiser_type == "pygame":
            VisualiserSwitcher.env_clname = PygameEnvVisualiser
            VisualiserSwitcher.planner_clname = PygamePlannerVisualiser
            VisualiserSwitcher.sampler_clname = PygameSamplerVisualiser
        elif visualiser_type == "klampt":
            VisualiserSwitcher.env_clname = KlamptEnvVisualiser
            VisualiserSwitcher.planner_clname = KlamptPlannerVisualiser
            VisualiserSwitcher.sampler_clname = KlamptSamplerVisualiser
        else:
            raise ValueError(f"Unknown visualiser_type {visualiser_type}")
