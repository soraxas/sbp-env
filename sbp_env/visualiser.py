from __future__ import annotations

import logging
import os
import sys
from abc import ABC
from typing import Optional
from typing import TYPE_CHECKING

import numpy as np

from .utils.common import Colour, Stats
from .utils.common import Node

from . import collisionChecker
from .collisionChecker import RobotArm4dCollisionChecker

os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"  # disable pygame prompt

import pygame
from pygame.locals import *

if TYPE_CHECKING:
    from .utils import planner_registry
    from .env import Env
    from .planners.basePlanner import Planner
    from .samplers.baseSampler import Sampler

LOGGER = logging.getLogger(__name__)


def nothing_to_paint(*args):
    """Paint function that does nothing.

    :param args: unused
    """
    pass


############################################################
#              Base without visualisation                  #
############################################################


class BasePlannerVisualiser(ABC):
    """
    Base Planner Visualiser that performs nothing.
    Good for benchmark when you don't care about visual inspection.
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
    Base Sampler Visualiser that performs nothing.
    Good for benchmark when you don't care about visual inspection.
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
    Base Environment Visualiser that performs nothing.
    Good for benchmark when you don't care about visual inspection.
    """

    def __init__(self, env_instance: Env, **kwargs):
        self.env_instance = env_instance
        self.args = env_instance.args

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
            raise RuntimeError(
                "start/goal have not been set.\n  Consider passing the start/goal with \n  $ python main.py <PLANNER> <MAP> start <start_x1,x2,..,xn> goal <goal_x1,x2,..,xn>"
            )
        return start, goal

    def __getattr__(self, attr):
        """This is called what self.attr doesn't exist.
        Forward the call to the :class:`Env` instance
        """
        return object.__getattribute__(self.env_instance, attr)

    def terminates_hook(self):
        self.env_instance.planner.visualiser.terminates_hook()
        self.env_instance.sampler.visualiser.terminates_hook()


############################################################
#              Pygame (2D) visualisation                   #
############################################################


class PygamePlannerVisualiser(BasePlannerVisualiser):
    """
    Planner Visualiser with the Pygame engine.py
    """

    def __init__(
        self,
        planner_instance: Planner,
        planner_data_pack: planner_registry.PlannerDataPack,
        **kwargs,
    ):
        """
        :param planner_instance: an instance of a planner
        :param planner_data_pack: a planner data pack that stores the implemented
            paint function or init function.
        """
        super().__init__(**kwargs)
        self.planner_instance = planner_instance
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
                self.planner_data_pack.visualise_pygame_paint_init(
                    self.planner_instance
                )

    def paint(self):
        """
        Paint method for planner visualiser.
        """
        self.paint_func(self.planner_instance)

    def draw_solution_path(self):
        """
        Method to draw solution path for planner visualiser.
        """
        solution_path = self.planner_instance.args.env.get_solution_path()
        if solution_path is None:
            return
        # redraw new path
        self.planner_instance.args.env.solution_path_screen.fill(Colour.ALPHA_CK)
        last_parent = solution_path[0]
        for node in solution_path[1:]:
            self.planner_instance.args.env.draw_path(
                last_parent,
                node,
                colour=Colour.blue,
                line_modifier=5,
                layer=self.planner_instance.args.env.solution_path_screen,
            )
            last_parent = node
        self.planner_instance.args.env.window.blit(
            self.planner_instance.args.env.path_layers, (0, 0)
        )
        self.planner_instance.args.env.window.blit(
            self.planner_instance.args.env.solution_path_screen, (0, 0)
        )

    def terminates_hook(self):
        """
        Terminates hook for planner visualiser that is executed at the end of loop.
        """
        if self.planner_data_pack is not None:
            if self.planner_data_pack.visualise_pygame_paint_terminate is not None:
                self.planner_data_pack.visualise_pygame_paint_terminate(
                    self.planner_instance
                )


class PygameSamplerVisualiser(BaseSamplerVisualiser):
    """
    Visualisation of the sampler with Pygame engine.py
    """

    def __init__(
        self,
        sampler_instance: Sampler,
        sampler_data_pack: Optional[planner_registry.SamplerDataPack] = None,
        **kwargs,
    ):
        """
        :param sampler_instance: an instance of a sampler
        :param sampler_data_pack: a sampler data pack that stores the implemented
            paint function or init function.
        """
        super().__init__(**kwargs)
        self.sampler_instance = sampler_instance
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
        super().init(**kwargs)

        if self.sampler_data_pack is not None:
            if self.sampler_data_pack.visualise_pygame_paint_init is not None:
                self.sampler_data_pack.visualise_pygame_paint_init(
                    self.sampler_instance
                )

    def paint(self):
        self.paint_func(self.sampler_instance)

    def terminates_hook(self):
        if self.sampler_data_pack is not None:
            if self.sampler_data_pack.visualise_pygame_paint_terminate is not None:
                self.sampler_data_pack.visualise_pygame_paint_terminate(
                    self.sampler_instance
                )


# noinspection PyAttributeOutsideInit
class PygameEnvVisualiser(BaseEnvVisualiser):
    """
    Environment Visualiser with the Pygame engine.py
    """

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.extra = 25
        self.img = pygame.image.load(self.args.engine.cc.image_fname)
        self.dim = self.args.engine.upper

    def visualiser_init(self, no_display: bool = False):
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
        self,
        start: Optional[np.ndarray] = None,
        goal: Optional[np.ndarray] = None,
        **kwargs,
    ):
        """A function that query user for start/goal and set them accordingly.

        :param start: the start configuration of the motion planning problem
        :param goal: the goal configuration of the motion planning problem

        """

        ##################################################
        # Get starting and ending point
        LOGGER.info("Select Starting Point and then Goal Point")
        if start is not None and goal is not None:
            return start, goal
        self.env_instance.start_pt = self.env_instance.goal_pt = None
        self.update_screen(update_all=True)
        while start is None or goal is None:
            mouse_pos = None
            for e in pygame.event.get():
                if e.type == MOUSEBUTTONDOWN:
                    mouse_pos = np.array(e.pos) / self.args.scaling

                    if type(self.args.engine.cc) == RobotArm4dCollisionChecker:
                        mouse_pos = np.array(
                            [*mouse_pos, *np.random.uniform(-np.pi, np.pi, 2)]
                        )
                    if not self.args.engine.cc.feasible(mouse_pos):
                        # failed to pass collision check
                        mouse_pos = None
                    elif e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                        LOGGER.info("Leaving.")
                        return
            # convert mouse pos to Node
            if mouse_pos is not None:
                if start is None:
                    start = Node(mouse_pos)
                    self.env_instance.start_pt = start
                    LOGGER.info(f"starting point set: {mouse_pos}")
                elif goal is None:
                    goal = Node(mouse_pos)
                    self.env_instance.goal_pt = goal
                    LOGGER.info(f"goal point set: {mouse_pos}")
            self.update_screen(update_all=True)
        return start, goal

    @staticmethod
    def process_pygame_event():
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

    def draw_stick_robot(
        self,
        node,
        colour1=Colour.cAlpha(Colour.orange, 128),
        colour2=Colour.cAlpha(Colour.cyan, 128),
        line_modifier=2.5,
        layer=None,
    ):
        """
        Draw a 4D stick robotic arm

        :param node: the origin of the robot arm
        :param colour1: the colour of the first link
        :param colour2: the colour of the second link
        :param line_modifier: modify the weight of the link to be drawn
        :param layer: the layer to draw the arm
        """

        # draw config for node 1
        pt1 = node.pos[:2]
        pt2 = self.args.engine.cc.get_pt_from_angle_and_length(
            pt1, node.pos[2], self.args.engine.cc.stick_robot_length_config[0]
        )
        pt3 = self.args.engine.cc.get_pt_from_angle_and_length(
            pt2, node.pos[3], self.args.engine.cc.stick_robot_length_config[1]
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
        self, node1, node2, colour=Colour.path_blue, line_modifier=1, layer=None
    ):
        """Draw a path that represents an edge

        :param node1: the starting node of the edge
        :param node2: the ending node of the edge
        :param colour: the color of the edge
        :param line_modifier: modify the weight of the edge to be drawn
        :param layer: the layer to draw th edge

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

        if type(self.args.engine.cc) == collisionChecker.RobotArm4dCollisionChecker:
            self.draw_stick_robot(node1, layer=self.path_layers)
        else:
            pygame.draw.line(
                layer,
                colour,
                node1.pos * self.args.scaling,
                node2.pos * self.args.scaling,
                int(line_modifier * self.args.scaling),
            )

    def draw_circle(self, pos, colour, radius, layer):
        """Draw a circle (e.g. to represent a node)

        :param pos: the origin position of the circle
        :param colour: the color of the circle
        :param radius: the radius of the circle
        :param layer: the layer to draw the circle

        """
        draw_pos = int(pos[0] * self.args.scaling), int(pos[1] * self.args.scaling)
        pygame.draw.circle(layer, colour, draw_pos, int(radius * self.args.scaling))

    def update_screen(self, update_all=False):
        """Refresh the screen

        :param update_all: Force update the screen (as oppose to limit drawing to
            speed up)

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
            if self.env_instance.start_pt is not None:
                self.draw_circle(
                    pos=self.env_instance.start_pt.pos,
                    colour=Colour.red,
                    radius=self.args.goal_radius,
                    layer=self.path_layers,
                )
            if self.env_instance.goal_pt is not None:
                self.draw_circle(
                    pos=self.env_instance.goal_pt.pos,
                    colour=Colour.green,
                    radius=self.args.goal_radius,
                    layer=self.path_layers,
                )

        # limits the screen update
        if count % 20 == 0:
            self.window.blit(self.background, (0, 0))

        if count % 60 == 0:
            try:
                self.planner.visualiser.paint()
            except AttributeError as e:
                # only raise the exception if the planning had started
                # because during setup there might be attributes that are
                # not available yet.
                if self.env_instance.started:
                    raise e
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
                self.args.sampler.visualiser.paint()
            except AttributeError as e:
                if self.env_instance.started:
                    raise e
                # print(e)
                pass

        # Sampled points
        if count % 4 == 0:
            self.sampledPoint_screen.fill(Colour.ALPHA_CK)
            # Draw sampled nodes
            for sampledPos in Stats.get_instance().sampledNodes:
                self.draw_circle(
                    pos=sampledPos,
                    colour=Colour.red,
                    radius=2,
                    layer=self.sampledPoint_screen,
                )
            self.window.blit(self.sampledPoint_screen, (0, 0))
            # remove them from list
            del Stats.get_instance().sampledNodes[:]

        # Texts
        if count % 10 == 0:
            _cost = (
                "INF"
                if self.planner.c_max == float("inf")
                else round(self.planner.c_max, 2)
            )
            text = "Cost: {} | Inv.Samples: {}(con) {}(obs)".format(
                _cost,
                Stats.get_instance().invalid_samples_connections,
                Stats.get_instance().invalid_samples_obstacles,
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
    """
    Planner Visualiser with the Klampt engine.py
    """

    def __init__(
        self,
        planner_instance: Planner,
        planner_data_pack: planner_registry.PlannerDataPack,
        **kwargs,
    ):
        """
        :param planner_instance: an instance of a planner
        :param planner_data_pack: a planner data pack that stores the implemented
            paint function or init function.
        """
        super().__init__(**kwargs)
        self.planner_instance = planner_instance
        self.planner_data_pack = planner_data_pack

        # retrieve function for painting
        self.paint_func = planner_data_pack.visualise_klampt_paint

        if self.paint_func is None:
            # paint function has not been provided. Do nothing in paint function
            self.paint_func = nothing_to_paint

    def init(self, **kwargs):
        """
        The delayed *initialisation* method for planner visualiser.
        """
        super().init(**kwargs)

        if self.planner_data_pack is not None:
            if self.planner_data_pack.visualise_klampt_paint_init is not None:
                self.planner_data_pack.visualise_klampt_paint_init(
                    self.planner_instance
                )

    def paint(self, **kwargs):
        """
        Paint method for planner visualiser.
        """
        self.paint_func(self.planner_instance)

    def terminates_hook(self):
        """
        Terminates hook for planner visualiser that is executed at the end of loop.
        """
        if self.planner_data_pack is not None:
            if self.planner_data_pack.visualise_klampt_paint_terminate is not None:
                self.planner_data_pack.visualise_klampt_paint_terminate(
                    self.planner_instance
                )


class KlamptEnvVisualiser(BaseEnvVisualiser):
    """
    Environment Visualiser with the Klampt engine.py
    """

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.drawn_label = set()

    def visualiser_init(self, no_display=False):
        """Delayed *initialisation* method for environment visualiser.

        :param no_display: Controls whether turn on the
            visualisation or not, defaults to False. This option is deprecated,
            and instead, the environment should derived directly from the base
            visualisation to turn off visualisation.

        """
        if not no_display:
            from klampt import vis

            vis.add("world", self.args.engine.cc.world)
            vis.show()

    def set_start_goal_points(
        self,
        start: Optional[np.ndarray] = None,
        goal: Optional[np.ndarray] = None,
        **kwargs,
    ):
        """A function that query user for start/goal and set them accordingly.

        :param start: the start configuration of the motion planning problem
        :param goal: the goal configuration of the motion planning problem

        """
        from klampt.io import resource

        def user_set_config(q, type_str="<..>"):
            """

            :param q:
            :param type_str:  (Default value = "<..>")

            """
            save = None
            # it's worthwhile to make sure that it's feasible
            while q is None or not self.args.engine.cc.feasible(q):
                print("=" * 20)
                print("=" * 20)
                print(type_str, q)
                print(type_str + " configuration isn't feasible")
                save, q = resource.edit(
                    type_str + " config", q, "Config", world=self.args.engine.cc.world
                )
                q = self.args.engine.cc.translate_from_klampt(q)

            return save, q

        _, start = user_set_config(start, "Start")
        _, goal = user_set_config(goal, "Goal")

        return start, goal

    def draw_node(self, pos, colour=(1, 0, 0, 1), size=15, label=None):
        """Draw a node in the klampt visualiser

        :param pos: the position of the node
        :param colour: the colour of the node
        :param size: the size of the node
        :param label: if given, add label to the node

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
        """Draw a path (line) in the klampt visualiser

        :param pos1: the position of the start of line
        :param pos2: the position of the end of line
        :param colour: the colour of the line

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

    def update_screen(self, update_all=False):
        """Refresh the screen

        :param update_all: Force update the screen (as oppose to limit drawing to
            speed up)

        """
        if "refresh_cnt" not in self.__dict__:
            # INIT (this section will only run when this function is first called)
            self.refresh_cnt = 0

        if update_all or self.args.always_refresh:
            count = 0  # FORCE UPDATE
        else:
            count = self.refresh_cnt
            self.refresh_cnt += 1

        if count % 60 == 0:
            try:
                self.planner.visualiser.paint()
            except AttributeError as e:
                if self.env_instance.started:
                    raise e
                # print(e)
                pass

        # Sampler hook
        if count % 20 == 0:
            try:
                self.args.sampler.visualiser.paint()
            except AttributeError as e:
                if self.env_instance.started:
                    raise e


class KlamptSamplerVisualiser(BaseSamplerVisualiser):
    """
    Visualisation of the sampler with Klampt engine.py
    """

    def __init__(
        self,
        sampler_data_pack: Optional[planner_registry.SamplerDataPack] = None,
        **kwargs,
    ):
        super().__init__(**kwargs)
        self.sampler_data_pack = sampler_data_pack
        self.paint_func = None

        # sampler can be nested, so sampler_data_pack is optional
        if sampler_data_pack is not None:
            # retrieve function for painting
            self.paint_func = sampler_data_pack.visualise_klampt_paint

        if self.paint_func is None:
            # paint function has not been provided. Do nothing in paint function
            self.paint_func = nothing_to_paint

    def init(self, **kwargs):
        super().init(**kwargs)

    def paint(self):
        self.paint_func(self)

    def terminates_hook(self):
        if self.sampler_data_pack is not None:
            if self.sampler_data_pack.visualise_klampt_paint_terminate is not None:
                self.sampler_data_pack.visualise_klampt_paint_terminate(self)


# noinspection PyAttributeOutsideInit
class BlackBoxEnvVisualiser(BaseEnvVisualiser):
    """
    Environment Visualiser with the Pygame engine.py
    """

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    # def update_screen(self, **kwargs):
    #     import matplotlib.pyplot as plt
    #
    #     # self.paint_func(self)
    #     self.__plot()
    #     plt.legend()
    #     plt.show()

    def __plot(self):
        import matplotlib.pyplot as plt

        from .randomness import RandomnessManager

        randomness = RandomnessManager(self.env_instance.args.engine.get_dimension())

        pts = self.env_instance.args.engine.transform(
            np.array([randomness.get_random("sobol_sequence") for _ in range(10000)])
        )
        # TODO: stop cc from collecting stats
        obs_pts = [
            pt for pt in pts if not self.env_instance.args.engine.cc.feasible(pt)
        ]

        plt.scatter(*self.env_instance.start_pt, c="green", label="start")
        plt.scatter(*self.env_instance.goal_pt, c="red", label="goal")
        plt.scatter(*np.array(obs_pts).T, c="black", label="infeasible")
        sol = self.get_solution_path(as_array=True)
        if sol is not None:
            plt.plot(*sol.T, label="solution path")

    def terminates_hook(self):
        import matplotlib.pyplot as plt

        self.__plot()
        self.env_instance.planner.visualiser.terminates_hook()
        self.env_instance.sampler.visualiser.terminates_hook()
        plt.legend()
        plt.show()


class VisualiserSwitcher:
    """Default to Pygame visualiser"""

    env_clname = PygameEnvVisualiser
    planner_clname = BasePlannerVisualiser
    sampler_clname = BaseSamplerVisualiser

    @staticmethod
    def choose_visualiser(visualiser_type: str):
        """Select the visualiser to use

        :param visualiser_type: the type of visualiser to use

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
        elif visualiser_type == "blackbox":
            VisualiserSwitcher.env_clname = BlackBoxEnvVisualiser
        else:
            raise ValueError(f"Unknown visualiser_type {visualiser_type}")
