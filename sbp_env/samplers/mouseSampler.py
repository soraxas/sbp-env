import time

from overrides import overrides

from ..samplers.baseSampler import Sampler
from ..utils import planner_registry

"""
For demo / testing only. This policy wait for user mouse input for next sampling node.
"""


class MouseSampler(Sampler):
    """A sampler that hooks into *PyGame*'s functionality to create sampled
    configurations with a mouse/touchpad, for **testing** if the motion planner is
    functioning correctly in creating tree edges or if certain bottlenecks within a
    map is accessibly.

    .. warning::
        This sampler is for **testing purpose** only, and hence will not be
        extended for :math:`d > 2` or for collision checker other than *PyGame*.

    """

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    @overrides
    def get_next_pos(self):
        return (
            self.get_mouse_click_position(scaling=self.args.scaling),
            self.report_success,
            self.report_fail,
        )

    @staticmethod
    def get_mouse_click_position(scaling):
        """Helper function to return the current position of the mouse pointer

        :param scaling: scaling factor ot be applied to the mouse position

        """
        import pygame

        while True:
            time.sleep(0.05)
            pygame.event.wait()
            if pygame.mouse.get_pressed()[0] == 1:  # Left mouse pressed
                pos = pygame.mouse.get_pos()
                pos = (int(pos[0] / scaling), int(pos[1] / scaling))
                return pos


# start register
sampler_id = "mouse"

planner_registry.register_sampler(
    sampler_id,
    sampler_class=MouseSampler,
)
# finish register
