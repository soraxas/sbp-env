from abc import ABC
from typing import Tuple, Callable

import numpy as np

from ..utils.common import PlanningOptions, Stats
from ..visualiser import VisualiserSwitcher, BaseSamplerVisualiser


# noinspection PyAttributeOutsideInit
class Sampler(ABC):
    """
    Abstract base sampler that defines each unique methods that some
    sampler, but not all samplers, uses.
    """

    GetNextPosReturnType = Tuple[np.ndarray, Callable, Callable]
    """The return type of :func:`get_next_pos`."""

    def __init__(self, **kwargs):
        super().__init__()
        self.use_radian = False
        if "sampler_data_pack" in kwargs:
            self.visualiser = VisualiserSwitcher.sampler_clname(
                sampler_instance=self, sampler_data_pack=kwargs["sampler_data_pack"]
            )
        else:
            # if kwargs does not contains data-pack, it means we do not need to
            # visualise this sampler (e.g. nested sampler)
            self.visualiser = BaseSamplerVisualiser()

    def init(self, use_radian: bool = False, **kwargs):
        """The delayed **initialisation** method

        :param use_radian: whether this sampler should returns value in radian (as
            opposite to Euclidean)
        :param start_pt: the starting configuration for the planning problem
        :param goal_pt: the goal configuration for the planning problem

        :type start_pt: :class:`~utils.common.Node`
        :type goal_pt: :class:`~utils.common.Node`

        """
        self.args = kwargs["args"]
        self.start_pos = self.args.start_pt.pos
        self.goal_pos = self.args.goal_pt.pos
        self.use_radian = use_radian
        self.visualiser.init(**kwargs)

    def get_next_pos(self, **kwargs):
        """Retrieve next sampled position

        :return: a sampled position, a callable to report success,
            and a callable to report failure
        """
        raise NotImplementedError()

    def get_valid_next_pos(self):
        """Loop until we find a valid next node. Uses ``get_next_pos`` internally."""
        while True:
            coordinate, report_success, report_fail = self.get_next_pos()
            Stats.get_instance().add_sampled_node(coordinate)
            if self.args.engine.cc.feasible(coordinate):
                return coordinate, report_success, report_fail
            report_fail(pos=coordinate, obstacle=True)
            Stats.get_instance().add_invalid(obs=True)

    def set_use_radian(self, value: bool = True):
        """Set this sampler to use radian or not

        :param value: the value to set

        """
        self.use_radian = value

    def report_success(self, **kwargs):
        """Report to the sampler that the last sample was successfully. This function is
        sampler dependent.

        :param kwargs: pass through to derived class

        """
        pass

    def report_fail(self, **kwargs):
        """Report to the sampler that the last sample was unsuccessful. This function is
        sampler dependent.

        :param kwargs: pass through to derived class

        """
        pass

    def add_tree_node(self, **kwargs):
        """Report to the sampler about the last node that was added to the tree

        :param kwargs: pass through to derived class

        """
        pass

    def add_sample_line(self, **kwargs):
        """Report to the sampler about the entire line that was sampled last time

        :param kwargs: pass through to derived class

        """
        pass

    @property
    def name(self) -> str:
        return self.__class__.__name__
