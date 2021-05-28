from utils.helpers import MagicDict
from visualiser import VisualiserSwitcher


# noinspection PyAttributeOutsideInit
class Sampler(VisualiserSwitcher.sampler_clname):
    """
    Abstract base sampler that defines each unique methods that some
    sampler, but not all samplers, uses.
    """

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.use_radian = False

    def init(self, use_radian: bool = False, **kwargs):
        """The delayed **initialisation** method

        :param use_radian: whether this sampler should returns value in radian (as
            opposite to Euclidean)
        :param start_pt: the starting configuration for the planning problem
        :param goal_pt: the goal configuration for the planning problem

        :type start_pt: :class:`~utils.helpers.Node`
        :type goal_pt: :class:`~utils.helpers.Node`

        """
        self.args = MagicDict(kwargs)
        self.start_pos = kwargs["start_pt"].pos
        self.goal_pos = kwargs["goal_pt"].pos
        self.use_radian = use_radian

        super().init(**kwargs)

    def get_next_pos(self, **kwargs):
        """Retrieve next sampled position. Must be override by subclass.

        :param **kwargs: not used

        """
        raise NotImplementedError()

    def get_valid_next_pos(self):
        """Loop until we find a valid next node. Uses ``get_next_pos`` internally."""
        while True:
            coordinate, report_success, report_fail = self.get_next_pos()
            self.args.env.stats.add_sampled_node(coordinate)
            if self.args.env.cc.feasible(coordinate):
                return coordinate, report_success, report_fail
            report_fail(pos=coordinate, obstacle=True)
            self.args.env.stats.add_invalid(obs=True)

    def set_use_radian(self, value: bool = True):
        """Set this sampler to use radian or not

        :param value: the value to set

        """
        self.use_radian = value

    def report_success(self, **kwargs):
        """Report to the sampler that the last sample was successfully. This function is
        sampler dependent.

        :param **kwargs: pass through to derived class

        """
        pass

    def report_fail(self, **kwargs):
        """Report to the sampler that the last sample was unsuccessful. This function is
        sampler dependent.

        :param **kwargs: pass through to derived class

        """
        pass

    def add_tree_node(self, **kwargs):
        """Report to the sampler about the last node that was added to the tree

        :param **kwargs: pass through to derived class

        """
        pass

    def add_sample_line(self, **kwargs):
        """Report to the sampler about the entire line that was sampled last time

        :param **kwargs: pass through to derived class

        """
        pass
