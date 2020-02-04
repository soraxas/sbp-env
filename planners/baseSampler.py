from helpers import MagicDict
from pygamevisualiser import VisualiserSwitcher
import numpy as np

class Planner(VisualiserSwitcher.planner_clname):

    def get_solution_path(self):
        if self.c_max == float('inf'):
            return []
        path = [self.goalPt]
        nn = self.goalPt.parent
        # while nn != self.startPt:
        while not np.all(np.isclose(nn.pos, self.startPt.pos)):
            if nn == nn.parent:
                raise RuntimeError(f"nn = nn.parent?\n{nn}\n{nn.parent}")
            path.append(nn)
            nn = nn.parent
        return reversed(path)


class Sampler(VisualiserSwitcher.planner_clname):
    """
    Base sampler that defines each unique methods that some
    sampler uses but not all. This sampler does nothing with its own.
    """

    def init(self, use_radian=False, *argv, **kwargs):
        self.args = MagicDict(kwargs)
        self.start_pos = kwargs['startPt'].pos
        self.goal_pos = kwargs['goalPt'].pos
        self.use_radian = use_radian

        super().init(*argv, **kwargs)

    def get_next_pos(self, *argv, **kwargs):
        pass

    def get_valid_next_pos(self):
        """Loop until we find a valid next node"""
        while True:
            coordinate, report_success, report_fail = self.get_next_pos()
            self.args.env.stats.add_sampled_node(coordinate)
            if self.args.env.cc.feasible(coordinate):
                return coordinate, report_success, report_fail
            report_fail(pos=coordinate, obstacle=True)
            self.args.env.stats.add_invalid(obs=True)

    def set_use_radian(self, value=True):
        self.use_radian = value

    def report_success(self, *argv, **kwargs):
        pass

    def report_fail(self, *argv, **kwargs):
        pass

    def add_tree_node(self, *argv, **kwargs):
        pass

    def add_sample_line(self, *argv, **kwargs):
        pass
