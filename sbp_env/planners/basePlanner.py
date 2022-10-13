from abc import ABC

import numpy as np

from ..utils.common import PlanningOptions
from ..visualiser import VisualiserSwitcher


class Planner(ABC):
    """
    Abstract base planner that does nothing by itself.
    """

    def __init__(self, args: PlanningOptions):
        super().__init__()
        self.args = args
        self.c_max = float("inf")
        self.visualiser = VisualiserSwitcher.planner_clname(
            planner_instance=self, planner_data_pack=self.args.planner_data_pack
        )

    def init(self, env, args):
        # self.args = args
        pass

    def get_solution_path(self):
        """Retrieve the current solution path via transversing each connected node's
        parent, in reverse, starting from the goal node.

        :return: a list of node that represent the solution path
        """
        if self.c_max == float("inf"):
            return []
        path = [self.goal_pt]
        nn = self.goal_pt.parent
        while not np.all(np.isclose(nn.pos, self.start_pt.pos)):
            if nn == nn.parent:
                raise RuntimeError(f"nn = nn.parent?\n{nn}\n{nn.parent}")
            path.append(nn)
            nn = nn.parent
        return reversed(path)
