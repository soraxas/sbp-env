import numpy as np

from utils.helpers import MagicDict
from visualiser import VisualiserSwitcher


class Planner(VisualiserSwitcher.planner_clname):
    """
    Abstract base planner that does nothing by itself.
    """

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.args = MagicDict(kwargs)
        self.c_max = float("inf")

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
