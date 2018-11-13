from helpers import MagicDict

class Sampler:
    """
    Base sampler that defines each unique methods that some
    sampler uses but not all. This sampler does nothing with its own.
    """

    def init(self, *argv, **kwargs):
        self.args = MagicDict(kwargs)
        self.start_pos = kwargs['startPt'].pos
        self.goal_pos = kwargs['goalPt'].pos

    def get_next_pos(self, *argv, **kwargs):
        pass

    def get_valid_next_pos(self):
        """Loop until we find a valid next node"""
        while True:
            coordinate, report_success, report_fail = self.get_next_pos()
            self.args.env.stats.add_sampled_node(coordinate)
            if not self.args.env.collides(coordinate):
                return coordinate, report_success, report_fail
            report_fail(pos=coordinate, obstacle=True)
            self.args.env.stats.add_invalid(obs=True)

    def report_success(self, *argv, **kwargs):
        pass

    def report_fail(self, *argv, **kwargs):
        pass

    def add_tree_node(self, *argv, **kwargs):
        pass

    def add_sample_line(self, *argv, **kwargs):
        pass

    def paint(self, *argv, **kwargs):
        pass
