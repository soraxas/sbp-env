import numpy as np
import scipy as sp
import scipy.ndimage
from overrides import overrides

from ..samplers import likelihoodPolicySampler
from ..utils import planner_registry


# noinspection PyAttributeOutsideInit
class NearbyPolicySampler(likelihoodPolicySampler.LikelihoodPolicySampler):
    __doc__ = r"""This sampler uses the same mechanism as
    :class:`samplers.likelihoodPolicySampler.LikelihoodPolicySampler` to update 
    probability :math:`p`. However, this sampler prioritise configurations that are 
    closer to existing tree nodes (i.e. the tree structure).

    """ + r"""
    .. note::
        {only_work_with_2d_image}

    .. note::
        {currently_expr_for_research}

    """.format(
        **likelihoodPolicySampler.experimental_sampler_note
    )

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    @overrides
    def init(self, **kwargs):
        super().init(**kwargs)
        self.prob_vector = np.zeros(self.shape)

    @overrides
    def _report_fail_impl(self, x, y, **kwargs):
        if "obstacle" in kwargs:
            self.obst_vector[x][y] += 2
        elif not kwargs["free"]:
            self.obst_vector[x][y] += 2
        elif kwargs["free"]:
            if "weight" in kwargs:
                self.prob_vector[x][y] += kwargs["weight"]
            else:
                self.prob_vector[x][y] += 1
            self.obst_vector[x][y] = 1

            #########################################################

            sigma_y = 1.0
            sigma_x = 1.0
            sigma = [sigma_y, sigma_x]
            if self.sampleCount % 20 == 0:
                pass
                self.prob_vector_normalized = np.copy(self.tree_vector)
                self.prob_vector_normalized = sp.ndimage.filters.gaussian_filter(
                    self.prob_vector_normalized, (8.0, 8.0), mode="reflect"
                )
                tree_vector_normalized = np.copy(self.tree_vector)
                tree_vector_normalized = sp.ndimage.filters.gaussian_filter(
                    self.tree_vector, (1.0, 1.0), mode="reflect"
                )

                self.prob_vector_normalized -= tree_vector_normalized
                self.prob_vector_normalized = np.clip(
                    self.prob_vector_normalized, 0, None
                )

                self.prob_vector_normalized /= self.prob_vector_normalized.sum()
            self.sampleCount += 1


# start register
sampler_id = "nearby_sampler"

planner_registry.register_sampler(
    sampler_id,
    sampler_class=likelihoodPolicySampler.LikelihoodPolicySampler,
    visualise_pygame_paint=likelihoodPolicySampler.pygame_likelihood_sampler_paint,
    visualise_pygame_paint_init=likelihoodPolicySampler.pygame_likelihood_sampler_paint_init,
)
# finish register
