
import numpy as np
import scipy as sp
import scipy.ndimage
from overrides import overrides

from planners.likelihoodPolicySampler import LikelihoodPolicySampler


class NearbyPolicySampler(LikelihoodPolicySampler):

    @overrides
    def init(self, **kwargs):
        super().init(**kwargs)
        self.prob_vector = np.zeros(self.shape)
        # self.prob_vector *= 2 # IMPORTANT because we are using log2


    @overrides
    def report_fail(self, **kwargs):
        p = kwargs['pos']
        if p is None:
            return
        try:
            p = p.pos
        except AttributeError as e:
            pass
        if 'alreadyDividedByProbBlockSize' not in kwargs:
            x = int(p[0]/self.PROB_BLOCK_SIZE)
            y = int(p[1]/self.PROB_BLOCK_SIZE)
        else:
            x = p[0]
            y = p[1]
        if x < 0 or x >= self.prob_vector.shape[0] or \
            y < 0 or y >= self.prob_vector.shape[1]:
                return

        # print(p)
        # exit()
        # ^ setting the right coor ^
        ###############################
        factor = 1.5

        if 'obstacle' in kwargs:
            self.obst_vector[x][y] += 2
        elif not kwargs['free']:
            self.obst_vector[x][y] += 2
            # self.prob_vector[x][y] -= (100-self.prob_vector[x][y])*0.1
            # if self.prob_vector[x][y] < 5:
                # self.prob_vector[x][y] = 5
            # print(p)
        elif kwargs['free']:
            if 'weight' in kwargs:
                self.prob_vector[x][y] += kwargs['weight']
            else:
                self.prob_vector[x][y] += 1
                # self.prob_vector[x][y] = 10
            self.obst_vector[x][y] = 1

    #########################################################

            sigma_y = 1.0
            sigma_x = 1.0
            sigma = [sigma_y, sigma_x]
            if self.sampleCount % 20 == 0:
                pass
                self.prob_vector_normalized = np.copy(self.tree_vector)
                self.prob_vector_normalized = sp.ndimage.filters.gaussian_filter(self.prob_vector_normalized, (8.0,8.0), mode='reflect')
                tree_vector_normalized = np.copy(self.tree_vector)
                tree_vector_normalized = sp.ndimage.filters.gaussian_filter(self.tree_vector, (1.0,1.0), mode='reflect')

                self.prob_vector_normalized -= tree_vector_normalized
                self.prob_vector_normalized = np.clip(self.prob_vector_normalized, 0, None)

            #     self.prob_vector_normalized = np.copy(self.prob_vector)
            #     # self.prob_vector_normalized = np.copy(np.log2(self.prob_vector))
            #     # self.prob_vector_normalized = np.f.prob_vector[x][y] -= (100-self.prob_vector[x][y])*0.1
            # # if self.prob_vector[x][y] < 5:
            #     # self.prob_vector[copy(self.prob_vector)
                # tree_vector_normalized = np.copy(self.tree_vector**2)
            #     tree_vector_normalized = sp.ndimage.filters.gaussian_filter(tree_vector_normalized, (2.0,2.0), mode='reflect')
            #     # self.prob_vector_normalized = tree_vector_normalized
                # self.prob_vector_normalized *= (1/self.obst_vector)
                # self.prob_vector_normalized = sp.ndimage.filters.gaussian_filter(self.prob_vector_normalized, sigma, mode='reflect')
                # self.prob_vector_normalized *= (1/tree_vector_normalized * 5)
            #     # self.prob_vector_normalized *= (1/self.tree_vector * 1.5)
                self.prob_vector_normalized /= self.prob_vector_normalized.sum()
            # prob_vector_normalized = np.copy(self.tree_vector)
            self.sampleCount += 1
