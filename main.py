import rrtstar
from randomPolicySampler import RandomPolicySampler
from likelihoodPolicySampler import LikelihoodPolicySampler
from particleFilterSampler import ParticleFilterSampler
from nearbyPolicySampler import NearbyPolicySampler

# if python says run, then we should run
if __name__ == '__main__':
    epsilon = 10.0
    max_number_nodes = 3000
    goal_radius = 15
    prob_block_size = 5
    SCALING = 4

    sampler = NearbyPolicySampler(prob_block_size=prob_block_size)
    sampler = LikelihoodPolicySampler(prob_block_size=prob_block_size)
    sampler = RandomPolicySampler()
    sampler = ParticleFilterSampler()

    rrt = rrtstar.RRT(
        showSampledPoint=True,
        scaling=SCALING,
        sampler=sampler,
        goalBias=False,
        image='map.png',
        epsilon=epsilon,
        max_number_nodes=max_number_nodes,
        radius=goal_radius
        )
    rrt.run()
