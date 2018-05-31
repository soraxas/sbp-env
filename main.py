import rrtstar
from randomPolicySampler import RandomPolicySampler
from likelihoodPolicySampler import LikelihoodPolicySampler
from nearbyPolicySampler import NearbyPolicySampler

# if python says run, then we should run
if __name__ == '__main__':
    epsilon = 7.0
    # epsilon = 7.0 * 10
    max_number_nodes = 3000
    goal_radius = 15
    prob_block_size = 5
    SCALING = 4

    sampler = NearbyPolicySampler(prob_block_size=prob_block_size)
    sampler = RandomPolicySampler()
    sampler = LikelihoodPolicySampler(prob_block_size=prob_block_size)

    CHECK_ENTIRE_PATH = False

    rrt = rrtstar.RRT(
        showSampledPoint=True,
        scaling=SCALING,
        sampler=sampler,
        goalBias=False,
        check_entire_path=CHECK_ENTIRE_PATH,
        image='map.png',
        epsilon=epsilon,
        max_number_nodes=max_number_nodes,
        radius=goal_radius)
    rrt.run()
    # running = True
    # while running:
    #     for event in pygame.event.get():
    #         if event.type == pygame.QUIT:
    #             running = False
# TODO remember to FIXME
