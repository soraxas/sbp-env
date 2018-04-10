import rrtstar
from randomPolicySampler import RandomPolicySampler
from likelihoodPolicySampler import LikelihoodPolicySampler

# if python says run, then we should run
if __name__ == '__main__':
    epsilon = 7.0
    max_number_nodes = 2000
    radius = 15

    # sampler = RandomPolicySampler()
    sampler = LikelihoodPolicySampler(prob_block_size=8)

    CHECK_ENTIRE_PATH = False

    rrt = rrtstar.RRT(sampler=sampler, goalBias=False, check_entire_path=CHECK_ENTIRE_PATH, image='map.png', epsilon=epsilon, max_number_nodes=max_number_nodes, radius=radius)
    rrt.run()
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
