import os
import random
import subprocess
import sys

import pygame

from env import Colour

CUR_PATH = os.path.dirname(sys.argv[0])
MAPS = ['maps/room1.png', 'maps/maze1.png', 'maps/noise.png']
MAPS = ['maps/noise.png']
MAPS = ['maps/room1.png']
#MAPS = ['maps/maze1.png']
POLICIES = ['original', 'dynamic-vonmises', 'rrt', 'birrt', 'informedrrt', 'prm']
REPEAT_DIFFERENT_LOC = 1
REPEAT_STATS = 5

sys.path.append(os.path.join(CUR_PATH, ".."))  # add top package to path

def main():
    # repeat for x number of times

    # Repeat the same in different map
    for test_map in MAPS:
        # Try different locations
        for _loc in range(REPEAT_DIFFERENT_LOC):
            if test_map == 'maps/maze1.png':
                start = (36, 26)
                goal = (297, 311)
            elif test_map == 'maps/room1.png':
                start = (99, 205)
                goal = (474, 359)
            elif test_map == 'maps/noise.png':
                start = (5, 27)
                goal = (431, 194)
            else:
                raise Exception("NOT SET")
            # start = get_random_free_space(test_map)
            # goal = get_random_free_space(test_map)
            # Repeat for X amount of time for statistical significant
            for _i in range(REPEAT_STATS):
                # Repeat the same for two different policies
                for policy in POLICIES:
                    policyname = '{}'.format(policy)
                    op = ''
                    prop = ['original', 'dynamic-vonmises']
                    if policy in prop:
                        op = '--proposal-dist={}'.format(policy)
                        policyname = 'rrdt-{}'.format(policy)
                        policy = 'rrdt'
                    else:
                        op = ''

                    print('Map:{map} policy:{policy} loc:{loc_repeat} @{start},{goal} for {repeating}'.format(
                        map=test_map,
                        policy=policyname,
                        loc_repeat=_loc,
                        start=start,
                        goal=goal,
                        repeating=_i))
                    args = ['python', os.path.join(CUR_PATH, 'benchmark_rrt_wrapper.py'),
                            policy, test_map,
                            "start", str(start[0]), str(start[1]),
                            "goal", str(goal[0]), str(goal[1]),
                            '--hide-sampled-points',
                            '--max-number-nodes=10000',
                            '--disable-pygame',
                            '--epsilon=7',
                            '--policy-name={}'.format(policyname),
                            op
                            ]
                    # remove empty args
                    args = [a for a in args if a]
                    subprocess.check_call(args)


def get_random_free_space(image):
    image = pygame.image.load(image)

    def collide(p):
        x = int(p[0])
        y = int(p[1])
        # make sure x and y is within image boundary
        if (x < 0 or x >= image.get_width() or
                y < 0 or y >= image.get_height()):
            return True
        color = image.get_at((x, y))
        return color != pygame.Color(*Colour.white)

    while True:
        new_p = int(random.random() * image.get_width()), int(random.random() * image.get_height())
        if not collide(new_p):
            return new_p


if __name__ == '__main__':
    main()
