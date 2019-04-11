import os
import random
import subprocess
import sys

import pygame

from env import Colour

CUR_PATH = os.path.dirname(sys.argv[0])
MAPS = ['maps/room1.png', 'maps/maze1.png', 'maps/noise.png']
POLICIES = ['informedrrt', 'multirrt']
REPEAT_DIFFERENT_LOC = 20
REPEAT_STATS = 20

sys.path.append(os.path.join(CUR_PATH, ".."))  # add top package to path

def main():
    # repeat for x number of times

    # Repeat the same in different map
    for test_map in MAPS:
        # Try different locations
        for _loc in range(REPEAT_DIFFERENT_LOC):
            start = get_random_free_space(test_map)
            goal = get_random_free_space(test_map)
            # Repeat for X amount of time for statistical significant
            for _i in range(REPEAT_STATS):
                # Repeat the same for two different policies
                for policy in POLICIES:
                    policyname = '{}'.format(policy)
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
                            '--policy-name={}'.format(policyname)
                            ]
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
