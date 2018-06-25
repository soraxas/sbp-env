import sys
import pygame
import csv

import rrtstar
import subprocess

def main():

    maps = ['room1.png', 'maze1.png', 'maze2.png']
    policies = ['random', 'disjoint']
    # repeat for x number of times
    REPEAT_DIFFERENT_LOC = 20
    REPEAT_STATS = 20

    # Repeat the same in different map
    for map in maps:
        # Try different locations
        for _loc in range(REPEAT_DIFFERENT_LOC):
            start = get_random_free_space(map)
            goal = get_random_free_space(map)
            # Repeat for X amount of time for statistatical significant
            for _i in range(REPEAT_STATS):
                # Repeat the same for two different policies
                for policy in policies:
                    print('Map:{map} policy:{policy} loc:{loc_repeat} @{start},{goal} for {repeating}'.format(
                        map=map,
                        policy=policy,
                        loc_repeat=_loc,
                        start=start,
                        goal=goal,
                        repeating=_i))
                    args = ['python', 'benchmark_rrt_wrapper.py', policy, map,
                            "start", str(start[0]), str(start[1]),
                            "goal", str(goal[0]), str(goal[1]),
                            '--hide-sampled-points',
                            '--max-number-nodes=15000',
                            '--disable-pygame'
                            ]
                    subprocess.check_call(args)


def get_random_free_space(image):
    import random
    from rrtstar import Colour
    image = pygame.image.load(image)
    def collide(p):
        x = int(p[0])
        y = int(p[1])
        # make sure x and y is within image boundary
        if(x < 0 or x >= image.get_width() or
           y < 0 or y >= image.get_height()):
            return True
        color = image.get_at((x, y))
        return (color != pygame.Color(*Colour.white))
    while True:
        new_p = int(random.random()*image.get_width()),  int(random.random()*image.get_height())
        if not collide(new_p):
            return new_p


if __name__ == '__main__':
    main()
