"""
This wrapper is to make it so that the module will output directly to csv formatted
file along with a screenshot when it initially found a solution, and one when it termiinated
"""
import sys
import os
import pygame
from memory_profiler import memory_usage
from timeit import default_timer
import csv
import time

LOG_EVERY_X_SAMPLES = 25
BENCHMARK_DIR_NAME = 'benchmark'


def main():
    # get rrt instance
    import main
    rrt = main.main()

    if not os.path.exists(BENCHMARK_DIR_NAME):
        os.makedirs(BENCHMARK_DIR_NAME)

    timestamp = time.strftime("%Y%m%d-%H%M%S", time.gmtime())
    directoryname = "map={map}_start={start}_goal={goal}".format(
                    map=sys.argv[2],
                    start='-'.join((str(x) for x in rrt.startPt.pos)),
                    goal='-'.join((str(x) for x in rrt.goalPt.pos)),
                )
    directoryname = os.path.join(BENCHMARK_DIR_NAME, directoryname)
    filename = "policy={policy}_timestamp={timestamp}".format(
                    policy=sys.argv[1],
                    timestamp=timestamp
                )
    # save inside a directory
    if not os.path.exists(directoryname):
        os.makedirs(directoryname)
    filename = os.path.join(directoryname, filename)

    found_solution = False
    count = 0

    with open('{}.csv'.format(filename), 'wt') as f:
        writer = csv.writer(f)
        # write basic information
        writer.writerow(('Map', 'Start pt (x,y)', 'End pt (x,y)', 'epsilon', 'goal bias', 'max nodes'))
        writer.writerow((sys.argv[2], rrt.startPt.pos, rrt.goalPt.pos, rrt.EPSILON, rrt.goalBias, rrt.NUMNODES))
        writer.writerow([])

        writer.writerow(('Num nodes', 'time(sec)', 'mem(mb)', 'inv.samples(con)', 'inv.samples(obs)', 'cost'))
        start_time = default_timer()

        def take_screenshot(term=False):
            rrt.pygame_show()
            rrt.update_screen(update_all=True)
            if term:
                # terminating
                pygame.image.save(rrt.window,'{}_term.jpg'.format(filename))
            else:
                pygame.image.save(rrt.window,'{}.jpg'.format(filename))
            rrt.pygame_hide()

        def log_performance():
            msg = rrt.stats.valid_sample, default_timer() - start_time, memory_usage()[0], rrt.stats.invalid_samples_connections, rrt.stats.invalid_samples_obstacles, rrt.c_max
            writer.writerow(msg)
            f.flush()

        while rrt.stats.valid_sample < rrt.NUMNODES:
            if rrt.stats.valid_sample >= count:
                count += LOG_EVERY_X_SAMPLES
                log_performance()

            if not found_solution and rrt.c_max != float('inf'):
                log_performance()
                found_solution = True
                take_screenshot()
            rrt.run_once()

        # take another screenshot and log when terminates
        log_performance()
        take_screenshot(term=True)


if __name__ == '__main__':
    main()
