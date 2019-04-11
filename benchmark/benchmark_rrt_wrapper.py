"""
This wrapper is to make it so that the module will output directly to csv formatted
file along with a screenshot when it initially found a solution, and one when it termiinated
"""
import csv
import os
import sys
import time
from timeit import default_timer

import pygame
from memory_profiler import memory_usage

CUR_PATH = os.path.dirname(sys.argv[0])
LOG_EVERY_X_SAMPLES = 25
BENCHMARK_DIR_NAME = os.path.join(CUR_PATH, '..', 'benchmark_output')
sys.path.append(os.path.join(CUR_PATH, '..'))  # add top package to path


def main():
    import main
    policyname = None
    for i in range(len(sys.argv)):
        if sys.argv[i].startswith('--policy-name='):
            policyname = sys.argv.pop(i)
            policyname = policyname.replace('--policy-name=', '')
    if policyname is None:
        # default to policy's name
        policyname = sys.argv[1]

    # get rrt instance
    env = main.main()

    if not os.path.exists(BENCHMARK_DIR_NAME):
        os.makedirs(BENCHMARK_DIR_NAME)

    timestamp = time.strftime("%Y%m%d-%H%M%S", time.gmtime())
    directoryname = "map={map}_start={start}_goal={goal}".format(
                    map=sys.argv[2],
                    start='-'.join((str(x) for x in env.startPt.pos)),
                    goal='-'.join((str(x) for x in env.goalPt.pos)),
                )
    directoryname = os.path.join(BENCHMARK_DIR_NAME, directoryname)
    filename = "policy={policy}_timestamp={timestamp}".format(
                    policy=policyname,
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
        writer.writerow((sys.argv[2], env.startPt.pos, env.goalPt.pos, env.args.epsilon, env.args.goalBias, env.args.max_number_nodes))
        writer.writerow([])

        writer.writerow(('Num nodes', 'time(sec)', 'mem(mb)', 'inv.samples(con)', 'inv.samples(obs)', 'cost'))
        start_time = default_timer()

        def take_screenshot(term=False):
            env.pygame_show()
            env.args.enable_pygame = True
            env.update_screen(update_all=True)
            if term:
                # terminating
                pygame.image.save(env.window,'{}_term.jpg'.format(filename))
            else:
                pygame.image.save(env.window,'{}.jpg'.format(filename))
            env.args.enable_pygame = False
            env.pygame_hide()

        def log_performance():
            msg = env.stats.valid_sample, default_timer() - start_time, memory_usage()[0], env.stats.invalid_samples_connections, env.stats.invalid_samples_obstacles, env.args.planner.c_max
            writer.writerow(msg)
            f.flush()

        while env.stats.valid_sample < env.args.max_number_nodes:
            if env.stats.valid_sample >= count:
                count += LOG_EVERY_X_SAMPLES
                log_performance()

            if not found_solution and env.args.planner.c_max != float('inf'):
                log_performance()
                found_solution = True
                take_screenshot()
            env.planner.run_once()

        # take another screenshot and log when terminates
        log_performance()
        take_screenshot(term=True)


if __name__ == '__main__':
    main()
