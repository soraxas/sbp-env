#!/usr/bin/env python
"""RRdT* Research

Usage:
  main.py (rrdt|rrt|birrt|informedrrt|multirrt|prm|particle|likelihood|nearby|mouse) <MAP>
          [options] [-v|-vv|-vvv]
  main.py (rrdt|rrt|birrt|informedrrt|multirrt|prm|particle|likelihood|nearby|mouse) <MAP>
          start <sx> <sy> goal <gx> <gy> [options] [-v|-vv|-vvv]
  main.py (-h | --help)
  main.py --version

Arguments:
  (rrdt|...|...)       Set the sampler to be used by the RRT*.
  <MAP>                  An image file that represent the map.

General Options:
  -h --help              Show this screen.
  --version              Show version.
  -v --verbose           Display debug message.

Display Options:
  --always-refresh       Set if the display should refresh on every ticks.
  -s --scaling=SCALING   Set scaling for display (ratio will be maintained).
                         [default: 1.5]
  --hide-sampled-points  Do not display sampled point as red dot on screen.
  --disable-pygame       Disable pygame display (to enhance performance).

General Sampler Options:
  --epsilon=EPSILON      Set epsilon value.
                         [default: 10.0]
  --radius=RADIUS        Set radius that will connect two nodes together.
                         [default: 15]
  --max-number-nodes=MAX_NODES
                         Set maximum number of nodes
                         [default: 30000]
  --ignore-step-size     Ignore step size (i.e. epsilon) when sampling.
  --goal-bias=BIAS       Probability of biasing goal position.
                         [default: 0.02]

Random Sampler Options:
  --random-method=METHOD
                        Set a random method used to generate the random
                        numbers. This enables the use of different
                        quasi-random generators.
                        Supported methods are:
                        - pseudo_random (random from numpy)
                        - saltelli (Saltelli's extension of Sobol sequence)
                        - sobol_sequence (Sobol sequence generator)
                        - latin_hypercube (Latin hypercube sampling)
                        - finite_differences (Derivative-based global
                                              sensitivity measure)
                        - fast (Fourier Amplitude Sensitivity Test)
                        [default: pseudo_random]

Disjoint Sampler Options:
  --no-restart-when-merge
                        This flag denotes if the local sampler from disjoint-
                        tree sampler should restart at a new location when
                        the disjointed tree branches jointed to an existing
                        one. The default behaviour is restart as soon as merged
                        to another existing tree (to encourage exploration).
                        When this flag is set, it will remain until its energy
                        is exhausted.

Likelihood/Nearby Sampler Options:
  --prob-block-size=SIZE
                        Set the dimension of the discretized block.
                        [default: 5]
"""
import logging
import sys

from docopt import docopt

import env
from helpers import MagicDict

LOGGER = logging.getLogger()


def main():
    args = docopt(__doc__, version='RRT* Research v1.0')

    if args['--verbose'] > 2:
        LOGGER.setLevel(logging.DEBUG)
    elif args['--verbose'] > 1:
        LOGGER.setLevel(logging.INFO)
    elif args['--verbose'] > 0:
        LOGGER.setLevel(logging.WARNING)
    else:
        LOGGER.setLevel(logging.ERROR)
    # INFO includes only loading
    # DEBUG includes all outputs
    ch = logging.StreamHandler(sys.stdout)
    ch.setFormatter(logging.Formatter('%(message)s'))
    # ch.setFormatter(logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s'))
    LOGGER.addHandler(ch)

    LOGGER.debug("commandline args: {}".format(args))

    from planners.rrtPlanner import RRTPlanner
    planner_type = RRTPlanner # default planner type
    if args['rrt']:
        from planners.randomPolicySampler import RandomPolicySampler
        sampler = RandomPolicySampler(random_method=args['--random-method'])
    elif args['birrt']:
        from planners.birrtPlanner import BiRRTSampler, BiRRTPlanner
        sampler = BiRRTSampler()
        planner_type = BiRRTPlanner
    elif args['multirrt']:
        from planners.multirrtPlanner import MultiRRTSampler, MultiRRTPlanner
        sampler = MultiRRTSampler()
        planner_type = MultiRRTPlanner
    elif args['rrdt']:
        from planners.rrdtPlanner import RRdTSampler, RRdTPlanner
        sampler = RRdTSampler(
            restart_when_merge=not args['--no-restart-when-merge'])
        planner_type = RRdTPlanner
    elif args['informedrrt']:
        from planners.informedrrtPlanner import InformedRRTPlanner, InformedRRTSampler
        sampler = InformedRRTSampler()
        planner_type = InformedRRTPlanner
    elif args['prm']:
        from planners.prmPlanner import PRMSampler, PRMPlanner
        sampler = PRMSampler()
        planner_type = PRMPlanner
    elif args['particle']:
        from planners.particleFilterSampler import ParticleFilterSampler
        sampler = ParticleFilterSampler()
    elif args['likelihood']:
        from planners.likelihoodPolicySampler import LikelihoodPolicySampler
        sampler = LikelihoodPolicySampler(
            prob_block_size=int(args['--prob-block-size']))
    elif args['nearby']:
        from planners.nearbyPolicySampler import NearbyPolicySampler
        sampler = NearbyPolicySampler(
            prob_block_size=int(args['--prob-block-size']))
    elif args['mouse']:
        from planners.mouseSampler import MouseSampler
        sampler = MouseSampler()

    rrt_options = MagicDict({
        'showSampledPoint':
        not args['--hide-sampled-points'],
        'scaling':
        float(args['--scaling']),
        'goalBias':
        float(args['--goal-bias']),
        'image':
        args['<MAP>'],
        'epsilon':
        float(args['--epsilon']),
        'max_number_nodes':
        int(args['--max-number-nodes']),
        'radius':
        float(args['--radius']),
        'goal_radius':
        2 / 3 * float(args['--radius']),
        'ignore_step_size':
        args['--ignore-step-size'],
        'always_refresh':
        args['--always-refresh'],
        'enable_pygame':
        not args['--disable-pygame'],
        'sampler':
        sampler,
    })
    rrtplanner = planner_type(**rrt_options)
    rrt_options.update({
        'planner': rrtplanner,
    })

    if args['start'] and args['goal']:
        rrt_options.update({
            'startPt': (float(args['<sx>']), float(args['<sy>'])),
            'goalPt': (float(args['<gx>']), float(args['<gy>']))
        })

    rrt = env.Env(**rrt_options)
    return rrt


if __name__ == '__main__':
    # run if run from commandline
    rrt = main()
    try:
        rrt.run()
    except Exception as e:
        LOGGER.error("==============================")
        LOGGER.exception("Exception occured: {}".format(e))
        LOGGER.error("==============================")
        LOGGER.error("Waiting to be exit...")
        try:
            rrt.wait_for_exit()
        except KeyboardInterrupt:
            pass
