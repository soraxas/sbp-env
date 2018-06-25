#!/usr/bin/env python
"""RRT* Research

Usage:
  main.py (random|disjoint|particle|likelihood|nearby|mouse) <MAP> [options]
  main.py (random|disjoint|particle|likelihood|nearby|mouse) <MAP>
          start <sx> <sy> goal <gx> <gy> [options]
  main.py (-h | --help)
  main.py --version

Arguments:
  (random|...|...)       Set the sampler to be used by the RRT*.
  <MAP>                  An image file that represent the map.

General Options:
  -h --help              Show this screen.
  --version              Show version.
  -v, --verbose          Display debug message.

Display Options:
  --always-refresh       Set if the display should refresh on every ticks.
  --scaling=SCALING      Set scaling for display (ratio will be maintained).
                         [default: 4]
  --hide-sampled-points  Do not display sampled point as red dot on screen.
  --disable-pygame       Disable pygame display (to enhance performance).

Sampler Options:
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

Likelihood/Nearby Sampler Options:
  --prob-block-size=SIZE
                        Set the dimension of the discretized block.
                        [default: 5]
"""
from docopt import docopt
import logging
import sys

import rrtstar

LOGGER = logging.getLogger()

def main():
    args = docopt(__doc__, version='RRT* Research v1.0')

    if args['--verbose']:
        LOGGER.setLevel(logging.DEBUG)
    else:
        LOGGER.setLevel(logging.ERROR)
    # INFO includes only loading
    # DEBUG includes all outputs
    ch = logging.StreamHandler(sys.stdout)
    ch.setFormatter(logging.Formatter('%(message)s'))
    # ch.setFormatter(logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s'))
    LOGGER.addHandler(ch)

    LOGGER.debug("commandline args: {}".format(args))

    if args['random']:
        from randomPolicySampler import RandomPolicySampler
        sampler = RandomPolicySampler()
    elif args['disjoint']:
        from disjointTree import DisjointParticleFilterSampler
        sampler = DisjointParticleFilterSampler()
    elif args['particle']:
        from particleFilterSampler import ParticleFilterSampler
        sampler = ParticleFilterSampler()
    elif args['likelihood']:
        from likelihoodPolicySampler import LikelihoodPolicySampler
        sampler = LikelihoodPolicySampler(prob_block_size=int(args['--prob-block-size']))
    elif args['nearby']:
        from nearbyPolicySampler import NearbyPolicySampler
        sampler = NearbyPolicySampler(prob_block_size=int(args['--prob-block-size']))
    elif args['mouse']:
        from mouseSampler import MouseSampler
        sampler = MouseSampler()

    rrt_options = {
        'showSampledPoint' : not args['--hide-sampled-points'],
        'scaling' : int(args['--scaling']),
        'sampler' : sampler,
        'goalBias' : float(args['--goal-bias']),
        'image' : args['<MAP>'],
        'epsilon' : float(args['--epsilon']),
        'max_number_nodes' : int(args['--max-number-nodes']),
        'radius' : float(args['--radius']),
        'ignore_step_size' : args['--ignore-step-size'],
        'always_refresh' : args['--always-refresh'],
        'enable_pygame' : not args['--disable-pygame'],
    }

    if args['start'] and args['goal']:
        rrt_options.update({'startPt' : (float(args['<sx>']), float(args['<sy>'])),
                            'goalPt' : (float(args['<gx>']), float(args['<gy>']))
                            })

    rrt = rrtstar.RRT(**rrt_options)
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
        rrt.wait_for_exit()
