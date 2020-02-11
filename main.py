#!/usr/bin/env python
"""RRdT* Research

Usage:
  main.py (rrdt|rrt|birrt|informedrrt|prm|likelihood|nearby|mouse) <MAP>
          [options] [-v|-vv|-vvv]
  main.py (rrdt|rrt|birrt|informedrrt|prm|likelihood|nearby|mouse) <MAP>
          start [--] <start_x1,x2,..,xn> goal <goal_x1,x2,..,xn>
          [options] [-v|-vv|-vvv]
  main.py (-h | --help)
  main.py --version

Arguments:
  (rrdt|...|...)       Set the sampler to be used by the RRT*.
  <MAP>                  An image/xml file that represent the map.

General Options:
  -h --help              Show this screen.
  --version              Show version.
  -v --verbose           Display debug message.

Environment Options:
  -e --engine=ENGINE     Environment engine to use.
                         Could be either "image" or "klampt".
                         "image" engine uses an image as the map.
                         "klampt" engine is a multi-dof engine that are more
                           features rich.

Display Options:
  --always-refresh       Set if the display should refresh on every ticks.
  -s --scaling=SCALING   Set scaling for display (ratio will be maintained).
                         [default: 1.5]
  --hide-sampled-points  Do not display sampled point as red dot on screen.
  --no-display           Disable visualisation.

General Sampler Options:
  --epsilon=EPSILON      Set epsilon value.
                         [default: 10.0]
  --max-number-nodes=MAX_NODES
                         Set maximum number of nodes
                         [default: 20000]
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

  --keep-go             asdoaskdsioa
Disjoint Sampler Options:
  --proposal-dist=METHOD
                        Set the proposal distribution to use in the MCMC random
                        walk.
                        Supported methods are:
                        - original (from the original RRdT paper)
                        - dynamic-vonmises
                        - ray-casting
                        [default: dynamic-vonmises]
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
import os
import sys

from docopt import docopt

os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'  # hide pygame prompt

from helpers import MagicDict
from pygamevisualiser import VisualiserSwitcher, BaseEnvVisualiser, \
    BasePlannerVisualiser, KlamptPlannerVisualiser, KlamptEnvVisualiser


LOGGER = logging.getLogger()


def main():
    args = docopt(__doc__, version='RRT* Research v2.0')
    ########## Switch Visualiser to inherinet from ##########
    from pygamevisualiser import (VisualiserSwitcher, PygamePlannerVisualiser,
                                  PygameEnvVisualiser, BaseEnvVisualiser,
                                  KlamptEnvVisualiser, KlamptPlannerVisualiser,
                                  BasePlannerVisualiser)

    # deal with environment engine
    args['--engine'] = '' if args['--engine'] is None else args['--engine'].lower()

    if args['--engine'] not in ('image', 'klampt', ''):
        raise RuntimeError(f"Unrecognised value '{args['--engine']}' for engine option!")
    if args['--engine'] == '':
        # try to infer engine based on file extension
        _notice = "NOTE: no --engine option given. Infering as --engine={} based " \
                  "on file extension '{}'"
        if len(args['<MAP>'].split('.')) <= 1:
            raise RuntimeError("No --engine given and file has no extension. "
                               "Unable to infer engine.")
        _file_extension = args['<MAP>'].split('.')[-1]
        if _file_extension in ("xml", ):
            args['--engine'] = "klampt"
        elif _file_extension in ("jpg", "png", ):
            args['--engine'] = "image"
        else:
            raise RuntimeError("No engine given and unable to infer engine from "
                               "extension '{}'".format(_file_extension))
        LOGGER.info(_notice.format(args['--engine'], _file_extension))

    if args['--no-display']:
        # use passthrough visualiser
        VisualiserSwitcher.choose_env_vis(BaseEnvVisualiser)
        VisualiserSwitcher.choose_planner_vis(BasePlannerVisualiser)
    else:
        if args['--engine'] == 'image':
            # use pygame visualiser
            VisualiserSwitcher.choose_env_vis(PygameEnvVisualiser)
            VisualiserSwitcher.choose_planner_vis(PygamePlannerVisualiser)
        elif args['--engine'] == 'klampt':
            VisualiserSwitcher.choose_planner_vis(KlamptPlannerVisualiser)
            VisualiserSwitcher.choose_env_vis(KlamptEnvVisualiser)

    ########################################

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
    # args['--radius'] = float(args['--epsilon'])
    args['--radius'] = 1.2 * float(args['--epsilon'])

    LOGGER.debug("commandline args: {}".format(args))

    from planners.rrtPlanner import RRTPlanner
    planner_type = RRTPlanner  # default planner type
    if args['rrt']:
        from planners.randomPolicySampler import RandomPolicySampler
        sampler = RandomPolicySampler(random_method=args['--random-method'])
    elif args['birrt']:
        from planners.birrtPlanner import BiRRTSampler, BiRRTPlanner
        sampler = BiRRTSampler()
        planner_type = BiRRTPlanner
    elif args['rrdt']:
        from planners.rrdtPlanner import RRdTSampler, RRdTPlanner
        sampler = RRdTSampler(
            restart_when_merge=not args['--no-restart-when-merge'])
        planner_type = RRdTPlanner
    elif args['informedrrt']:
        from planners.informedrrtSampler import InformedRRTSampler
        sampler = InformedRRTSampler()
    elif args['prm']:
        from planners.prmPlanner import PRMSampler, PRMPlanner
        sampler = PRMSampler()
        planner_type = PRMPlanner
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

    planner_options = MagicDict({
        'keep_go_forth'             : args['--keep-go'],
        'showSampledPoint'          : not args['--hide-sampled-points'],
        'scaling'                   : float(args['--scaling']),
        'goalBias'                  : float(args['--goal-bias']),
        'image'                     : args['<MAP>'],
        'epsilon'                   : float(args['--epsilon']),
        'max_number_nodes'          : int(args['--max-number-nodes']),
        'radius'                    : float(args['--radius']),
        'goal_radius'               : 2 / 3 * float(args['--radius']),
        'ignore_step_size'          : args['--ignore-step-size'],
        'always_refresh'            : args['--always-refresh'],
        'sampler'                   : sampler,
        'rrdt_proposal_distribution': args['--proposal-dist'],
        'no_display'                : args['--no-display'],
        'engine'                    : args['--engine'],
        'planner_type'              : planner_type,
        'startPt'                   : args['<start_x1,x2,..,xn>'],
        'goalPt'                    : args['<goal_x1,x2,..,xn>']
    })

    # for klampt as it's in radian
    if args['--engine'] == 'klampt':
        args['--goal_radius'] = 0.001

    # quick and dirty fix for docopt not able to handle negative argument
    if planner_options['startPt']:
        if planner_options['startPt'].startswith("(") and planner_options['startPt'].endswith(")"):
            planner_options['startPt'] = planner_options['startPt'][1:-1]
    if planner_options['goalPt']:
       if planner_options['goalPt'].startswith("(") and planner_options['goalPt'].endswith(")"):
            planner_options['goalPt'] = planner_options['goalPt'][1:-1]

    # import csv
    #
    # import time
    # timestamp = time.strftime("%Y%m%d-%H%M%S", time.gmtime())
    # fname = f"result/{planner_type.__name__}_{timestamp}.csv"
    # with open(fname, 'w') as f:
    #     writer = csv.writer(f)
    #     planner_options['writer'] = writer
    #     planner_options['fname'] = fname
    #
    #     import env
    #     environment = env.Env(**planner_options)
    #     environment.run()
    import env
    environment = env.Env(**planner_options)
    environment.run()



if __name__ == '__main__':
    main()

    # # run if run from commandline
    # environment = main()
    # try:
    #     environment.run()
    # except Exception as e:
    #     LOGGER.error("==============================")
    #     LOGGER.exception("Exception occured: {}".format(e))
    #     LOGGER.error("==============================")
    #     LOGGER.error("Waiting to be exit...")
    #     try:
    #         while True:
    #             pass
    #             # environment.process_pygame_event()
    #     except KeyboardInterrupt:
    #         pass

    #
    #
    # import experimentrepeater as er
    # from experimentrepeater import repeat
    #
    #
    # # def good_function_to_run(args, **kwargs):
    # #     time_to_sleep, out = args
    # #     for i in range(3):
    # #         time.sleep(time_to_sleep)
    # #         kwargs["update_progress"](f"{i + 1}/3")
    # #         kwargs["log"](f"something bad happened at {i}")
    # #     kwargs["save_stats"](["YAY", out])
    #
    # VisualiserSwitcher.choose_env_vis(BaseEnvVisualiser)
    # VisualiserSwitcher.choose_planner_vis(BasePlannerVisualiser)
    #
    #
    # # VisualiserSwitcher.choose_planner_vis(KlamptPlannerVisualiser)
    # # VisualiserSwitcher.choose_env_vis(KlamptEnvVisualiser)
    #
    #
    #
    # er.run(
    #     job_name="job_name",
    #     num_processes=8,
    #     func=expr,
    #     args=(
    #         ['rrt']
    #         # 'rrt'
    #         # 'rrt'
    #         # 'rrt'
    #         # 'rrt'
    #     ),
    #     # args=(
    #     #     repeat([1, "out1"]) * 4,
    #     #     repeat([2, "out2"]) * 5,
    #     #     repeat([1, "out3"]) * 1,
    #     #     [1, "out4"],
    #     #     repeat([0.5, "out5"]) * 10,
    #     # ),
    # )
