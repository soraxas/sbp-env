#!/usr/bin/env python
"""SBP-Bench

Usage:
  main.py ({all_available_planners}) <MAP>
          [options] [-v|-vv|-vvv]
  main.py ({all_available_planners}) <MAP>
          start [--] <start_x1,x2,..,xn> goal <goal_x1,x2,..,xn>
          [options] [-v|-vv|-vvv]
  main.py (-h | --help)
  main.py --version

Arguments:
  (rrt|...|...)          Set the sampler to be used by the RRT*.
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

General Planner Options:
  --radius=RADIUS        Set radius for node connections.
                         [default: 12.0]
  --epsilon=EPSILON      Set epsilon value.
                         [default: 10.0]
  --max-number-nodes=MAX_NODES
                         Set maximum number of nodes
                         [default: 10000]
  --ignore-step-size     Ignore step size (i.e. epsilon) when sampling.
  --goal-bias=BIAS       Probability of biasing goal position.
                         [default: 0.02]
  --skip-optimality      Skip optimality guarantee (i.e. skip performing rewiring)

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
import re
import sys

from docopt import docopt

import planners
from utils import planner_registry

assert planners

LOGGER = logging.getLogger()

__doc__ = __doc__.format(
    all_available_planners="|".join(
        sorted(
            (planner.name for planner in planner_registry.PLANNERS.values()),
            reverse=True,
        )
    )
)


def main(map_fname=None, start=None, goal=None, sampler=None):
    args = docopt(__doc__, version="SBP-Bench Research v2.0")

    ########## Switch Visualiser to inherinet from ##########
    from pygamevisualiser import VisualiserSwitcher

    # allow the map filename, start and goal point to be override.
    if map_fname is not None:
        args["<MAP>"] = map_fname
    if start is not None:
        args["<start_x1,x2,..,xn>"] = start
    if goal is not None:
        args["<goal_x1,x2,..,xn>"] = goal

    # setup environment engine
    args["--engine"] = "" if args["--engine"] is None else args["--engine"].lower()
    if args["--engine"] not in ("image", "klampt", ""):
        raise RuntimeError(
            f"Unrecognised value '{args['--engine']}' for engine option!"
        )
    if args["--engine"] == "":
        # try to infer engine based on file extension
        _notice = (
            "NOTE: no --engine option given. Inferring as --engine={} based "
            "on file extension '{}'"
        )
        if len(args["<MAP>"].split(".")) <= 1:
            raise RuntimeError(
                "No --engine given and file has no extension. "
                "Unable to infer engine."
            )
        _file_extension = args["<MAP>"].split(".")[-1]
        if _file_extension in ("xml",):
            args["--engine"] = "klampt"
        elif _file_extension in ("jpg", "png",):
            args["--engine"] = "image"
        else:
            raise RuntimeError(
                "No engine given and unable to infer engine from "
                "extension '{}'".format(_file_extension)
            )
        LOGGER.info(_notice.format(args["--engine"], _file_extension))

    if args["--no-display"]:
        # use pass-through visualiser
        VisualiserSwitcher.choose_visualiser("base")
    else:
        if args["--engine"] == "image":
            # use pygame visualiser
            VisualiserSwitcher.choose_visualiser("pygame")
        elif args["--engine"] == "klampt":
            VisualiserSwitcher.choose_visualiser("klampt")

    ########################################

    if args["--verbose"] > 2:
        LOGGER.setLevel(logging.DEBUG)
    elif args["--verbose"] > 1:
        LOGGER.setLevel(logging.INFO)
    elif args["--verbose"] > 0:
        LOGGER.setLevel(logging.WARNING)
    else:
        LOGGER.setLevel(logging.ERROR)
    # INFO includes only loading msgs
    # DEBUG includes all outputs
    ch = logging.StreamHandler(sys.stdout)
    ch.setFormatter(logging.Formatter("%(message)s"))
    LOGGER.addHandler(ch)
    LOGGER.debug("commandline args: {}".format(args))

    # find out which planner is chosen
    planner_canidates = [
        a
        for a in args
        if (not a.startswith("--") and args[a] is True and a not in ("start", "goal"))
    ]
    assert (
        len(planner_canidates) == 1
    ), f"Planner to use '{planner_canidates}' has length {len(planner_canidates)}"
    planner_to_use = planner_canidates[0]
    print(planner_to_use)

    planner_data_pack = planner_registry.PLANNERS[planner_to_use]

    planner_type = planner_data_pack.planner_class
    sampler_data_pack = planner_registry.SAMPLERS[planner_data_pack.sampler_id]
    sampler = sampler_data_pack.sampler_class(
        sampler_data_pack=sampler_data_pack,
        # sanitise keyword arguments by removing prefix -- and replacing - as _
        **{re.sub(r"^--", "", k).replace("-", "_"): v for (k, v) in args.items()},
    )

    planner_options = dict(
        planner_data_pack=planner_data_pack,
        skip_optimality=args["--skip-optimality"],
        showSampledPoint=not args["--hide-sampled-points"],
        scaling=float(args["--scaling"]),
        goalBias=float(args["--goal-bias"]),
        image=args["<MAP>"],
        epsilon=float(args["--epsilon"]),
        max_number_nodes=int(args["--max-number-nodes"]),
        radius=float(args["--radius"]),
        goal_radius=2 / 3 * float(args["--radius"]),
        ignore_step_size=args["--ignore-step-size"],
        always_refresh=args["--always-refresh"],
        sampler=sampler,
        rrdt_proposal_distribution=args["--proposal-dist"],
        no_display=args["--no-display"],
        engine=args["--engine"],
        planner_type=planner_type,
        startPt=args["<start_x1,x2,..,xn>"],
        goalPt=args["<goal_x1,x2,..,xn>"],
    )

    # reduce goal radius for klampt as it plans in radian
    if args["--engine"] == "klampt":
        args["--goal_radius"] = 0.001

    # quick and dirty fix for docopt not able to handle negative argument
    if planner_options["startPt"]:
        if planner_options["startPt"].startswith("(") and planner_options[
            "startPt"
        ].endswith(")"):
            planner_options["startPt"] = planner_options["startPt"][1:-1]
    if planner_options["goalPt"]:
        if planner_options["goalPt"].startswith("(") and planner_options[
            "goalPt"
        ].endswith(")"):
            planner_options["goalPt"] = planner_options["goalPt"][1:-1]

    import env

    environment = env.Env(**planner_options)
    environment.run()


if __name__ == "__main__":
    main()
