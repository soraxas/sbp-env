#!/usr/bin/env python
"""SBP-Env

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
                         Could be either "image", "klampt" or "4d".
                         "image" engine uses an image as the map.
                         "4d" engine uses an image as the map and an 4d robot arm.
                         "klampt" engine is a multi-dof engine that is more
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
from typing import Optional

import numpy as np
from docopt import docopt

import env
import planners
from utils import planner_registry
from utils.common import MagicDict
from visualiser import VisualiserSwitcher

assert planners

LOGGER = logging.getLogger()

# add all of the registered planners
__doc__ = __doc__.format(
    all_available_planners="|".join(
        sorted(
            (planner.name for planner in planner_registry.PLANNERS.values()),
            reverse=True,
        )
    )
)


def generate_args(
    planner_id: Optional[str],
    map_fname: Optional[str],
    start: Optional[np.ndarray] = None,
    goal: Optional[np.ndarray] = None,
) -> MagicDict:
    """The entry point of the planning scene module

    :param map_fname: overrides the map to test
    :param start: overrides the starting configuration
    :param goal: overrides the goal configuration

    :return: the default dictionary of arguments to config the planning problem
    """
    if map_fname is None or planner_id is None:
        if len(sys.argv) < 3:
            raise ValueError(
                "Both `map_fname` and `planner_id` must be provided to " "generate args"
            )
    else:
        # inject the inputs for docopt to parse
        sys.argv[1:] = [planner_id, map_fname]

    args = docopt(__doc__, version="SBP-Env Research v2.0")

    # allow the map filename, start and goal point to be override.
    if start is not None:
        args["<start_x1,x2,..,xn>"] = start
    if goal is not None:
        args["<goal_x1,x2,..,xn>"] = goal

    # setup environment engine
    args["--engine"] = "" if args["--engine"] is None else args["--engine"].lower()
    if args["--engine"] not in ("image", "klampt", "4d", ""):
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
        if args["--engine"] in ("image", "4d"):
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

    sampler_data_pack = planner_registry.SAMPLERS[planner_data_pack.sampler_id]
    sampler = sampler_data_pack.sampler_class(
        sampler_data_pack=sampler_data_pack,
        # sanitise keyword arguments by removing prefix -- and replacing - as _
        **{re.sub(r"^--", "", k).replace("-", "_"): v for (k, v) in args.items()},
    )

    planning_option = MagicDict(
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
        start_pt=args["<start_x1,x2,..,xn>"],
        goal_pt=args["<goal_x1,x2,..,xn>"],
    )

    # reduce goal radius for klampt as it plans in radian
    if planning_option.engine == "klampt":
        planning_option["goal_radius"] = 0.001

    return planning_option


if __name__ == "__main__":
    planning_option = generate_args(planner_id=None, map_fname=None)

    environment = env.Env(**planning_option)
    environment.run()
