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
  -o --output-dir=STATS_DIR
                         Specify the output folder [default: runs]
  --save-output          When set, the planning stats will be saved under the
                         'STATS_DIR' folder
  --first-solution       When set, the planning stops as soon as an initial
                         solution is obtained.

Display Options:
  --always-refresh       Set if the display should refresh on every ticks.
  -s --scaling=SCALING   Set scaling for display (ratio will be maintained).
                         [default: 1.5]
  --hide-sampled-points  Do not display sampled point as red dot on screen.
  --no-display           Disable visualisation.

General Planner Options:
  --radius=RADIUS        Set radius for node connections.
  --epsilon=EPSILON      Set epsilon value for step size.
                         [default: 10.0]
  --max-number-nodes=MAX_NODES
                         Set maximum number of nodes
                         [default: 10000]
  --ignore-step-size     Ignore step size (i.e. epsilon) when sampling.
  --goal-bias=BIAS       Probability of biasing goal position.
                         [default: 0.02]
  --skip-optimality      Skip optimality guarantee (i.e. skip performing rewiring)

4D Simulaotr Options:
  --4d-robot-lengths=LENGTH_1,LENGTH_2
                         Set the lengths of the 4D rover arm manipulator. Should be
                         a comma separated list of two numbers.
                         [default: 30,30]


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
import sys
from typing import Optional, Union, List, Any

import numpy as np
from docopt import docopt

from . import planners, _version
from .engine import Engine, ImageEngine, RobotArmEngine, KlamptEngine, BlackBoxEngine
from .utils import planner_registry
from .utils.common import PlanningOptions, Stats

assert planners

LOGGER = logging.getLogger()

RAW_DOC_STRING = __doc__


def generate_args(
    planner_id: str,
    input_fname: Optional[str] = None,
    engine: Optional[Engine] = None,
    start_pt: Union[np.ndarray, str] = None,
    goal_pt: Union[np.ndarray, str] = None,
    display: bool = False,
    **kwargs,
) -> PlanningOptions:
    default_args = dict(
        no_display=not display,
    )
    for k, v in default_args.items():
        if k not in kwargs:
            kwargs[k] = v

    argv = []
    if all(x is None for x in (input_fname, engine)) or all(
        x is not None for x in (input_fname, engine)
    ):
        raise ValueError(
            "Either an input file `input_fname` or a custom Engine `engine` must be "
            "provided to generate args, and they are mutually exclusive."
        )
    if planner_id not in planner_registry.PLANNERS:
        raise ValueError(f"The given planner id '{planner_id}' does not exists.")
    # inject the inputs for docopt to parse
    argv[1:] = [planner_id, str(input_fname)]
    if isinstance(engine, BlackBoxEngine):
        if "epsilon" not in kwargs:
            # default epsilon to cc epsilon
            kwargs["epsilon"] = engine.cc.epsilon
    args = generate_args_main(
        start_pt=start_pt, goal_pt=goal_pt, argv=argv, engine=engine, **kwargs
    )

    return args


def generate_args_main(
    start_pt: Optional[Union[np.ndarray, str]] = None,
    goal_pt: Optional[Union[np.ndarray, str]] = None,
    engine: Optional[Engine] = None,
    argv: Optional[List[Any]] = None,
    **kwargs,
) -> PlanningOptions:
    """Get the default set of arguments

    :param planner_id: the planner to use in the planning environment
    :param map_fname: the filename of the map to use in the planning environment
    :param start_pt: overrides the starting configuration
    :param goal_pt: overrides the goal configuration
    :param engine: a custom engine
    :param argv: a custom argv list

    :return: the default dictionary of arguments to config the planning problem
    """
    if argv is None:
        argv = sys.argv[1:]

    # add all of the registered planners
    args = docopt(
        doc=format_doc_with_registered_planners(RAW_DOC_STRING),
        argv=argv,
        version=_version.__version__,
    )

    # allow the map filename, start and goal point to be override.
    if start_pt is not None:
        args["<start_x1,x2,..,xn>"] = start_pt
    if goal_pt is not None:
        args["<goal_x1,x2,..,xn>"] = goal_pt

    # setup environment engine
    args["--engine"] = "" if args["--engine"] is None else args["--engine"].lower()
    if args["--engine"] not in ("image", "klampt", "4d", ""):
        raise RuntimeError(
            f"Unrecognised value '{args['--engine']}' for engine option!"
        )
    if engine is None and args["--engine"] == "":
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
        elif _file_extension in (
            "jpg",
            "png",
        ):
            args["--engine"] = "image"
        else:
            raise RuntimeError(
                "No engine given and unable to infer engine from "
                "extension '{}'".format(_file_extension)
            )
        LOGGER.info(_notice.format(args["--engine"], _file_extension))

    args["--4d-robot-lengths"] = args["--4d-robot-lengths"].split(",")
    if len(args["--4d-robot-lengths"]) != 2:
        raise RuntimeError(
            "The comma separated robot arm lengths must contains exactly 2 numbers."
        )
    args["--4d-robot-lengths"] = tuple(map(float, args["--4d-robot-lengths"]))

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

    planner_data_pack = planner_registry.PLANNERS[planner_to_use]

    sampler_data_pack = planner_registry.SAMPLERS[planner_data_pack.sampler_id]

    # setup defaults
    if args["--engine"] == "klampt":
        # setup defaults for klampt
        if args["--epsilon"] == "10.0":
            args["--epsilon"] = 1.0

    def cast_float_if_not_none(value: Optional[str]) -> Optional[float]:
        if value is not None:
            value = float(value)
        return value

    planning_option = PlanningOptions(
        planner_data_pack=planner_data_pack,
        skip_optimality=args["--skip-optimality"],
        showSampledPoint=not args["--hide-sampled-points"],
        scaling=float(args["--scaling"]),
        goalBias=float(args["--goal-bias"]),
        # image=args["<MAP>"],
        epsilon=float(args["--epsilon"]),
        max_number_nodes=int(args["--max-number-nodes"]),
        radius=cast_float_if_not_none(args["--radius"]),
        ignore_step_size=args["--ignore-step-size"],
        always_refresh=args["--always-refresh"],
        sampler_data_pack=sampler_data_pack,
        rrdt_proposal_distribution=args["--proposal-dist"],
        no_display=args["--no-display"],
        start_pt=args["<start_x1,x2,..,xn>"],
        goal_pt=args["<goal_x1,x2,..,xn>"],
        rover_arm_robot_lengths=args["--4d-robot-lengths"],
        output_dir=args["--output-dir"],
        save_output=args["--save-output"],
        first_solution=args["--first-solution"],
    )

    # reduce goal radius for klampt as it plans in radian
    if args["--engine"] == "klampt":
        planning_option.goal_radius = 0.001

    # add the keys which will be added by the env later
    planning_option.update(
        dict(
            planner=None,
            sampler=None,
        )
    )
    if engine is None:
        engine = {"image": ImageEngine, "4d": RobotArmEngine, "klampt": KlamptEngine,}[
            args["--engine"]
        ](planning_option, args["<MAP>"])
    planning_option.engine = engine
    planning_option.update(kwargs)
    planning_option.compute_default_values()

    return planning_option


def format_doc_with_registered_planners(doc: str):
    """
    Format the main.py's doc with the current registered planner

    :param doc: the doc string to be formatted
    """
    return doc.format(
        all_available_planners="|".join(
            sorted(
                (planner.name for planner in planner_registry.PLANNERS.values()),
                reverse=True,
            )
        )
    )


__doc__ = format_doc_with_registered_planners(__doc__)
