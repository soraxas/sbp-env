# Sampling-Based Motion Planners' Testing Environment

[![Python version](https://img.shields.io/badge/python-3.7%20%7C%203.8%20%7C%203.9%20%7C%203.10-blue.svg)](https://cs.tinyiu.com/sbp-env)
[![CI](https://github.com/soraxas/sbp-env/actions/workflows/ci.yaml/badge.svg)](https://github.com/soraxas/sbp-env/actions/workflows/ci.yaml)
[![Build docs](https://github.com/soraxas/sbp-env/actions/workflows/sphinx.yaml/badge.svg)](https://cs.tinyiu.com/sbp-env)
[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)
[![License](https://img.shields.io/github/license/soraxas/sbp-env.svg)](https://github.com/soraxas/sbp-env/blob/master/LICENSE)
[![DOI](https://joss.theoj.org/papers/10.21105/joss.03782/status.svg)](https://doi.org/10.21105/joss.03782)

Sampling-based motion planners' testing environment (`sbp-env`) is a full feature framework to quickly test different sampling-based algorithms for motion planning. `sbp-env` focuses on the flexibility of tinkering with different aspects of the framework, and had divided the main planning components into two categories (i) **samplers** and (ii) **planners**.

The focus of *motion planning research* had been mainly on (i) improving the sampling efficiency (with methods such as heuristic or learned distribution) and (ii) the algorithmic aspect of the planner using different routines to build a connected graph. Therefore, by separating the two components one can quickly swap out different components to test novel ideas.

Have a look at the [documentations](https://cs.tinyiu.com/sbp-env) for more detail information. If you are looking for the previous code for the RRdT* paper it is now archived at [soraxas/rrdt](https://github.com/soraxas/rrdt).

## Quick start with custom arbitrary environments

```sh
pip install sbp-env
```

```python
import sbp_env

from math import exp, sin, cos

for functor in [
    # simple inequality
    lambda x: (x[1] < x[0] + 1) and (x[1] > x[0] - 1),
    # equation adopted from https://au.mathworks.com/help/matlab/ref/peaks.html
    lambda x: 0
    < (
        3 * (1 - x[0]) ** 2.0 * exp(-(x[0] ** 2) - (x[1] + 1) ** 2)
        - 10 * (x[0] / 5 - x[0] ** 3 - x[1] ** 5) * exp(-x[0] ** 2 - x[1] ** 2)
        - 1 / 3 * exp(-((x[0] + 1) ** 2) - x[1] ** 2)
    ),
    lambda x: -0.22 < (cos(x[0]) * sin(x[1])),
    lambda x: 0.05 < (cos(x[0] ** 2 + x[1] ** 2)),
]:
    engine = sbp_env.engine.BlackBoxEngine(
        collision_checking_functor=functor,
        lower_limits=[-5, -5], upper_limits=[5, 5],
        cc_epsilon=0.1,  # collision check resolution
    )
    planning_args = sbp_env.generate_args(
        planner_id="rrt", engine=engine,
        start_pt=[-3, -3], goal_pt=[3, 3],
        display=True, first_solution=True,
    )

    env = sbp_env.env.Env(args=planning_args)
    env.run()
    print(env.get_solution_path(as_array=True))
```

<p align="center">
  <img src="https://raw.githubusercontent.com/soraxas/sbp-env/master/docs/images/functor-engine1.png" width="400" />
  <img src="https://raw.githubusercontent.com/soraxas/sbp-env/master/docs/images/functor-engine2.png" width="400" />
  <img src="https://raw.githubusercontent.com/soraxas/sbp-env/master/docs/images/functor-engine3.png" width="400" />
  <img src="https://raw.githubusercontent.com/soraxas/sbp-env/master/docs/images/functor-engine4.png" width="400" />
</p>

## Installation

#### Optional

I recommend first create a virtual environment with

```sh
# assumes python3 and bash shell
python -m venv sbp_env
source sbp_env/bin/activate
```

#### Install dependencies

You can install all the needed packages with pip.

```sh
pip install -r requirements.txt
```

There is also an optional dependency on [`klampt`](https://github.com/krishauser/Klampt) if you want to use the 3D simulator. Refer to its [installation guide](https://github.com/krishauser/Klampt#installation) for details.

<img align="right" width="300" height="auto" src="https://raw.githubusercontent.com/soraxas/sbp-env/master/docs/images/klampt-simulator.png" />

## Quick Guide

You can get a detailed help message with

```sh
python main.py --help
```

but the basic syntax is

```sh
python main.py <PLANNER> <MAP> [options]
```

It will open a new window that display a map on it. Every white pixel is assumed to be free, and non-white pixels are obstacles. You will need to use your mouse to select two points on the map, the first will be set as the starting point and the second as the goal point.

## Demos

### Run maps with different available Planners

This repository contains a framework to performs quick experiments for Sampling-Based Planners (SBPs) that are implemented in Python. The followings are planners that had implemented and experimented in this framework.

Note that the commands shown in the respective demos can be customised with additional options. In fact, the actual command format used for the demonstrations is

```sh
python main.py <PLANNER> maps/room1.png start <sx>,<sy> goal <sx>,<sy> -vv
```

to have a fix set of starting and goal points for consistent visualisation, but we omitted the start/goal options in the following commands for clarity.

### RRdT*

```sh
python main.py rrdt maps/room1.png -vv
```

<p align="center">
    <img width="600" height="auto" src="https://raw.githubusercontent.com/soraxas/sbp-env/master/docs/images/rrdt.gif" alt="RRdT* Planner" />
</p>

### RRT*

```sh
python main.py rrt maps/room1.png -vv
```

<p align="center">
    <img width="600" height="auto" src="https://raw.githubusercontent.com/soraxas/sbp-env/master/docs/images/rrt.gif" alt="RRT* Planner" />
</p>

### Bi-RRT*

```sh
python main.py birrt maps/room1.png -vv
```

<p align="center">
    <img width="600" height="auto" src="https://raw.githubusercontent.com/soraxas/sbp-env/master/docs/images/birrt.gif" alt="Bi-RRT* Planner" />
</p>

### Informed RRT*

```sh
python main.py informedrrt maps/room1.png -vv
```

<p align="center">
<img width="600" height="auto" src="https://raw.githubusercontent.com/soraxas/sbp-env/master/docs/images/informedrrt.gif" alt="Informed RRT* Planner" />
</p>

The red ellipse shown is the dynamic sampling area for Informed RRT*

### Others

There are also some other planners included in this repository. Some are preliminary planner that inspired RRdT*, some are planners with preliminary ideas, and some are useful for debugging.

## Reference to this repository

You can use the following citation if you use this repository for your research
```bibtex
@article{lai2021SbpEnv,
  doi = {10.21105/joss.03782},
  url = {https://doi.org/10.21105/joss.03782},
  year = {2021},
  publisher = {The Open Journal},
  volume = {6},
  number = {66},
  pages = {3782},
  author = {Tin Lai},
  title = {sbp-env: A Python Package for Sampling-based Motion Planner and Samplers},
  journal = {Journal of Open Source Software}
}
```
