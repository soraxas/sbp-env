# Rapidly-exploring Random disjointed-Trees

RRdT* uses multiple disjointed-trees to exploit local-connectivity of spaces via [Markov Chain Monte Carlo (MCMC)](https://en.wikipedia.org/wiki/Markov_chain_Monte_Carlo) random walk, which utilises neighbourhood information derived from previous successful and failed samples. The active balancing of global exploration and local exploitation is the key to improve sample efficiency.

This is a repository that contains a Python implementation of the algorithm describe in the paper, along with implementation of other state-of-the-art planners that had been experimentally compared to RRdT*.

The algorithm is described in the [paper](https://arxiv.org/abs/1810.03749) from ICRA'19. A complementary video that explains the concept can be found [here](https://youtu.be/Cy-kYrzkPvA). If you want to cite this work in your research, you can use [this BibTex entry](#bibtex).


## Installation

#### Optional

I always recommend people to create a virtual environment for each of their specific research project. You can do this with
```sh
# assuming python3 and bash shell
python -m venv rrdt
source rrdt/bin/activate
```
to create a virtual environment named as `rrdt`.

#### Install dependencies

You can install all the needed packages with pip.
```sh
pip install -r requirements.txt
```


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
python main.py <PLANNER> maps/room1.png start <sx> <sy> goal <sx> <sy> -vv
```
to have a fix set of starting and goal points for consistent visualisation, but we omitted the start/goal options in the following commands for clarity.

### RRdT*

```sh
python main.py rrdt maps/room1.png -vv
```

<p align="center">
    <img width="600" height="auto" src="doc/images/rrdt.gif" alt="RRdT* Planner" />
</p>

### RRT*

```sh
python main.py rrt maps/room1.png -vv
```

<p align="center">
    <img width="600" height="auto" src="doc/images/rrt.gif" alt="RRT* Planner" />
</p>

### Bi-RRT*

```sh
python main.py birrt maps/room1.png -vv
```

<p align="center">
    <img width="600" height="auto" src="doc/images/birrt.gif" alt="Bi-RRT* Planner" />
</p>

### Informed RRT*

```sh
python main.py informedrrt maps/room1.png -vv
```

<p align="center">
<img width="600" height="auto" src="doc/images/informedrrt.gif" alt="Informed RRT* Planner" />
</p>

The red ellipse shown is the dynamic sampling area for Informed RRT*

### Others

There are also some other planners included in this repository. Some are preliminary planner that inspired RRdT*, some are planners with preliminary ideas, and some are useful for debugging.

## How does it works

You can have a look of the paper *[Balancing Global Exploration and Local-connectivity Exploitation with RRdT](https://arxiv.org/abs/1810.03749)*, or this video that briefly explain the concepts:
<p align="center">
<a href="https://youtu.be/Cy-kYrzkPvA"><img width="600" height="auto" src="doc/images/rrdt-video-thumbnail.png" alt="RRdT youtube video" /></a>
</p>

## BibTeX
If you found the algorithm or the implementations of SBPs framework useful, please cite the following paper.
```
@inproceedings{lai_rrdt2019,
    title={Balancing Global Exploration and Local-connectivity Exploitation with Rapidly-exploring Random disjointed-Trees},
    author={Lai, Tin and Ramos, Fabio and Francis, Gilad},
    booktitle={Proceedings of The International Conference on Robotics and Automation (ICRA)},
    year={2019},
    organization={IEEE}
}
```
