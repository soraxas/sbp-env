# Rapidly-exploring Random disjointed-Trees

RRdT* uses multiple disjointed-trees to exploit local-connectivity of spaces via [Markov Chain Monte Carlo (MCMC)](https://en.wikipedia.org/wiki/Markov_chain_Monte_Carlo) random walk, which utilises neighbourhood information derived from previous successful and failed samples. The active balancing of global exploration and local exploitation is the key to improve sample efficiency.

This is a repository that contains a Python implementation of the algorithm describe in the paper, along with implementation of other state-of-the-art planners that had been experimentally compared to RRdT*.

The algorithm is described in the [paper](https://arxiv.org/abs/1810.03749) from ICRA'19. A complementary video that explains the concept can be found [here](https://www.youtube.com/watch?v=6kAZnQeULdY). If you want to cite this work in your research, you can use [this BibTex entry](#bibtex).


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
It will open a new window that display a map on it. Every white pixel is assumed to be free, and non-white pixels are obstacles. You will need to use your mouse to perform two points on the map, the first will be set as the starting point and the second as the goal point.


## Available Planners

This repository contains a framework to performs quick experiments for Sampling-Based Planners (SBPs) that are implemented in Python. The followings are planners that had implemented and experimented in this framework.

Note that the command provided here can be customised with additional options. The one provided here are useful for debugging as it enables verbose mode. In fact, the actual command format used for the demonstrations is
```sh
python main.py <PLANNER> maps/room1.png start <sx> <sy> goal <sx> <sy> -vv
python main.py <PLANNER> maps/room1.png start 86 214 goal 454 350 -vv
```
to have a fix set of starting and goal points for consistent visualisation, but we omitted the start/goal options in the following commands for clarity.

### RRdT*

```sh
python main.py rrdt maps/room1.png -vv
```

### RRT*

```sh
python main.py rrt maps/room1.png -vv
```

### Bi-RRT*

```sh
python main.py birrt maps/room1.png -vv
```

### Informed-RRT*

```sh
python main.py prm maps/room1.png -vv --max-number-nodes=10000
```

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
