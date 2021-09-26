---
title: 'sbp-env: A Python Package for Sampling-based Motion Planner and Samplers'
tags:
  - Python
  - motion planning
  - sampling-based planner
  - robotics
authors:
  - name: Tin Lai
    orcid: 0000-0003-0641-5250
    affiliation: '1'
affiliations:
  - name: School of Computer Science, The University of Sydney, Australia
    index: 1
date:
bibliography: master.bib
---

# Background

Sampling-based motion planning is one of the fundamental methods for robots to navigate and integrate with the real world [@elbanhawi2014_SampRobo].
Motion planning involves planning the trajectories of the actuated part of the robot, under various constraints, while avoiding collisions with surrounding obstacles.
Sampling-based motion planners (SBPs) are robust methods that avoid explicitly constructing the often intractable high-dimensional Configuration Space (C-Space).
Instead, SBPs sample the C-Space randomly for valid connections and iteratively builds a roadmap of connectivity.
Most SBPs are guaranteed to find a solution if one exists [@kavraki1996_AnalProb], and such a planner is said to be _probabilistic complete_ .
Further developments on SBPs had granted _asymptotic optimality_[@elbanhawi2014_SampRobo]---a guarantee that the method will converge, in the limit, to the optimal solution.

SBPs are applicable to a wide range of application
For example, planning in arbitrary cost map [@iehlCostmapPlanningHigh2012], cooperative multi-agents planning [@jinmingwuCooperativePathfindingBased2019], as well as in dynamic environments [@yershova2005_DynaRRTs].
On the one hand, researchers had focused on the algorithmic side of improving the graph or tree building [@lai2018_BalaGlob;@klemmRRTConnectFaster2015;@zhongTripleRrtsRobotPath2012;@elbanhawi2014_SampRobo;@lai2021lazyExperienceGraph;@lai2021rapidlyexploring].
On the other hand, the advancement of neural networks allows an abundance of learning approaches to be applied in SBPs [@strubAdaptivelyInformedTrees2020;@bagnell2014_ReinLear] and on improving the sampling distribution [@alcin2016_ExtrLear;@lai2020_BayeLoca;@lai2021plannerFlows;@laiLearningPlanOptimally2020;@lai2021diffSamp].


# Statement of need

The focus of motion planning research had been mainly on (i) the algorithmic aspect of the planner using different routines to build a connected graph and (ii) improving the sampling efficiency (with methods such as heuristic or learned distribution). Traditionally, robotic research focuses on algorithmic development, which has inspired several motion planning libraries written in C++, such as Move3D [@simeon2001move3d] and OMPL [@sucan2012open]. In particular, OMPL had been one of the most well-known motion planning libraries due to its versatility and had been a core part of the planning algorithm used in the MoveIt framework [@chitta2012moveit]. However, swapping the sampler within each planner is very restrictive as they are hard-coded to use a specific sampler. In addition, it is cumbersome to integrate any learning-based approach into the framework as there are only a limited amount of choices of deep-learning libraries in C++.

Python has been a popular language to use in Machine Learning due to its rapid scripting nature. For example, PyTorch [@paszke2019pytorch] and Tensorflow [@abadi2016tensorflow] are the two popular choices of neural network framework in Python. There had been an extensive amount of implemented learning approaches available as Python packages. It shall be noted that the aforementioned OMPL has Python bindings available; however, OMPL uses an outdated Py++ code generator, and every modification to the source code will require hours to updates bindings plus recompilation.
Some Python repositories are available that are dedicated to robotics motion planning [@sakai2018pythonrobotics]; however, most are only showcasing various planning algorithms with no integrated environment and simulators.

![Implementation details on the class hierarchy structure of `sbp-env`.\label{fig:class-diagram}](class_diagram.png)

# Overview

We introduce `sbp-env`, a _sampling-based motion planners' testing environment_, as a complete feature framework to allow rapid testing of different sampling-based algorithms for motion planning.
Our `sbp-env` focuses on the flexibility of tinkering with different aspects of the framework and has divided the main planning components into two main categories: (i) samplers and (ii) planners.
The division of components allows users to decouple the two components and focus only on the component that serves as the main focus of the research.
The `sbp-env` had implemented the entire robot planning framework with multiple degree-of-freedom, which allow benchmarking motion planning algorithm with the same planner under different backend simulators.
Therefore, by separating the two components, one can quickly swap out different components to test novel ideas.

Building the framework enables researchers to implements their novel idea and validate their hypothesis rapidly.
In particular, users can use as simple as an _image_, or as complicated as _xml files_ to define the environment.
All samplers and planners can be added as a plugin system, and `sbp-env` will auto-discover newly implemented planners or samplers that are added to the dedicated folders.

Figure \ref{fig:class-diagram} illustrates the hierarical structure of our package.
Our implementation of `sbp-env` define abstract interfaces for **sampler** and **planners**, of which all corresponding concrete class must inherit from them.
In addition, there are classes that represent the full-body simulations of the environments and the corresponding visualisation methods.
Note that all visualisation can be turned off on-demand, which is beneficial when users benchmark their algorithm.
The docunmentation of `sbp-env` is available at [https://cs.tinyiu.com/sbp-env](https://cs.tinyiu.com/sbp-env).


# References
