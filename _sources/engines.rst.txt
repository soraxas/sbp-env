Simulator Engine
================

Currently there are three engines used in ``sbp-env`` that are used to simulate collisions and the planning environment.

The first one is a **2D** image space engine with the configuration space (*C-Space*) as :math:`C \subseteq \mathbb{R}^2`.
This engine is very easy to use and very easy to add new environments.
This uses the :class:`collisionChecker.ImgCollisionChecker` for environment simulation and with the :class:`visualiser.PygameEnvVisualiser` to visualise the planning scene.
