Planning Scene Visualisers
================


2D Image Space Visualiser
-------------------------


.. class:: visualiser.PygameEnvVisualiser



.. autoclass:: visualiser.PygameEnvVisualiser
  :members:
  :private-members:
  :show-inheritance:


.. autoclass:: visualiser.PygamePlannerVisualiser
  :members:
  :private-members:
  :show-inheritance:


.. autoclass:: visualiser.PygameSamplerVisualiser
  :members:
  :private-members:
  :show-inheritance:


4D Image Space Manipulator Visualiser
-------------------------

The 4D manipulator visualiser uses the same
:class:`visualiser.PygameEnvVisualiser`,
:class:`visualiser.PygamePlannerVisualiser` and
:class:`visualiser.PygameSamplerVisualiser` as the `2D Image Space Visualiser`_
to visualise the planning scene. However, it performs the necessary kinematics transformations to translates configurations :math:`q \in C` to worldspace :math:`x \in \mathcal{W}`
with the help of the collision checker :class:`collisionChecker.RobotArm4dCollisionChecker`.

.. warning::

    Currently this visualiser/simulator is hard-coded so it might requires you to dig into the source code to understand what's going on.


.. todo::

    add images to showcase 4d robot arm



3D Object-based Visualiser
-------------------------


.. autoclass:: visualiser.PygameEnvVisualiser
  :members:
  :private-members:
  :show-inheritance:


.. autoclass:: visualiser.PygamePlannerVisualiser
  :members:
  :private-members:
  :show-inheritance:


.. autoclass:: visualiser.PygameSamplerVisualiser
  :members:
  :private-members:
  :show-inheritance:




Currently there are three engines used in ``sbp-env`` that are used to simulate collisions and the planning environment.

The first one is a **2D**  engine with the configuration space (*C-Space*) as :math:`C \subseteq \mathbb{R}^2`.
This engine is very easy to use and very easy to add new environments.
This uses a the :class:`collisionChecker.ImgCollisionChecker` for environment simulation and with the :class:`visualiser.PygameEnvVisualiser` to visualise the planning scene.
