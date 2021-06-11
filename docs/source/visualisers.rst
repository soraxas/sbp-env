Planning Scene Visualisers
==========================

Each simulator engines would needs a corresponding visualiser for the framework to
know what to display. The simulator and visualiser are decoupled, for the purpose of
benchmarking. During benchmark, one would want to disable the visualisation because
the drawing (or the preparation) of visual elements would slow down the planning
process. Even if the slowdown is consistent across different types of planners, it
always best to have the ability to obtain quick results.

Each planner can register a visualisation function for a specific type of simulator.
You can **disable the visualisation** of the planning problem, regardless of simulator
type, with the following.

.. prompt:: bash

    python main.py <PLANNER> <MAP> start <q_1>,...,<q_n> goal <p_1>,...,<p_n> --no-display

For example:

.. prompt:: bash

    python main.py rrt maps/intel_lab.png start 25,25 goal 225,225 --no-display

.. important::
    Notice that in the example above, the argument `start` and `goal` are directly
    provided in the prompt. This is because with the `--no-display` flag there won't be
    any GUI for user to select the start/goal pair. Therefore, it is necessary for user
    to directly supply the starting and target configurations.

The following sections showcase the different classes of visualiser.

2D Image Space Visualiser
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


4D Image Space Manipulator Visualiser
-------------------------------------

The 4D manipulator visualiser uses the same
:class:`visualiser.PygameEnvVisualiser`,
:class:`visualiser.PygamePlannerVisualiser` and
:class:`visualiser.PygameSamplerVisualiser` as the `2D Image Space Visualiser`_
to visualise the planning scene. However, it performs the necessary kinematics transformations to translates configurations :math:`q \in C` to worldspace :math:`x \in \mathcal{W}`
with the help of the collision checker :class:`collisionChecker.RobotArm4dCollisionChecker`.

.. warning::

    The settings of the robot arm's joint length is configurable from the collision
    checker's construction, but has not been exposed to the commandline interface yet.
    See :meth:`collisionChecker.RobotArm4dCollisionChecker.__init__`



3D Object-based Visualiser
--------------------------


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
