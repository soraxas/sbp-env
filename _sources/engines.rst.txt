Simulator Engine
================

Currently there are three engines used in ``sbp-env`` that are used to simulate collisions and the planning environment.

2D Image Space
--------------

The first one is a **2D** image space engine with the configuration space (*C-Space*) as :math:`C \subseteq \mathbb{R}^2`.
This engine is very easy to use and very easy to add new environments.
This uses the :class:`collisionChecker.ImgCollisionChecker` for environment simulation and with the :class:`visualiser.PygameEnvVisualiser` to visualise the planning scene.

.. prompt:: bash

    python main.py rrdt maps/room1.png -vv

.. sidebar:: 2D Image simulator

    .. Figure:: ../images/rrdt.gif

This simulator uses images as its *C-Space* by treating each white pixlels as configurations in :math:`C_\text{free}`,
and each non-white pixles as configurations in :math:`C_\text{obs}`.
The two dimensional space of the *C-Space*

.. math::
    q \equiv [x_0, x_1] \in C \subseteq \mathbb{R}^2

would be taken as the *width* and the *height* of the image

.. math::
    0 \le x_0 < \mathcal{I}_\text{Width} \\
    0 \le x_1 < \mathcal{I}_\text{Height}

where :math:`\mathcal{I}` denotes the image space.

.. important::
    Internally, all pixels with an **RGB value** of `(255, 255, 255)` would be treated as :math:`q \in C_\text{free}`,
    and any non-white pixels value will be treated as :math:`q \in C_\text{obs}` (e.g. `(10, 20, 10)`, `(0, 0, 0)`, etc.).
    The alpha channel would not be considered.


4D Robot Arm
--------------

This simulator internally depends on :class:`collisionChecker.ImgCollisionChecker` to check for collision in the image space.

.. prompt:: bash

    python main.py rrt maps/4d.png -s .5 --engine 4d


.. sidebar:: 4D Robot Arm simulator

    .. Figure:: ../images/rrdt.gif

In conctast to the former, this simulator not only check for point mass collisions, but performs forward kinentamic on the given joints configuration to obtain body points in worldspace :math:`\mathcal{W}\subseteq \mathbb{R}^2`.
Since the robot arm contains two configurable joints, the full configuration space :math:`C` is given by

.. math::
    q \equiv [x_0, x_1, r_0, r_1] \in C \subseteq \mathbb{R}^4

where

.. math::
    \begin{aligned}
        0      & \le  x_0  <  \mathcal{I}_\text{Width} \\
        0      & \le  x_1  <  \mathcal{I}_\text{Height} \\
        - \pi  & \le  r_0  <  \pi \\
        - \pi  & \le  r_1  <  \pi
    \end{aligned}

and we use :math:`r_0, r_1` to denote the rotational angles of the first and second joints respectively.


.. important::
    Similar to :class:`collisionChecker.ImgCollisionChecker`, all white pixels will be within :math:`q \in C_\text{free}` and vice versa.
    The body points obtained by the forward kinentamic on :math:`r_0, r_1` would be used to check collision in :math:`\mathcal{W}` to ensure the entire body is in free space.



:math:`n`-D Mainpulator
--------------

This simulator internally depends on :class:`collisionChecker.ImgCollisionChecker` to check for collision in the image space.

.. prompt:: bash

    python main.py rrt maps/4d.png -s .5 --engine 4d


.. sidebar:: :math:`n`-D Mainpulator simulator

    .. Figure:: ../images/klampt-simulator.png

In conctast to the former, this simulator not only check for point mass collisions, but performs forward kinentamic on the given joints configuration to obtain body points in worldspace :math:`\mathcal{W}\subseteq \mathbb{R}^3` and :math:`C \subseteq \mathbb{R}^d`.
Since the robot arm contains two configurable joints, the full configuration space :math:`C` is given by

.. math::
    q \equiv [r_0, \ldots, r_{d-1}] \in C \subseteq \mathbb{R}^

where

.. math::
    \begin{aligned}
        - \pi  & \le  r_i  <  \pi \quad \forall i \in \{0, \ldots, d-1\}
    \end{aligned}

and we use :math:`r_0, r_1` to denote the rotational angles of the first and second joints respectively.


.. important::
    Internally, all pixels with an **RGB value** of `(255, 255, 255)` would be treated as :math:`q \in C_\text{free}`,
    and any non-white pixels value will be treated as :math:`q \in C_\text{obs}` (e.g. `(10, 20, 10)`, `(0, 0, 0)`, etc.).
    The alpha channel would not be considered.





