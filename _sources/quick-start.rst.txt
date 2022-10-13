.. _quick_start:

Quick Start
===========

Installation
----------------------

Using pip

.. code-block:: bash

    pip install sbp-env

Or manually:

Download the latest code and install required dependencies listed inside
`requirements.txt` (recommend to always isolate your python workspace with virtual
environment, e.g., conda/pyenv)

.. code-block:: bash

    # get code from remote
    git clone https://github.com/soraxas/sbp-env
    cd sbp-env

    # optional, using virtual env
    conda create --name my_ws python=3.8
    conda activate my_ws

You would first need to install the dependencies with

.. code-block:: bash

    # install dependencies
    pip3 install -r requirements.txt
    # optional dependencies for manipulator simulator
    pip3 install -r requirements_klampt.txt

The quickest way to test out the capability of a planner is to experiment with a *2D
Image Space* planner.

.. sidebar:: 2D Image simulator

    .. Figure:: ../images/rrdt.gif

.. code-block:: bash

    python main.py rrt maps/room1.png

The syntax always requires a positional argument which specify the planning
environment. In the case of the 2D engine or 4D rover arm, it should be a path to an
image file (png/jpg). For the case of the :math:`n`-D manipulator, it should be a path
to an xml file as specified in `Klampt
<https://github.com/krishauser/Klampt>`_.


Using other simulator
----------------------

The type of simulator can be set with the :code:`--engine` flag. For example, the 4D
robot arm simulator can be used with the :code:`4d` value.

.. sidebar:: 4D Robot Arm simulator

    .. Figure:: ../images/robot-arm4d.gif

.. code-block:: bash

    python main.py rrt maps/4d.png --engine 4d

You can also set the length of the rover arm (a list of two float that corresponds to
the 2 sticks' length) with the following.

.. code-block:: bash

    python main.py rrt maps/room1.png --engine 4d --4d-robot-lengths 15,15

Notice that the argument for :code:`--4d-robot-lengths` must be
comma-separated numbers, each of the number represent the length of the rover arm.

For the 4d rover arm simulator, :code:`sbp-env` will visualise each node with the
actual arm that is currently used for collision-checking.
The two arms are currently being displayed in orange and cyan colour for
distinguishing them when the display is cluttered.
The colour are currently non-configurable, and only matters to the visualisation of
the planning horizon.



.. sidebar:: :math:`n`-D Manipulator simulator

    .. Figure:: ../images/klampt-rrdt.gif

You can also launch the 3D simulator environment with the following command, given
that you had already installed the optional :code:`klampt` dependencies that was
mentioned previously. Note that you will need to supply the environment file as
specified in `klampt's doc
<http://motion.cs.illinois.edu/software/klampt/latest/pyklampt_docs/Manual-FileTypes.html>`_ as they are passed directly to the :code:`klampt` simulator backend.

We have included a complementary set of klampt data `here
<https://github.com/soraxas/sbp-env/releases/tag/klampt-data>`_, which is adapted
from the official :code:`klampt` `example repository
<https://github.com/krishauser/Klampt-examples>`_.

You can use the following commands to quickly download the klampt data files:

.. code-block:: bash

    cd sbp-env

    # download klampt-data from remote
    wget https://github.com/soraxas/sbp-env/releases/download/klampt-data/klampt_data.tar.gz
    # extract
    tar xzf klampt_data.tar.gz

And last but not least, you can start the :code:`klampt` simulator with

.. code-block:: bash

    python main.py rrt klampt_data/tx90blocks.xml --engine klampt



.. important::
    If you don't have :code:`OpenGL` installed, you might also need to install it for
    using the :code:`klampt` simulator.
    If you are using pip, you can install it with

    .. code-block:: bash

        pip3 install pyopengl pyqt5


Saving the planner statistics
------------------------------

During the planning episode, :code:`sbp-env` will keep track of various statistics which
are beneficial to compare the performance between planner. The statistics will always
be displayed as part of the :code:`tqdm` progress bar.

You can save the statistics to a :code:`.csv` file with the :code:`--save-output` flag,
e.g.

.. code-block:: bash

    python main.py rrt maps/room1.png start 100,100 goal 350,350 --save-output

which would save the output to a timestamped :code:`.csv` file under the :code:`runs/`
folder by default. You can also customise the output folder with
:code:`--output-dir=MY_FOLDER`.

The recorded statistics have the following meanings:
    - :code:`nodes`: The number of nodes
    - :code:`time`: Timestamp
    - :code:`cc_feasibility`: The number of collision-checks for feasibility (node)
    - :code:`cc_visibility`: The number of collision-checks for visibility (edge)
    - :code:`invalid_feasibility`: The number of invalid feasibility checks
    - :code:`invalid_visibility`: The number of invalid visibility checks
    - :code:`c_max`: The current cost of the solution trajectory
