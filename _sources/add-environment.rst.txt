Adding new environment to test
==============================

With :code:`sbp-env`, it is extremely easy to add a new environment and run the
same set of planners on that new environment.
You can add an environment by supplying an image or xml file format in the
commandline via

.. code-block:: bash

    python main.py rrt <IMG_OR_XML> --engine ENGINE

and the required type depends on the simulator engine.
Both the 2D and 4D simulator will requires an image format to be used as the obstacle
space, while the :code:`klampt` simulator will requires xml format as specified in `klampt's doc
<http://motion.cs.illinois.edu/software/klampt/latest/pyklampt_docs/Manual-FileTypes.html>`_.

I recommend always using png as it is a lossless format, which is essential in for
the framework as it does not introduce artificial 'noise' to the image.
The framework uses the color of pixels to determine whether a given pixel is in
free-space :math:`C_\text{free}` or inside obstacles :math:`C_\text{obs}`.

Only white-pixels are considered as being in free-space. In other words, all
non-white pixels are taken as being inside :math:`C_\text{obs}`.
The image library used in Python take pixel as a value in-between 0-255 (or a tuple
of 3 values for RGB image).
We always ignore the alpha channel, and convert the given image to a gray-scale image.
Therefore, any pixels that are not of the value of 255 (represents the color white)
will be treated as obstacles.

The following figures showcase some of the Image-Space map that comes with
:code:`sbp-env`, and serve as examples of how regions within images are considered as
free-space.

.. sidebar:: Room

    .. Figure:: ../../maps/room1.png

.. sidebar:: Intel Lab

    .. Figure:: ../../maps/intel_lab.png

.. sidebar:: Random Obstacles

    .. Figure:: ../../maps/4d.png

.. sidebar:: Maze

    .. Figure:: ../../maps/maze1.png

