C-space Samplers
================


Random Policy Sampler
---------------------


.. currentmodule:: samplers.randomPolicySampler

:class:`~samplers.randomPolicySampler.RandomPolicySampler` samples configuration
:math:`q \in \mathbb{R}^d` across each dimension uniformly where :math:`q \sim
\mathcal{U}(0,1)^d`, with some percentage of bias towards the target configuration; i.e.,
when :math:`p \sim \mathcal{U}(0,1) < \epsilon`.


.. autoclass:: RandomPolicySampler
  :members:
  :private-members:
  :show-inheritance:


Bi-RRT Sampler
---------------------

The sampler that is used internally by :class:`~planners.birrtPlanner.BiRRTPlanner`.
Internally, :class:`~samplers.birrtSampler.BiRRTSampler` uses :class:`~samplers.randomPolicySampler.RandomPolicySampler`
to draw from its supported random methods.
The main differeces lies in the epsilon biasing where when :math:`p \sim \mathcal{U}(0,1) < \epsilon`, the sampler
will bias towards the correct **start** or **goal** tree depending on the tree that :class:`~samplers.birrtSampler.BiRRTSampler`
is currently planning for (in contrast to only always biasing towards the goal tree).

.. autoclass:: samplers.birrtSampler.BiRRTSampler
  :members:
  :private-members:
  :show-inheritance:


RRdT*
---------------------

All motion planner should derived from this base class.

.. image:: ../images/rrdt.gif

.. autoclass:: planners.rrdtPlanner.RRdTPlanner
  :members:
  :private-members:
  :show-inheritance:
  :inherited-members:


Informedrrt-RRT*
---------------------

All motion planner should derived from this base class.

.. image:: ../images/informedrrt.gif

The bulk of the implementation occurs in :class:`~samplers.informedrrtSampler.InformedRRTSampler`.
The Informed-RRT* is implemented by directly deriving the base motion planner as :class:`~planners.rrtPlanner.RRTPlanner`,
and registering its sampler as the :class:`~samplers.informedrrtSampler.InformedRRTSampler`.

After defining the informed sampler, registering a new informed planner is simply is as shown below.

.. literalinclude:: ../../planners/informedrrtPlanner.py
  :start-after: start register
  :end-before: finish register




PRM*
---------------------

All motion planner should derived from this base class.

.. autoclass:: planners.prmPlanner.PRMPlanner
  :members:
  :private-members:
  :show-inheritance:
  :inherited-members:


Likelihood Planner
---------------------

The bulk of the implementation occurs in :class:`~samplers.likelihoodPolicySampler.LikelihoodPolicySampler`.
The Informed-RRT* is implemented by directly deriving the base motion planner as :class:`~planners.rrtPlanner.RRTPlanner`,
and registering its sampler as the :class:`~samplers.likelihoodPolicySampler.LikelihoodPolicySampler`.

After defining the informed sampler, registering a new informed planner is simply is as shown below.

.. literalinclude:: ../../planners/likelihoodPlanner.py
  :start-after: start register
  :end-before: finish register


Nearby Planner
---------------------

All motion planner should derived from this base class.

.. .. autoclass:: planners.nearbyPlanner.NearbyPlanner
..   :members:
..   :private-members:
..   :show-inheritance:
..   :inherited-members:


Mouse Planner
---------------------

All motion planner should derived from this base class.

.. .. autoclass:: planners.mousePlanner.MousePlanner
..   :members:
..   :private-members:
..   :show-inheritance:
..   :inherited-members:


Abstract Base Planner
---------------------

There is also a special base planner that all motion planner should be derived from.

.. autoclass:: planners.basePlanner.Planner
  :members:
  :private-members:
  :show-inheritance:
  :inherited-members:


Abstract Base Sampler
---------------------


.. currentmodule:: samplers.baseSampler

:class:`~samplers.randomPolicySampler.RandomPolicySampler` samples configuration
:math:`q \in \mathbb{R}^d` across each dimension uniformly where :math:`q \sim
\mathcal{U}(0,1)^d`, with some percentage of bias towards the target configuration.


.. autoclass:: Sampler
  :members:
  :private-members:
  :show-inheritance:

