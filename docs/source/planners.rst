Motion Planners
===============

The following are all of the available motion planners in ``sbp-bench``.





asdas


RRT*
---------------------

Rapidly-exploring Random Tree* (RRT*) is an anytime, asymptotic optimal sampling-based
motion planner that continuously updates its solution within the given budget.
The planner itself is implemented in :class:`~planners.rrtPlanner.RRTPlanner`, while
the default sampling
policy is implemented in :class:`~samplers.randomPolicySampler.RandomPolicySampler`.

.. image:: ../images/rrt.gif

.. autoclass:: planners.rrtPlanner.RRTPlanner
  :members:
  :private-members:
  :show-inheritance:


Bi-RRT*
---------------------

The bidrectional RRT* planner, or sometimes it's also referred to as the *RRT-Connect\**.
The class :class:`~planners.birrtPlanner.BiRRTPlanner` uses an adopted version of
random policy sampler that makes it suitable for using in both the start and goal
trees, which is implemented in :class:`~samplers.birrtSampler.BiRRTSampler`.

.. image:: ../images/birrt.gif

.. autoclass:: planners.birrtPlanner.BiRRTPlanner
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

-------------------



RRdT* Planner
---------------------

.. autoclass:: planners.rrdtPlanner.RRdTPlanner
  :members:
  :undoc-members:
  :private-members:
  :show-inheritance:
  :inherited-members:



asdas
---------



basePlanner







  RRdT* Planner
  ---------------------

  .. autoclass:: planners.rrdtPlanner.RRdTPlanner
  :members:
  :undoc-members:
  :private-members:
  :show-inheritance:
  :inherited-members:

RRdT* Planner
---------------------

.. automodule:: planners.rrdtPlanner
  :members:
  :undoc-members:
  :private-members:
  :show-inheritance:
  :inherited-members:


.. .. automodule:: planners.prmPlanner
..   :member:

Some words.

.. .. autosummary::
..    :toctree: _autosummary
..   ..  :template: custom-module-template.rst
..    :recursive:

..    planners

.. .. autosummary::
..    :toctree: _autosummary
..    :template: custom-module-template.rst
..    :recursive:

..    env



informedrrtPlanner
prmPlanner
likelihoodPlanner
rrdtPlanner
basePlanner
mousePlanner
rrtPlanner
birrtPlanner
nearbyPlanner