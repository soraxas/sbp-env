Motion Planners
===============

The following are all of the available motion planners in ``sbp-bench``.


RRT*
---------------------

.. image:: ../images/rrt.gif

.. autoclass:: planners.rrtPlanner.RRTPlanner
  :members:
  :private-members:
  :show-inheritance:

The sampler is registered as follows

.. literalinclude:: ../../samplers/randomPolicySampler.py
  :start-after: start register
  :end-before: finish register

and the planner is registered as follows

.. literalinclude:: ../../planners/rrtPlanner.py
  :start-after: start register
  :end-before: finish register


Bi-RRT*
---------------------

.. image:: ../images/birrt.gif

.. autoclass:: planners.birrtPlanner.BiRRTPlanner
  :members:
  :private-members:
  :show-inheritance:

The sampler is registered as follows

.. literalinclude:: ../../samplers/birrtSampler.py
  :start-after: start register
  :end-before: finish register

and the planner is registered as follows

.. literalinclude:: ../../planners/birrtPlanner.py
  :start-after: start register
  :end-before: finish register


RRdT*
---------------------

.. image:: ../images/rrdt.gif

.. autoclass:: planners.rrdtPlanner.RRdTPlanner
  :members:
  :private-members:
  :show-inheritance:

The sampler and planner is registered as follows

.. literalinclude:: ../../planners/rrdtPlanner.py
  :start-after: start register
  :end-before: finish register

Informedrrt-RRT*
---------------------

The *Informed-RRT\** motion planning behaves similar to :class:`planners.rrtPlanner
.RRTPlanner` before a first solution is found.
After an initial solution is found, this planner uses an ellipses heuristic to speed
up convergence rate of the resulting solution.

.. image:: ../images/informedrrt.gif

The bulk of the implementation occurs in
:class:`~samplers.informedrrtSampler.InformedRRTSampler`.
The Informed-RRT* is implemented by directly deriving the base motion planner as
:class:`~planners.rrtPlanner.RRTPlanner`,
and registering its sampler as the
:class:`~samplers.informedrrtSampler.InformedRRTSampler`.

The sampler is registered as follows

.. literalinclude:: ../../samplers/informedSampler.py
  :start-after: start register
  :end-before: finish register

and the planner is registered as follows

.. literalinclude:: ../../planners/informedrrtPlanner.py
  :start-after: start register
  :end-before: finish register


PRM*
---------------------

.. autoclass:: planners.prmPlanner.PRMPlanner
  :members:
  :private-members:
  :show-inheritance:

The sampler is registered as follows

.. literalinclude:: ../../samplers/prmSampler.py
  :start-after: start register
  :end-before: finish register

and the planner is registered as follows

.. literalinclude:: ../../planners/prmPlanner.py
  :start-after: start register
  :end-before: finish register

Likelihood Planner
---------------------

Refers to :class:`samplers.likelihoodPolicySampler.LikelihoodPolicySampler` for details.

The bulk of the implementation occurs in :class:`~samplers.likelihoodPolicySampler.LikelihoodPolicySampler`.
The Informed-RRT* is implemented by directly deriving the base motion planner as :class:`~planners.rrtPlanner.RRTPlanner`,
and registering its sampler as the :class:`~samplers.likelihoodPolicySampler.LikelihoodPolicySampler`.

The sampler is registered as follows

.. literalinclude:: ../../samplers/likelihoodPolicySampler.py
  :start-after: start register
  :end-before: finish register

and the planner is registered as follows

.. literalinclude:: ../../planners/likelihoodPlanner.py
  :start-after: start register
  :end-before: finish register

Nearby Planner
---------------------

Refers to :class:`samplers.nearbyPolicySampler.NearbyPolicySampler` for details.

The sampler is registered as follows

.. literalinclude:: ../../samplers/nearbyPolicySampler.py
  :start-after: start register
  :end-before: finish register

and the planner is registered as follows

.. literalinclude:: ../../planners/nearbyPlanner.py
  :start-after: start register
  :end-before: finish register

Mouse Planner
---------------------

Refers to :class:`samplers.mouseSampler.MouseSampler` for details.

The sampler is registered as follows

.. literalinclude:: ../../samplers/mouseSampler.py
  :start-after: start register
  :end-before: finish register

and the planner is registered as follows

.. literalinclude:: ../../planners/mousePlanner.py
  :start-after: start register
  :end-before: finish register

Abstract Base Planner
---------------------

There is also a special base planner that all motion planner should be derived from.

.. autoclass:: planners.basePlanner.Planner
  :members:
  :private-members:
  :show-inheritance:
  :inherited-members:
