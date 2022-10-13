.. _motion_planners_page:

Motion Planners
===============

The following are all of the available motion planners in ``sbp-env``.


RRT*
---------------------

.. image:: ../images/rrt.gif

.. autoclass:: sbp_env.planners.rrtPlanner.RRTPlanner
  :members:
  :private-members:
  :show-inheritance:

The sampler is registered as follows

.. literalinclude:: ../../sbp_env/samplers/randomPolicySampler.py
  :start-after: start register
  :end-before: finish register

and the planner is registered as follows

.. literalinclude:: ../../sbp_env/planners/rrtPlanner.py
  :start-after: start register
  :end-before: finish register


Bi-RRT*
---------------------

.. image:: ../images/birrt.gif

.. autoclass:: sbp_env.planners.birrtPlanner.BiRRTPlanner
  :members:
  :private-members:
  :show-inheritance:

The sampler is registered as follows

.. literalinclude:: ../../sbp_env/samplers/birrtSampler.py
  :start-after: start register
  :end-before: finish register

and the planner is registered as follows

.. literalinclude:: ../../sbp_env/planners/birrtPlanner.py
  :start-after: start register
  :end-before: finish register


RRdT*
---------------------

.. image:: ../images/rrdt.gif

.. autoclass:: sbp_env.planners.rrdtPlanner.RRdTPlanner
  :members:
  :private-members:
  :show-inheritance:

The sampler and planner is registered as follows

.. literalinclude:: ../../sbp_env/planners/rrdtPlanner.py
  :start-after: start register
  :end-before: finish register

Informedrrt-RRT*
---------------------

The *Informed-RRT\** motion planning behaves similar to :class:`sbp_env.planners.rrtPlanner
.RRTPlanner` before a first solution is found.
After an initial solution is found, this planner uses an ellipses heuristic to speed
up convergence rate of the resulting solution.

.. image:: ../images/informedrrt.gif

The bulk of the implementation occurs in
:class:`~sbp_env.samplers.informedrrtSampler.InformedRRTSampler`.
The Informed-RRT* is implemented by directly deriving the base motion planner as
:class:`~sbp_env.planners.rrtPlanner.RRTPlanner`,
and registering its sampler as the
:class:`~sbp_env.samplers.informedrrtSampler.InformedRRTSampler`.

The sampler is registered as follows

.. literalinclude:: ../../sbp_env/samplers/informedSampler.py
  :start-after: start register
  :end-before: finish register

and the planner is registered as follows

.. literalinclude:: ../../sbp_env/planners/informedrrtPlanner.py
  :start-after: start register
  :end-before: finish register


PRM*
---------------------

.. autoclass:: sbp_env.planners.prmPlanner.PRMPlanner
  :members:
  :private-members:
  :show-inheritance:

The sampler is registered as follows

.. literalinclude:: ../../sbp_env/samplers/prmSampler.py
  :start-after: start register
  :end-before: finish register

and the planner is registered as follows

.. literalinclude:: ../../sbp_env/planners/prmPlanner.py
  :start-after: start register
  :end-before: finish register

Likelihood Planner
---------------------

Refers to :class:`sbp_env.samplers.likelihoodPolicySampler.LikelihoodPolicySampler` for details.

The bulk of the implementation occurs in :class:`~sbp_env.samplers.likelihoodPolicySampler.LikelihoodPolicySampler`.
The Informed-RRT* is implemented by directly deriving the base motion planner as :class:`~sbp_env.planners.rrtPlanner.RRTPlanner`,
and registering its sampler as the :class:`~sbp_env.samplers.likelihoodPolicySampler.LikelihoodPolicySampler`.

The sampler is registered as follows

.. literalinclude:: ../../sbp_env/samplers/likelihoodPolicySampler.py
  :start-after: start register
  :end-before: finish register

and the planner is registered as follows

.. literalinclude:: ../../sbp_env/planners/likelihoodPlanner.py
  :start-after: start register
  :end-before: finish register

Nearby Planner
---------------------

Refers to :class:`sbp_env.samplers.nearbyPolicySampler.NearbyPolicySampler` for details.

The sampler is registered as follows

.. literalinclude:: ../../sbp_env/samplers/nearbyPolicySampler.py
  :start-after: start register
  :end-before: finish register

and the planner is registered as follows

.. literalinclude:: ../../sbp_env/planners/nearbyPlanner.py
  :start-after: start register
  :end-before: finish register

Mouse Planner
---------------------

Refers to :class:`sbp_env.samplers.mouseSampler.MouseSampler` for details.

The sampler is registered as follows

.. literalinclude:: ../../sbp_env/samplers/mouseSampler.py
  :start-after: start register
  :end-before: finish register

and the planner is registered as follows

.. literalinclude:: ../../sbp_env/planners/mousePlanner.py
  :start-after: start register
  :end-before: finish register

Abstract Base Planner
---------------------

There is also a special base planner that all motion planner should be derived from.

.. autoclass:: sbp_env.planners.basePlanner.Planner
  :members:
  :private-members:
  :show-inheritance:
  :inherited-members:
