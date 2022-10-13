Misc Classes
============

The following are classes and functions of which the details is not essential to use `sbp-env`.


Data structure
--------------

.. autoclass:: sbp_env.utils.common.Tree
  :members:
  :private-members:
  :inherited-members:

.. autoclass:: sbp_env.utils.common.Node
  :members:
  :private-members:
  :inherited-members:

.. autoclass:: sbp_env.utils.common.PlanningOptions
  :members:
  :private-members:
  :inherited-members:

Randomness
----------------------------

The following are the random methods supported in `sbp-env`.

.. autodata:: sbp_env.randomness.SUPPORTED_RANDOM_METHODS


Drawing random samplings
^^^^^^^^^^^^^^^^^^^^^^^^

The :class:`sbp_env.randomness.RandomnessManager:` is used by most samplers to draw random numbers.

.. autoclass:: sbp_env.randomness.RandomnessManager
  :members:
  :private-members:
  :inherited-members:


Drawing normally distributed samplings
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The :class:`sbp_env.randomness.NormalRandomnessManager:` is used by :class:`sbp_env.planners.rrdtPlanner.RRdTSampler` to draw from a von Mises distribution.

.. autoclass:: sbp_env.randomness.NormalRandomnessManager
  :members:
  :private-members:
  :inherited-members:


Planners and Samplers Registry
------------------------------

The following function is used to register a new custom *planner*.

.. autofunction:: sbp_env.utils.planner_registry.register_planner

The following function is used to register a new custom *sampler*.

.. autofunction:: sbp_env.utils.planner_registry.register_sampler


Registry data structure
^^^^^^^^^^^^^^^^^^^^^^^

.. autoclass:: sbp_env.utils.planner_registry.PlannerDataPack
  :members:
  :private-members:
  :inherited-members:

.. autoclass:: sbp_env.utils.planner_registry.SamplerDataPack
  :members:
  :private-members:
  :inherited-members:




