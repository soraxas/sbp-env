Misc Classes
============

The following are classes and functions of which the details is not essential to use `sbp-env`.


Data structure
--------------

.. autoclass:: utils.common.Tree
  :members:
  :private-members:
  :inherited-members:

.. autoclass:: utils.common.Node
  :members:
  :private-members:
  :inherited-members:


Randomness
----------------------------

The following are the random methods supported in `sbp-env`.

.. autodata:: randomness.SUPPORTED_RANDOM_METHODS


Drawing random samplings
^^^^^^^^^^^^^^^^^^^^^^^^

The :class:`randomness.RandomnessManager:` is used by most samplers to draw random numbers.

.. autoclass:: randomness.RandomnessManager
  :members:
  :private-members:
  :inherited-members:


Drawing normally distributed samplings
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The :class:`randomness.NormalRandomnessManager:` is used by :class:`planners.rrdtPlanner.RRdTSampler` to draw from a von Mises distribution.

.. autoclass:: randomness.NormalRandomnessManager
  :members:
  :private-members:
  :inherited-members:


Planners and Samplers Registry
-----------------------------

The following function is used to register a new custom *planner*.

.. autofunction:: utils.planner_registry.register_planner

The following function is used to register a new custom *sampler*.

.. autofunction:: utils.planner_registry.register_sampler


Registry data structure
^^^^^^^^^^^^^^^^^^^^^^^

.. autoclass:: utils.planner_registry.PlannerDataPack
  :members:
  :private-members:
  :inherited-members:

.. autoclass:: utils.planner_registry.SamplerDataPack
  :members:
  :private-members:
  :inherited-members:




