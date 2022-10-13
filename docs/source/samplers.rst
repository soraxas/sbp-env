.. _samplers_page:

C-space Samplers
================

The following are samplers that generates sampled configurations :math:`q \in C` in
*C-Space* with different strategies.

Random Policy Sampler
---------------------

.. autoclass:: sbp_env.samplers.randomPolicySampler.RandomPolicySampler
  :members:
  :private-members:
  :show-inheritance:

.. autodata:: sbp_env.randomness.SUPPORTED_RANDOM_METHODS
    :noindex:

Bidirectional Random Sampler
----------------------------

.. autoclass:: sbp_env.samplers.birrtSampler.BiRRTSampler
  :members:
  :private-members:
  :show-inheritance:


Informed Sampler
---------------------

.. autoclass:: sbp_env.samplers.informedSampler.InformedSampler
  :members:
  :private-members:
  :show-inheritance:


RRdT Particle Sampler
---------------------

.. autoclass:: sbp_env.planners.rrdtPlanner.RRdTSampler
  :members:
  :private-members:
  :show-inheritance:


PRM Sampler
---------------------

.. autoclass:: sbp_env.samplers.prmSampler.PRMSampler
  :members:
  :private-members:
  :show-inheritance:

Likelihood Sampler
---------------------

.. autoclass:: sbp_env.samplers.likelihoodPolicySampler.LikelihoodPolicySampler
  :members:
  :private-members:
  :show-inheritance:

Nearby Sampler
---------------------

.. autoclass:: sbp_env.samplers.nearbyPolicySampler.NearbyPolicySampler
  :members:
  :private-members:
  :show-inheritance:

Mouse Sampler
---------------------

.. autoclass:: sbp_env.samplers.mouseSampler.MouseSampler
  :members:
  :private-members:
  :show-inheritance:

Abstract Base Sampler
---------------------

There is also a special base Sampler that all motion planner should be derived from.

.. autoclass:: sbp_env.samplers.baseSampler.Sampler
  :members:
