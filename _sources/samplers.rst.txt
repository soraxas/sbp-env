C-space Samplers
================

The following are samplers that generates sampled configurations :math:`q \in C` in
*C-Space* with different strategies.

Random Policy Sampler
---------------------

.. autoclass:: samplers.randomPolicySampler.RandomPolicySampler
  :members:
  :private-members:
  :show-inheritance:

.. autodata:: randomness.SUPPORTED_RANDOM_METHODS
    :noindex:

Bidirectional Random Sampler
----------------------------

.. autoclass:: samplers.birrtSampler.BiRRTSampler
  :members:
  :private-members:
  :show-inheritance:


Informed Sampler
---------------------

.. autoclass:: samplers.informedSampler.InformedSampler
  :members:
  :private-members:
  :show-inheritance:


RRdT Particle Sampler
---------------------

.. autoclass:: planners.rrdtPlanner.RRdTSampler
  :members:
  :private-members:
  :show-inheritance:


PRM Sampler
---------------------

.. autoclass:: samplers.prmSampler.PRMSampler
  :members:
  :private-members:
  :show-inheritance:

Likelihood Sampler
---------------------

.. autoclass:: samplers.likelihoodPolicySampler.LikelihoodPolicySampler
  :members:
  :private-members:
  :show-inheritance:

Nearby Sampler
---------------------

.. autoclass:: samplers.nearbyPolicySampler.NearbyPolicySampler
  :members:
  :private-members:
  :show-inheritance:

Mouse Sampler
---------------------

.. autoclass:: samplers.mouseSampler.MouseSampler
  :members:
  :private-members:
  :show-inheritance:

Abstract Base Sampler
---------------------

There is also a special base Sampler that all motion planner should be derived from.

.. autoclass:: samplers.baseSampler.Sampler
  :members:
