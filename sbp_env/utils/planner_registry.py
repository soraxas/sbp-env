from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Callable, Dict, Type, TYPE_CHECKING

if TYPE_CHECKING:
    from ..planners.basePlanner import Planner
    from ..samplers.baseSampler import Sampler


@dataclass
class BaseDataPack:
    """
    Base data pack that is common to sampler and planner, which
    includes visualisation method for pygame
    """

    #: an unique id to identify this data-pack
    name: str
    visualise_pygame_paint_init: Optional[Callable]
    visualise_pygame_paint: Optional[Callable]
    visualise_pygame_paint_terminate: Optional[Callable]
    visualise_klampt_paint_init: Optional[Callable]
    visualise_klampt_paint: Optional[Callable]
    visualise_klampt_paint_terminate: Optional[Callable]


@dataclass
class PlannerDataPack(BaseDataPack):
    """
    Data pack to stores planner class and the sampler with `sample_id` to use
    """

    #: the class to construct the planner
    planner_class: Type[Planner]
    #: the sampler id to associate to this planner
    sampler_id: str


@dataclass
class SamplerDataPack(BaseDataPack):
    """
    Data pack to store sampler class
    """

    #: the class to construct the sampler
    sampler_class: Type[Sampler]


#: the registry to store a dictionary to map a friendly name to a planner data pack
PLANNERS: Dict[str, PlannerDataPack] = {}
#: the registry to store a dictionary to map a friendly name to a sampler data pack
SAMPLERS: Dict[str, SamplerDataPack] = {}


def register_planner(
    planner_id: str,
    planner_class: Type["Planner"],
    sampler_id: str,
    visualise_pygame_paint_init: Optional[Callable] = None,
    visualise_pygame_paint: Optional[Callable] = None,
    visualise_pygame_paint_terminate: Optional[Callable] = None,
    visualise_klampt_paint_init: Optional[Callable] = None,
    visualise_klampt_paint: Optional[Callable] = None,
    visualise_klampt_paint_terminate: Optional[Callable] = None,
) -> None:
    """Register a planner to make it available for planning.

    :param planner_id: the unique planner id for this registering planner
    :param planner_class: the planner class
    :param sampler_id: sampler id to construct
    :param visualise_pygame_paint_init: the paint function for pygame,
        during initialisation
    :param visualise_pygame_paint: the paint function for pygame
    :param visualise_pygame_paint_terminate: the paint function for pygame,
        when terminating
    :param visualise_klampt_paint_init: the paint function for klampt,
        during initialisation
    :param visualise_klampt_paint: the paint function for klampt
    :param visualise_klampt_paint_terminate: the paint function for klampt,
        when terminating

    """
    from ..planners.basePlanner import Planner

    if planner_id in PLANNERS:
        raise ValueError(f"A planner with name '{planner_id}' already exists!")
    if not issubclass(planner_class, Planner):
        raise TypeError(
            f"The given class '{planner_class}' must derived from base "
            f"type '{Planner.__name__}'!"
        )

    PLANNERS[planner_id] = PlannerDataPack(
        name=planner_id,
        visualise_pygame_paint_init=visualise_pygame_paint_init,
        visualise_pygame_paint=visualise_pygame_paint,
        visualise_pygame_paint_terminate=visualise_pygame_paint_terminate,
        visualise_klampt_paint_init=visualise_klampt_paint_init,
        visualise_klampt_paint=visualise_klampt_paint,
        visualise_klampt_paint_terminate=visualise_klampt_paint_terminate,
        planner_class=planner_class,
        sampler_id=sampler_id,
    )


def register_sampler(
    sampler_id: str,
    sampler_class: Type["Sampler"],
    visualise_pygame_paint_init: Optional[Callable] = None,
    visualise_pygame_paint: Optional[Callable] = None,
    visualise_pygame_paint_terminate: Optional[Callable] = None,
    visualise_klampt_paint_init: Optional[Callable] = None,
    visualise_klampt_paint: Optional[Callable] = None,
    visualise_klampt_paint_terminate: Optional[Callable] = None,
) -> None:
    """Register a sampler to make it available for planning.

    :param sampler_id: the unique id for this sampler
    :param sampler_class: the class to construct this sampler
    :param visualise_pygame_paint_init: the paint function for pygame,
        during initialisation
    :param visualise_pygame_paint: the paint function for pygame
    :param visualise_pygame_paint_terminate: the paint function for pygame,
        when terminating
    :param visualise_klampt_paint_init: the paint function for klampt,
        during initialisation
    :param visualise_klampt_paint: the paint function for klampt
    :param visualise_klampt_paint_terminate: the paint function for klampt,
        when terminating

    """
    from ..samplers.baseSampler import Sampler

    if sampler_id in SAMPLERS:
        raise ValueError(f"A sampler with name '{sampler_id}' already exists!")
    if not issubclass(sampler_class, Sampler):
        raise TypeError(
            f"The given class '{sampler_class}' must derived from base "
            f"type '{Sampler.__name__}'!"
        )

    SAMPLERS[sampler_id] = SamplerDataPack(
        name=sampler_id,
        visualise_pygame_paint_init=visualise_pygame_paint_init,
        visualise_pygame_paint=visualise_pygame_paint,
        visualise_pygame_paint_terminate=visualise_pygame_paint_terminate,
        visualise_klampt_paint_init=visualise_klampt_paint_init,
        visualise_klampt_paint=visualise_klampt_paint,
        visualise_klampt_paint_terminate=visualise_klampt_paint_terminate,
        sampler_class=sampler_class,
    )
