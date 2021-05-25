from dataclasses import dataclass
from typing import Optional, Callable, Dict, Type, TYPE_CHECKING

if TYPE_CHECKING:
    from planners.basePlanner import Planner
    from samplers.baseSampler import Sampler


@dataclass
class BaseDataPack:
    name: str
    visualise_pygame_paint_init: Optional[Callable]
    visualise_pygame_paint: Optional[Callable]
    visualise_pygame_paint_terminate: Optional[Callable]


@dataclass
class PlannerDataPack(BaseDataPack):
    planner_class: Type['Planner']
    sampler_id: str


@dataclass
class SamplerDataPack(BaseDataPack):
    sampler_class: Type['Sampler']


# the registry to store all registered planners and samplers
PLANNERS: Dict[str, PlannerDataPack] = {}
SAMPLERS: Dict[str, SamplerDataPack] = {}


def register_planner(
        planner_id: str,
        planner_class: Type['Planner'],
        sampler_id: str,
        visualise_pygame_paint_init: Optional[Callable] = None,
        visualise_pygame_paint: Optional[Callable] = None,
        visualise_pygame_paint_terminate: Optional[Callable] = None,
) -> None:
    if planner_id in PLANNERS:
        raise ValueError(f"A planner with name '{planner_id}' already exists!")

    PLANNERS[planner_id] = PlannerDataPack(
        name=planner_id,
        visualise_pygame_paint_init=visualise_pygame_paint_init,
        visualise_pygame_paint=visualise_pygame_paint,
        visualise_pygame_paint_terminate=visualise_pygame_paint_terminate,
        planner_class=planner_class,
        sampler_id=sampler_id,
    )


def register_sampler(
        sampler_id: str,
        sampler_class: Type['Sampler'],
        visualise_pygame_paint_init: Optional[Callable] = None,
        visualise_pygame_paint: Optional[Callable] = None,
        visualise_pygame_paint_terminate: Optional[Callable] = None,
) -> None:
    if sampler_id in SAMPLERS:
        raise ValueError(f"A sampler with name '{sampler_id}' already exists!")

    SAMPLERS[sampler_id] = SamplerDataPack(
        name=sampler_id,
        visualise_pygame_paint_init=visualise_pygame_paint_init,
        visualise_pygame_paint=visualise_pygame_paint,
        visualise_pygame_paint_terminate=visualise_pygame_paint_terminate,
        sampler_class=sampler_class,
    )
