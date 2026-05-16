from .base import GridPlanner
from .dijkstra import DijkstraPlanner

__all__ = ['GridPlanner', 'DijkstraPlanner', 'make_planner']


def make_planner(name: str) -> GridPlanner:
    key = name.strip().lower()
    if key in ('dijkstra', 'default', ''):
        return DijkstraPlanner()
    raise ValueError(f'Unknown planner algorithm: {name!r} (supported: dijkstra)')
