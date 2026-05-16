from __future__ import annotations

from typing import List, Protocol, Tuple

import numpy as np

GridCoord = Tuple[int, int]


class GridPlanner(Protocol):
    """Contract for grid search on an occupancy grid (Nav2-style int8 costs)."""

    def plan(
        self,
        occ: np.ndarray,
        start: GridCoord,
        goal: GridCoord,
        *,
        lethal_cost: int,
        allow_unknown: bool,
    ) -> List[GridCoord] | None:
        """Return cell path start→goal inclusive, or None if unreachable."""
