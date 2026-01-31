from __future__ import annotations

from typing import Dict


class Manifest:
    def __init__(self) -> None:
        self.stats: Dict[str, float] = {}

    def update(self, labels: Dict[str, float]) -> None:
        raise NotImplementedError("Implement manifest aggregation.")
