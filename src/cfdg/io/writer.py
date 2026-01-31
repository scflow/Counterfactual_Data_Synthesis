from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, List


class OutputWriter:
    def __init__(self, output_root: str) -> None:
        self.output_root = Path(output_root)

    def write_scene(self, scene_token: str, trajectory: List[Dict[str, Any]], labels: Dict[str, Any], meta: Dict[str, Any]) -> None:
        scene_dir = self.output_root / "scenes" / scene_token
        scene_dir.mkdir(parents=True, exist_ok=True)

        with open(scene_dir / "meta.json", "w", encoding="utf-8") as f:
            json.dump(meta, f, ensure_ascii=False, indent=2)
        with open(scene_dir / "labels.json", "w", encoding="utf-8") as f:
            json.dump(labels, f, ensure_ascii=False, indent=2)

        try:
            import pandas as pd

            df = pd.DataFrame(trajectory)
            df.to_parquet(scene_dir / "trajectory.parquet", index=False)
        except Exception:
            with open(scene_dir / "trajectory.json", "w", encoding="utf-8") as f:
                json.dump(trajectory, f, ensure_ascii=False)
