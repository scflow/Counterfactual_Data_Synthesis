from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict

import yaml


def load_yaml(path: str | Path) -> Dict[str, Any]:
    """Load YAML config into a dict."""
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def deep_merge(base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
    merged: Dict[str, Any] = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = deep_merge(merged[key], value)
        else:
            merged[key] = value
    return merged


@dataclass
class AppConfig:
    raw: Dict[str, Any]

    @classmethod
    def from_files(cls, *paths: str | Path) -> "AppConfig":
        merged: Dict[str, Any] = {}
        for path in paths:
            merged = deep_merge(merged, load_yaml(path))
        return cls(raw=merged)
