"""Path helpers for repository resources."""
from __future__ import annotations

from pathlib import Path


def get_repo_root() -> Path:
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "persistent").exists():
            return parent
    if len(here.parents) > 5:
        return here.parents[5]
    return here.parent
