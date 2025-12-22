from dataclasses import dataclass
from typing import Any


@dataclass
class StoredParameter:
    key: str
    value: Any
    updated_at: float
