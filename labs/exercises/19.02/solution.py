"""19.02 — reference solution. Mirrors ``Param.schema`` and ``validate_arguments`` in
``19-llm-robot-agents/code/robot_agent/skills.py``.
"""

from __future__ import annotations

import math
import re
from collections.abc import Mapping
from typing import Any

JsonDict = dict[str, Any]


def param_schema(
    name: str,
    type: str,
    description: str,
    unit: str | None = None,
    minimum: float | None = None,
    maximum: float | None = None,
    enum: tuple[str, ...] | None = None,
    pattern: str | None = None,
    default: Any = None,
) -> JsonDict:
    """One parameter declaration -> its JSON Schema fragment, description included."""
    s: JsonDict = {"type": type}
    desc = description
    if unit:
        s["x-unit"] = unit
        desc += f" Unit: {unit}."
    if minimum is not None:
        s["minimum"] = minimum
    if maximum is not None:
        s["maximum"] = maximum
    if minimum is not None or maximum is not None:
        desc += f" Range: [{minimum}, {maximum}]."
    if enum is not None:
        s["enum"] = list(enum)
    if pattern is not None:
        s["pattern"] = pattern
    if default is not None:
        s["default"] = default
        desc += f" Default: {default}."
    s["description"] = desc
    return s


def validate_arguments(schema: Mapping[str, Any], args: Any) -> list[str]:
    """Every reason this call must not reach the robot."""
    if not isinstance(args, Mapping):
        return [f"arguments must be an object, got {type(args).__name__}"]
    errors: list[str] = []
    props: Mapping[str, Any] = schema.get("properties", {})
    for name in schema.get("required", []):
        if name not in args:
            errors.append(f"{name}: required")
    if schema.get("additionalProperties") is False:
        errors += [f"{k}: unknown parameter" for k in args if k not in props]
    for name, value in args.items():
        spec = props.get(name)
        if spec is None:
            continue
        t = spec.get("type")
        if t == "number":
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                errors.append(f"{name}: expected a number, got {value!r}")
                continue
            if not math.isfinite(value):
                errors.append(f"{name}: must be finite")
                continue
        elif t == "integer":
            if isinstance(value, bool) or not isinstance(value, int):
                errors.append(f"{name}: expected an integer, got {value!r}")
                continue
        elif t == "string":
            if not isinstance(value, str):
                errors.append(f"{name}: expected a string, got {value!r}")
                continue
        elif t == "boolean" and not isinstance(value, bool):
            errors.append(f"{name}: expected true/false, got {value!r}")
            continue
        unit = f" {spec['x-unit']}" if "x-unit" in spec else ""
        if "minimum" in spec and value < spec["minimum"]:
            errors.append(f"{name}: {value}{unit} is below the minimum {spec['minimum']}{unit}")
        if "maximum" in spec and value > spec["maximum"]:
            errors.append(f"{name}: {value}{unit} is above the maximum {spec['maximum']}{unit}")
        if "enum" in spec and value not in spec["enum"]:
            errors.append(f"{name}: {value!r} is not one of {spec['enum']}")
        if "pattern" in spec and isinstance(value, str) and re.fullmatch(spec["pattern"], value) is None:
            errors.append(f"{name}: {value!r} does not match {spec['pattern']}")
    return errors
