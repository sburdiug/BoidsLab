import itertools
import json
import math
import re
from pathlib import Path

TOKEN = re.compile(r"\$\{([a-zA-Z_][a-zA-Z_0-9]*)\}\Z")


def load_manifest(path):
    path = Path(path).resolve()
    data = json.loads(path.read_text(encoding="utf-8"), parse_constant=lambda x: (_ for _ in ()).throw(ValueError(x)))
    if set(data) != {"schema_version", "name", "cases"} or data["schema_version"] != 1:
        raise ValueError("invalid manifest header")
    names = set()
    for case in data["cases"]:
        if not {"name", "args", "grid"} <= case.keys() or set(case) - {"name", "args", "grid", "scene"}:
            raise ValueError("invalid case fields")
        name = case["name"]
        if not isinstance(name, str) or not re.fullmatch(r"[A-Za-z0-9_-]+", name) or name in names:
            raise ValueError("invalid or duplicate case name")
        names.add(name)
        grid, args = case["grid"], case["args"]
        if not isinstance(grid, dict) or not isinstance(args, list) or not all(isinstance(a, str) for a in args):
            raise ValueError("invalid grid or args")
        if any(x in args for x in ("--trace", "--scene", "--trace-every")):
            raise ValueError("trace and scene flags are managed by the runner")
        used = set()
        for token in args:
            match = TOKEN.fullmatch(token)
            if match:
                used.add(match.group(1))
            elif "${" in token:
                raise ValueError("placeholders must occupy a whole token")
        if used != set(grid):
            raise ValueError("grid keys must match placeholders")
        for values in grid.values():
            if not isinstance(values, list) or not values or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
                raise ValueError("grid values must be nonempty finite numbers")
            if len(set(values)) != len(values):
                raise ValueError("duplicate grid values produce duplicate trials")
        keys = sorted(grid)
        for values in itertools.product(*(grid[k] for k in keys)):
            params = dict(zip(keys, values))
            expanded = [str(params[m.group(1)]) if (m := TOKEN.fullmatch(a)) else a for a in args]
            scene = (path.parent / case["scene"]).resolve() if case.get("scene") else None
            yield {"name": name, "params": params, "args": expanded, "scene": scene}
