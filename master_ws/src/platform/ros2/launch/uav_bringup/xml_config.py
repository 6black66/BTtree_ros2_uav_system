"""Typed node XML configuration shared by launch entrypoints."""
from pathlib import Path
import math
import re
import xml.etree.ElementTree as ET


def load_node_config(config_dir, name, variables):
    path = Path(config_dir) / "node" / name / f"{name}.xml"
    root = ET.parse(path).getroot()
    if root.tag != "node" or root.attrib.get("name") != name:
        raise ValueError(f"{path}: expected <node name='{name}'>")
    parameters = {}
    for entry in root:
        if entry.tag != "param" or set(entry.attrib) != {"name", "type", "value"}:
            raise ValueError(f"{path}: expected param name/type/value")
        key, kind, value = (entry.attrib[k] for k in ("name", "type", "value"))
        if not key or key == "config_file" or key in parameters:
            raise ValueError(f"{path}: duplicate, empty or reserved parameter {key}")
        def substitute(match):
            variable = match.group(1)
            if variable not in variables:
                raise ValueError(f"{path}: unknown variable {variable}")
            return str(variables[variable])
        value = re.sub(r"\$\{([^}]+)\}", substitute, value)
        if kind == "bool" and value in ("true", "false"):
            value = value == "true"
        elif kind == "int":
            if not re.fullmatch(r"[+-]?\d+", value):
                raise ValueError(f"{path}: invalid integer {key}")
            value = int(value)
            if not -(2**63) <= value < 2**63:
                raise ValueError(f"{path}: integer overflow {key}")
        elif kind == "double":
            value = float(value)
            if not math.isfinite(value):
                raise ValueError(f"{path}: nonfinite double {key}")
        elif kind != "string":
            raise ValueError(f"{path}: invalid type/value for {key}")
        parameters[key] = value
    return parameters

