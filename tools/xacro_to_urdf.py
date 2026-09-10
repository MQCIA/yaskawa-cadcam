#!/usr/bin/env python3
"""
Minimal xacro -> plain URDF converter for the ROS-Industrial `motoman`
support packages, so the URDFs can be served statically and loaded in the
browser with `urdf-loader` (no ROS toolchain required).

It handles exactly the subset those packages use:
  * a single ``<xacro:macro name=... params="prefix">`` wrapper
  * ``${prefix}`` substitution (-> "")
  * ``${...}`` math expressions with ``pi`` and ``radians()``
  * the ``<xacro:include .../>`` of common_materials (dropped)
  * ``<xacro:material_yaskawa_blue/>`` (-> a concrete <material>)
  * ``package://<pkg>/meshes/...`` mesh paths rewritten to relative paths,
    with every ``/collision/`` folder redirected to ``/visual/`` (we only
    vendor visual meshes).

Usage:
  python xacro_to_urdf.py <macro.xacro> <robot_name> <mesh_prefix> <out.urdf>

``mesh_prefix`` is prepended to the relative mesh path, e.g. "visual/" for the
AR2010 (meshes live in <model>/visual/) or "" when the on-disk layout already
matches the tail of the package path.
"""

import math
import re
import sys

YASKAWA_BLUE = '<material name="yaskawa_blue"><color rgba="0.0 0.1804 0.5451 1.0"/></material>'


def eval_expr(match: re.Match) -> str:
    expr = match.group(1)
    value = eval(expr, {"__builtins__": {}}, {"pi": math.pi, "radians": math.radians})
    return repr(value)


def convert(macro_path: str, robot_name: str, out_path: str) -> None:
    with open(macro_path, "r", encoding="utf-8") as f:
        text = f.read()

    # Drop the xacro include of common materials.
    text = re.sub(r"<xacro:include[^>]*/>", "", text)
    # Concrete material in place of the macro call.
    text = text.replace("<xacro:material_yaskawa_blue/>", YASKAWA_BLUE)
    # Remove the macro open/close tags, keep the body.
    text = re.sub(r"<xacro:macro[^>]*>", "", text)
    text = text.replace("</xacro:macro>", "")
    # prefix -> empty
    text = text.replace("${prefix}", "")
    # Evaluate ${...} math expressions.
    text = re.sub(r"\$\{([^}]*)\}", eval_expr, text)

    # Rewrite mesh paths: package://<pkg>/meshes/<...> -> <...>, and any
    # /collision/ segment -> /visual/ (only visual meshes are vendored).
    def rewrite_mesh(m: re.Match) -> str:
        rel = m.group(1)  # everything after .../meshes/
        # Normalise to start at the visual/ or collision/ folder, dropping any
        # extra model-name subfolder (e.g. motopos_d500/visual/ -> visual/).
        for key in ("visual/", "collision/"):
            idx = rel.find(key)
            if idx != -1:
                rel = rel[idx:]
                break
        rel = rel.replace("collision/", "visual/")
        return f'filename="{rel}"'

    text = re.sub(
        r'filename="package://[^"]*?/meshes/([^"]+)"', rewrite_mesh, text
    )

    # Ensure a single <robot name="..."> wrapper.
    text = re.sub(r"<robot[^>]*>", "", text)
    text = text.replace("</robot>", "")
    text = text.strip()
    # Drop the leading xml declaration if present in the body.
    text = re.sub(r"<\?xml[^>]*\?>", "", text).strip()

    urdf = (
        '<?xml version="1.0" ?>\n'
        f'<robot name="{robot_name}">\n{text}\n</robot>\n'
    )
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(urdf)
    print(f"wrote {out_path}")


if __name__ == "__main__":
    macro, name, out = sys.argv[1], sys.argv[2], sys.argv[3]
    convert(macro, name, out)
