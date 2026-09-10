# 3D robot & positioner models — sources and how to add them

The app can load articulated **URDF** models (per-link STL meshes with working
joints) served from `frontend/public/models/`. Two example Yaskawa models are
bundled and load out of the box:

| Model | Type | Folder | Source |
|-------|------|--------|--------|
| MOTOMAN-AR2010 | 6-axis welding robot | `frontend/public/models/ar2010/` | ROS-Industrial `motoman_ar2010_support` |
| MotoPos D500 | 2-axis rotary positioner | `frontend/public/models/motopos_d500/` | ROS-Industrial `motoman_motopos_d500_support` |

> The D500 is bundled as an **example rotary positioner**. Your actual **H1000D**
> can be added the same way once you have its model (see below).

## Where to get official Yaskawa models

### 1. ROS-Industrial `motoman` (used here — free, no login)
The most practical source for developers who need URDF + meshes with joints:

- Repo: <https://github.com/ros-industrial/motoman> (branch `noetic-devel`), license **Apache-2.0**.
- Robot packages: `motoman_ar2010_support`, `motoman_ma2010_support`,
  `motoman_gp7_support`, `motoman_gp8_support`, `motoman_mh5_support`, …
- Positioner packages: `motoman_motopos_d500_support`,
  `motoman_motopos_mh1655_support`.
- Each package has `urdf/*_macro.xacro` (link/joint definitions) and
  `meshes/visual/*.stl` (geometry).

### 2. Official Yaskawa CAD (STEP / IGES / DXF) — for exact geometry
These are the authoritative CAD files (require a free member login):

- **e-mechatronics.com** (Yaskawa Japan) product CAD pages, e.g. the AR900 page
  lists model `YR-1-06VX7-A00`, drawing `HW1384163`, with STEP/IGES revisions:
  <https://www.e-mechatronics.com/product/robot/arc/lineup/ar900/cad.html>
- **Yaskawa Downloads portal**: <https://www.yaskawa.com/downloads/-/document/downloads>
  → filter *Document Sub Type* = "CAD Drawings" or "Solid Model Files".
- Regional product pages (e.g. `yaskawa.co.uk` / `yaskawa.eu.com`) → product
  detail → "2D/3D CAD/CAE" download links.
- **MotoSim Model Library** (registration required) for MotoSim-ready models —
  this is also the reference to validate kinematics against.

STEP/IGES are B-Rep solids without joint articulation. To use them here you must
tessellate to mesh (glTF/STL) and, for jointed motion, split per link — a URDF
from source (1) is usually the faster path to an articulated model.

### 3. Yaskawa developer resources
- Yaskawa "for developers" / MotoPlus SDK and API docs (controller-side
  application development). For 3D visualisation models specifically, sources
  (1) and (2) above are what you want.

## How the bundled models were produced

The ROS xacro files were converted to plain, browser-loadable URDF with
[`tools/xacro_to_urdf.py`](../tools/xacro_to_urdf.py), which:
- instantiates the `<xacro:macro>` and drops `${prefix}`,
- evaluates `${…}` math (`pi`, `radians()`),
- replaces the materials include with a concrete Yaskawa-blue material,
- rewrites `package://…/meshes/{visual,collision}/…` to relative `visual/…`
  paths (only visual meshes are vendored).

## Adding a new model (e.g. your exact AR model or the H1000D)

### From ROS-Industrial motoman
```bash
PKG=motoman_ar1440_support   # example
RAW=https://raw.githubusercontent.com/ros-industrial/motoman/noetic-devel
mkdir -p frontend/public/models/<id>/visual
# 1) meshes
for m in base_link link_1_s link_2_l link_3_u link_4_r link_5_b link_6_t; do
  curl -s "$RAW/$PKG/meshes/visual/$m.stl" -o frontend/public/models/<id>/visual/$m.stl
done
# 2) xacro -> urdf
curl -s "$RAW/$PKG/urdf/<name>_macro.xacro" -o /tmp/<name>_macro.xacro
python3 tools/xacro_to_urdf.py /tmp/<name>_macro.xacro <id> frontend/public/models/<id>/<id>.urdf
```
Then register it in `frontend/lib/models.ts` (`ROBOT_MODELS` or
`POSITIONER_MODELS`) with its `url` and joint-name map.

### From official STEP/CAD
Convert STEP → glTF/GLB (FreeCAD, Blender, or `assimp`), place the file under
`frontend/public/models/`, and load it. For articulated motion, export one mesh
per link and build a small URDF referencing them (same as above).

## Supported mesh formats
The URDF loader here wires an **STL** mesh callback (ROS-Industrial meshes are
STL). Collada (`.dae`) or glTF links can be supported by extending the
`loadMeshCb` in `frontend/components/UrdfModel.tsx`.
