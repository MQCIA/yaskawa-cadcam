# Open-source Yaskawa / Motoman motion stacks

Ready-made packages you can clone for robot motion, IK, and controller I/O.
None of these replace a certified integrator sign-off on a live DX200/YRC cell.

## Official / ROS-Industrial (download these first)

| Project | License | What you get |
|---------|---------|--------------|
| [ros-industrial/motoman](https://github.com/ros-industrial/motoman) (`noetic-devel`) | Apache-2.0 | URDFs + meshes for AR/MA/GP…, `motoman_driver` (ROS 1), MoveIt configs (e.g. `motoman_ma2010_moveit_config`) |
| [Yaskawa-Global/motoros2](https://github.com/Yaskawa-Global/motoros2) | Yaskawa / see repo | **ROS 2** MotoPlus node: `FollowJointTrajectory`, joint state — the supported path to drive a real Motoman from ROS 2 |
| [Yaskawa-Global/motoros2_interfaces](https://github.com/Yaskawa-Global/motoros2_interfaces) | see repo | Msg/srv/action definitions for MotoROS2 |
| [ros-industrial/industrial_core](https://github.com/ros-industrial/industrial_core) | BSD | Simple message / trajectory download helpers (ROS 1 era) |

Clone examples:

```bash
git clone -b noetic-devel https://github.com/ros-industrial/motoman.git
git clone https://github.com/Yaskawa-Global/motoros2.git
git clone https://github.com/Yaskawa-Global/motoros2_interfaces.git
```

MoveIt planning (ROS 1 MA2010):

```bash
# after building the motoman workspace
roslaunch motoman_ma2010_moveit_config moveit_planning_execution.launch
```

## Kinematics / planning libraries (no controller required)

| Project | Notes |
|---------|--------|
| [petercorke/robotics-toolbox-python](https://github.com/petercorke/robotics-toolbox-python) | Already used in this repo’s FastAPI IK |
| [Phylliade/ikpy](https://github.com/Phylliade/ikpy) | Lightweight IK on URDF chains (used in public AR2010 weld demos) |
| [MoveIt 2](https://github.com/moveit/moveit2) | Collision-aware joint trajectories → feed MotoROS2 FJT |

## Welding / offline-programming references

| Resource | Notes |
|----------|--------|
| [Sai Navaneet — AR2010 weld automation](https://sainavaneet.github.io/portfolio/source/projects/AR2010-Weld/) | Geometry → seams → torch TCP → IK → YRC1000 host control (portfolio; not a drop-in package) |
| Verbotics Weld workcells (`.vbmodel` = zip) | Proprietary app; cells unpack to URDF + `model.json` (TCP) for study |

## What this CAD/CAM app already vendors

- MA2010 / AR2010 visual STLs from `motoman` (Apache-2.0) under `frontend/public/models/`
- Production **TOOL 0** from the DX200 dump: `frontend/lib/tool0.ts` / `docs/CELL.md`
- Browser CCD IK + DX200 `.JBI` postprocessor (pulse path when joints are available)

## Recommended next download for “ready motion”

1. **Simulation / planning:** `motoman` + MoveIt config for your exact arm.  
2. **Live robot (ROS 2):** `motoros2` on the controller + MoveIt 2 on the PC.  
3. **Keep** this web app’s TOOL0 / pulse tables synced with `TOOL.CND` and `RC.PRM` from the pendant dump.
