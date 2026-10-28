# Sim-Ready Diner — MuJoCo Scene

![demo](media/demo.gif)

A physically valid diner interior for robotics simulation, built in Blender and exported to MuJoCo.
Every asset has a separate visual mesh and a hand-built convex collision setup with realistic mass.
A simple test agent can be driven through the scene to verify navigation, clearances and object behavior.

## What's inside

- **~45 assets**: bar counter, 11 bar stools, sofa, 7 tables, 7 chairs, plates, forks, knife, mugs, walls
- **Convex collision decomposition** per asset (e.g. a chair = 13 convex parts, so the space under the seat stays free)
- **Realistic mass distribution**: e.g. tables carry 20 of 25 kg in the cast-iron base, so they don't tip over easily
- **Static vs dynamic bodies**: built-in elements (walls, counter, bolted stools, sofa) are fixed; furniture and tableware are free bodies
- **Blender → MuJoCo exporter** (`scripts/export_room.py`) driven by simple naming conventions
- **Keyboard teleop** (`scripts/drive.py`) for a test agent with a force-limited speed controller

| Visual | Collision |
|---|---|
| ![visual](media/visual.png) | ![collision](media/collision.png) |

## Quick start

```bash
git clone https://github.com/<your-username>/sim-ready-diner.git
cd sim-ready-diner
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python scripts/drive.py scene/room.xml
```

**Controls**
- Arrow keys: move the agent relative to the camera view (press the same arrow again to stop)
- Mouse: left drag = rotate camera, right drag = pan, scroll = zoom
- `3`: toggle collision geometry, `2`: toggle visual meshes

## Asset conventions (Blender)

| Rule | Meaning |
|---|---|
| One collection = one asset | `chair_01`, `table_03`, ... |
| `COL_` prefix on an object | Collision part; each one becomes a single convex geom |
| Any other mesh | Visual only, no contacts, no mass |
| `STATIC_` prefix on a collection | Fixed body (walls, counter, bolted stools) |
| Custom property `mass` on a collection | Total asset mass, kg |
| Custom property `mass` on a `COL_` object | Exact mass of that part (e.g. heavy base) |
| Empty named `AGENT_START` | Spawn point of the test agent |

To re-export: open `blender/diner.blend`, run `scripts/export_room.py` from the Text Editor.
Output goes to `scene/room.xml` and `scene/assets/`.

## Physics notes

- `alignfree="true"`: meshes are exported in world space, so free-body frames are moved to each body's center of mass. Without it, small light objects (cutlery, plates) rotate around the world origin and the simulation becomes unstable.
- `timestep="0.001"` + `integrator="implicitfast"`: stable contacts for thin, light objects and a damped agent.
- `multiccd`: multiple contact points for convex-convex collisions, so plates and cutlery rest flat instead of balancing on one point.

## Project structure

blender/ diner.blend source scene
scripts/ export_room.py Blender -> MJCF exporter
drive.py keyboard teleop
scene/ room.xml, assets/ exported MuJoCo scene
media/ demo video

## Author

Maksym Skrypka  — Technical Artist, Robotics Simulation

[ArtStation](https://www.artstation.com/olmax3d)


[LinkedIn](https://pl.linkedin.com/in/maksym-skrypka-b5636794) 
