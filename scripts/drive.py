"""
drive.py — keyboard teleop for the test agent in a MuJoCo scene

Arrow keys move the agent relative to the camera view:
  UP = away from camera, DOWN = towards camera, LEFT / RIGHT = sideways.
Press the same arrow again to stop.

Usage:
  python drive.py scene/room.xml
"""

import math
import sys
import time

import numpy as np
import mujoco
import mujoco.viewer

XML = sys.argv[1] if len(sys.argv) > 1 else "scene/room.xml"
SPEED = 1.0          # target speed, m/s
GAIN = 300.0         # N per (m/s) of speed error
MAX_FORCE = 400.0    # N, limits how hard the agent can push furniture
FOLLOW_AGENT = False # True: camera keeps the agent in the center

KEY_RIGHT, KEY_LEFT, KEY_DOWN, KEY_UP = 262, 263, 264, 265

model = mujoco.MjModel.from_xml_path(XML)
data = mujoco.MjData(model)
dof_x = model.joint("agent_x").dofadr[0]
dof_y = model.joint("agent_y").dofadr[0]
agent_id = model.body("agent").id

cmd = [0.0, 0.0]  # [forward, right]


def key_callback(key):
    moves = {KEY_UP: (1, 0), KEY_DOWN: (-1, 0), KEY_LEFT: (0, -1), KEY_RIGHT: (0, 1)}
    if key not in moves:
        return
    new = list(moves[key])
    cmd[:] = [0.0, 0.0] if cmd == new else new  # same key again = stop


def scene_bounds():
    """Center and horizontal size of the scene, ignoring the infinite floor plane."""
    mujoco.mj_forward(model, data)
    mask = model.geom_type != mujoco.mjtGeom.mjGEOM_PLANE
    pts = data.geom_xpos[mask]
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    return (lo + hi) / 2, float((hi - lo)[:2].max())


prev_time = 0.0

with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
    # Start with a top-down view of the whole scene
    center, size = scene_bounds()
    with viewer.lock():
        viewer.cam.lookat[:] = center
        viewer.cam.distance = size * 1.3
        viewer.cam.elevation = -89.9  # straight down (exactly -90 is unstable)
        viewer.cam.azimuth = 90       # screen "up" = world +Y

    while viewer.is_running():
        start = time.time()

        # Camera-relative directions on the floor plane
        az = math.radians(viewer.cam.azimuth)
        forward = np.array([math.cos(az), math.sin(az)])
        right = np.array([math.sin(az), -math.cos(az)])
        v_target = SPEED * (cmd[0] * forward + cmd[1] * right)

        # Simple speed controller: push towards target speed, with a force limit
        v = np.array([data.qvel[dof_x], data.qvel[dof_y]])
        force = np.clip(GAIN * (v_target - v), -MAX_FORCE, MAX_FORCE)
        data.qfrc_applied[dof_x] = force[0]
        data.qfrc_applied[dof_y] = force[1]

        mujoco.mj_step(model, data)

        # Detect automatic reset caused by instability and report the culprit
        if data.time < prev_time:
            dof = data.warning[mujoco.mjtWarning.mjWARN_BADQACC].lastinfo
            body = model.body(model.dof_bodyid[dof]).name
            print(f"[{prev_time:.3f}s] UNSTABLE: simulation reset, culprit body: '{body}' (DOF {dof})")
        prev_time = data.time

        if FOLLOW_AGENT:
            viewer.cam.lookat[:2] = data.xpos[agent_id][:2]

        viewer.sync()

        # Keep real-time speed
        wait = model.opt.timestep - (time.time() - start)
        if wait > 0:
            time.sleep(wait)
