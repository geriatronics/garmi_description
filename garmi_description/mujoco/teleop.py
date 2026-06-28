#!/usr/bin/env python3
# Copyright 2026 Technical University of Munich
# SPDX-License-Identifier: Apache-2.0
"""Twist teleoperation for the Garmi mecanum base in MuJoCo.

Opens the shared Tk joystick widget next to the MuJoCo viewer and drives the
base by body twist instead of poking individual wheels. The twist is mapped to
the four wheel velocity actuators with the mecanum inverse kinematics.

A "closed loop" checkbox (on by default) adds a PI controller on the *measured*
base twist, which tracks the commanded twist and cancels the open-loop mecanum
drift -- the role wheel-odometry/IMU feedback plays on the real robot (here
using MuJoCo's ground-truth base velocity as perfect odometry). Toggle it off
to feel the raw open-loop drift.

The TwistJoystick widget is shared with the ROS 2 Gazebo teleop node.

Run with a display (see `docker compose up teleop`), or locally from this dir:

    python teleop.py
"""
import os
import sys

import mujoco
import mujoco.viewer
import numpy as np

# Base geometry (matches garmi.xml / the URDF).
WHEEL_R = 0.0759          # wheel radius [m]
LXY = 0.319 + 0.2755      # half wheelbase + half track [m]
WHEEL_CLAMP = 10.0        # rad/s, matches the actuator ctrlrange

# Teleop maxima (kept so a single axis stays within the wheel speed limit).
VMAX = 0.7                # m/s
WMAX = 1.2                # rad/s

# Closed-loop twist controller gains (per axis: vx, vy, wz).
#
# FF is a feedforward that compensates the base's open-loop DC gain: forward
# and strafe are ~unity, but in-place rotation is ~4x (the free rollers make
# the base spin faster than no-slip kinematics predicts), so its feedforward is
# scaled down. With an accurate feedforward the controller barely has to
# integrate, which is what avoids the velocity overshoot/reversal when the
# command returns to zero -- so we deliberately use feedforward + proportional
# only (KI = 0). Raise KI slightly for tighter steady-state tracking at the
# cost of a little overshoot.
FF = (1.0, 1.0, 0.25)
KP = (1.6, 1.6, 1.6)
KI = (0.0, 0.0, 0.0)
I_CLAMP = (0.5, 0.5, 0.8)  # anti-windup limit on the integral term


def twist_to_wheels(vx, vy, wz):
    """Mecanum inverse kinematics: body twist -> (fl, fr, rl, rr) wheel speeds."""
    fl = (vx + vy + LXY * wz) / WHEEL_R
    fr = (vx - vy - LXY * wz) / WHEEL_R
    rl = (vx - vy + LXY * wz) / WHEEL_R
    rr = (vx + vy - LXY * wz) / WHEEL_R
    clamp = lambda v: max(-WHEEL_CLAMP, min(WHEEL_CLAMP, v))
    return [clamp(fl), clamp(fr), clamp(rl), clamp(rr)]


def control_step(desired, measured, integ, dt, closed):
    """Return (wheel_speeds, new_integ). Open loop ignores the measurement."""
    if not closed:
        return twist_to_wheels(*desired), integ
    cmd = [0.0, 0.0, 0.0]
    new_integ = list(integ)
    for k in range(3):
        e = desired[k] - measured[k]
        new_integ[k] = max(-I_CLAMP[k], min(I_CLAMP[k], integ[k] + KI[k] * e * dt))
        cmd[k] = FF[k] * desired[k] + KP[k] * e + new_integ[k]
    return twist_to_wheels(*cmd), new_integ


def measure_twist(m, d, base_id, dofadr):
    """Base twist (vx, vy, wz) in the base frame -- 'perfect odometry'.

    For a free joint MuJoCo stores qvel as linear velocity in the global frame
    followed by angular velocity in the local frame, so we rotate the linear
    part into the base frame and take the local yaw rate directly.
    """
    R = d.xmat[base_id].reshape(3, 3)
    v_body = R.T @ d.qvel[dofadr:dofadr + 3]
    w_body = d.qvel[dofadr + 3:dofadr + 6]
    return (float(v_body[0]), float(v_body[1]), float(w_body[2]))


def main():
    import tkinter as tk
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))
    from twist_joystick import TwistJoystick

    m = mujoco.MjModel.from_xml_path("scene.xml")
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    d.ctrl[:] = m.key_ctrl[0]  # hold arms / lift / head at home
    wheel_ids = [m.actuator(f"wheel_{w}").id for w in ("fl", "fr", "rl", "rr")]
    base_id = m.body("base_link").id
    base_dofadr = m.joint("base_free").dofadr[0]
    integ = [0.0, 0.0, 0.0]

    root = tk.Tk()
    root.title("Garmi twist teleop (MuJoCo)")
    root.configure(bg="#222")
    js = TwistJoystick(root, vmax=VMAX, wmax=WMAX)
    js.grid(row=0, column=0)

    closed_var = tk.IntVar(value=1)
    tk.Checkbutton(root, text="closed loop", variable=closed_var,
                   command=lambda: integ.__setitem__(slice(None), [0.0, 0.0, 0.0]),
                   fg="#ccc", bg="#222", selectcolor="#444",
                   activebackground="#222", activeforeground="#fff").grid(row=1, column=0)

    viewer = mujoco.viewer.launch_passive(m, d)
    substeps = max(1, round(1.0 / 60.0 / m.opt.timestep))
    dt = substeps * m.opt.timestep

    def tick():
        if not viewer.is_running():
            root.destroy()
            return
        wheels, integ[:] = control_step(js.twist(),
                                        measure_twist(m, d, base_id, base_dofadr),
                                        integ, dt, bool(closed_var.get()))
        for aid, val in zip(wheel_ids, wheels):
            d.ctrl[aid] = val
        for _ in range(substeps):
            mujoco.mj_step(m, d)
        viewer.sync()
        root.after(16, tick)

    tick()
    root.mainloop()
    viewer.close()


if __name__ == "__main__":
    main()
