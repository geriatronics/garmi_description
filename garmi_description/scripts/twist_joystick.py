# Copyright 2026 Technical University of Munich
# SPDX-License-Identifier: Apache-2.0
"""A small, dependency-free Tk joystick widget for commanding a body twist.

Pure Tkinter (no ROS, no MuJoCo) so it can be reused by both the MuJoCo teleop
([garmi_description/mujoco/teleop.py]) and the ROS 2 Gazebo teleop node
([garmi_description/scripts/twist_teleop.py]).

  * a draggable joystick sets linear velocity (up = +x forward, left = +y),
  * a vertical slider sets angular velocity (turn rate).

Both spring back to zero on release. Read the current command from ``.vx``,
``.vy``, ``.wz`` (or ``.twist()``).
"""
import math
import tkinter as tk


class TwistJoystick(tk.Frame):
    def __init__(self, master, vmax=0.7, wmax=1.2, radius=95):
        super().__init__(master, bg="#222")
        self.vmax, self.wmax = vmax, wmax
        self.vx = self.vy = self.wz = 0.0

        R = radius
        cx = cy = R + 12
        cv = tk.Canvas(self, width=2 * (R + 12), height=2 * (R + 12),
                       bg="#222", highlightthickness=0)
        cv.grid(row=0, column=0, padx=10, pady=10)
        cv.create_oval(cx - R, cy - R, cx + R, cy + R, outline="#555", width=2)
        cv.create_line(cx, cy - R, cx, cy + R, fill="#333")
        cv.create_line(cx - R, cy, cx + R, cy, fill="#333")
        knob = cv.create_oval(cx - 14, cy - 14, cx + 14, cy + 14, fill="#44aaff", outline="")

        def on_drag(e):
            dx, dy = e.x - cx, e.y - cy
            dist = math.hypot(dx, dy)
            if dist > R:
                dx, dy = dx * R / dist, dy * R / dist
            cv.coords(knob, cx + dx - 14, cy + dy - 14, cx + dx + 14, cy + dy + 14)
            self.vx = -dy / R * self.vmax   # up = forward
            self.vy = -dx / R * self.vmax   # left = +y

        def on_release(_):
            cv.coords(knob, cx - 14, cy - 14, cx + 14, cy + 14)
            self.vx = self.vy = 0.0

        cv.bind("<B1-Motion>", on_drag)
        cv.bind("<ButtonRelease-1>", on_release)
        tk.Label(self, text="drag: forward / strafe", fg="#ccc", bg="#222").grid(row=1, column=0)

        ang = tk.Scale(self, from_=wmax, to=-wmax, resolution=0.05, length=2 * R,
                       orient="vertical", label="turn", fg="#ccc", bg="#222",
                       troughcolor="#444", highlightthickness=0,
                       command=lambda v: setattr(self, "wz", float(v)))
        ang.grid(row=0, column=1, padx=10)
        ang.bind("<ButtonRelease-1>", lambda _: (ang.set(0), setattr(self, "wz", 0.0)))

    def twist(self):
        return (self.vx, self.vy, self.wz)
