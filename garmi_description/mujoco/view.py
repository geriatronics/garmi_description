#!/usr/bin/env python3
# Copyright 2026 Technical University of Munich
# SPDX-License-Identifier: Apache-2.0
"""Open the Garmi scene in the interactive MuJoCo viewer, in its home pose.

`python -m mujoco.viewer --mjcf=scene.xml` starts at the model's zero
configuration (arms flat). This launcher loads the `home` keyframe first, so
the robot opens in the same ready pose the Gazebo model starts in.
"""
import mujoco
import mujoco.viewer

m = mujoco.MjModel.from_xml_path("scene.xml")
d = mujoco.MjData(m)
mujoco.mj_resetDataKeyframe(m, d, 0)  # home keyframe: ready pose + holding ctrl
mujoco.mj_forward(m, d)
mujoco.viewer.launch(m, d)
