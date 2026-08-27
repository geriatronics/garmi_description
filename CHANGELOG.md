# Changelog

All notable changes to this project are documented here. The format is based on
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project
adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- **Code quality gate matching garmi-core's** — `ruff.toml` carries garmi-core's ruff rules
  verbatim, a `.pre-commit-config.yaml` runs them (autofixing staged files on commit, judging
  the whole tree on push), and CI reaches the same verdict on every pull request with the same
  pinned ruff. garmi-core vendors this repository as a submodule and lints the whole checked-out
  tree on push, so a finding here used to block a push there that nobody could fix from that
  side; now it is caught where it can be fixed.

### Changed
- **Python sources brought up to that gate** — quotes, import order and formatting normalised,
  docstrings added, and a handful of real cleanups: an unused `numpy` import dropped, an unused
  model argument removed from `measure_twist`, the wheel clamp turned from an assigned lambda
  into a named function, `zip()` in the teleop loop made `strict=True`, unclosed file handles in
  the asset builder given context managers, and its output tidying extracted into its own
  function. Behaviour is unchanged; Qt's `mousePressEvent`-style overrides keep their names,
  which the gate is told about rather than renaming what Qt looks up.

## [0.1.3] - 2026-08-25

### Fixed
- **2D lidar angular resolution** — both `lidar2d_0` and `lidar2d_1` scanned at
  0.5 deg (540 samples over their 270 deg span), which is Clearpath's *simulation*
  default rather than the sensor's own step. The Hokuyo UST on the robot reports at
  its native 0.25 deg, so the model now uses 1080 samples and matches what the
  hardware publishes. Coarse beams straddle thin obstacles: at 0.5 deg the example
  navigation misses table legs at range that the real robot sees.

## [0.1.2] - 2026-07-14

### Added
- `docs/real_robot.md` documenting the real robot's ROS 2 interface — the two
  namespaced controller managers (`/garmi/arms`, `/r100_0603`), the `olive` head
  servo, how the sim controllers map to the hardware, and how to aggregate the
  per-manager `/joint_states` — with a pointer from the README.

### Changed
- Renamed joints, links (TF frames) and controllers to match the **real Garmi
  robot**, so the model lines up with its `/joint_states`, TF, and controller
  topics: arms → `left_fr3_*` / `right_fr3_*` (joints, links, hand & finger
  frames), head → `o1_motor_1` / `o1_motor_2`, controllers →
  `left_arm_joint_trajectory_controller`, `platform_velocity_controller`, etc.
  (wheels and lift already matched). The URDF and MJCF now use identical names.

## [0.1.1] - 2026-06-28

### Fixed
- MuJoCo base: added the missing **left side panel** and **rear panel** of the
  mobile base (only one side/end was drawn before, exposing the rear wheels and
  the inside of the base), and corrected the **light colours** — front lights
  white, rear lights red — to match the URDF.
- MuJoCo teleop: cancel the small base rotation that appeared when **strafing**
  (a tightly-clamped yaw integral in the closed-loop base controller, effective
  for combined strafe+turn commands too), and cap strafing slower than forward
  driving to match the real robot.

## [0.1.0] - 2026-06-28

First public release of the portable Garmi robot description.

### Added
- Self-contained, single-file **URDF** of the Garmi robot (dual Franka FR3
  arms + Franka Hands, mecanum mobile base, telescoping lift, pan/tilt head),
  with all meshes, materials and textures bundled.
- **RViz** example (`docker compose up rviz`) and a **Gazebo** example
  (`docker compose up gazebo`) with a single rqt window combining the
  joint-trajectory GUI and a holonomic twist **joystick** plugin for the base.
- Gazebo motion demo (`docker compose up gazebo-motion`) driven by an example
  node, plus configurable arm controllers (trajectory or forward velocity).
- A curated **MuJoCo (MJCF)** model under `garmi_description/mujoco/`
  (`garmi.xml` + `scene.xml`), reusing the `mujoco_menagerie` FR3 arms and
  Franka Hand, with physically-modelled mecanum rollers, self-collision
  volumes, a telescoping lift, and joint/velocity limits mirroring the URDF.
- MuJoCo interactive viewer (`docker compose up mujoco`) and a twist teleop
  (`docker compose up mujoco-teleop`) with a feedforward + proportional
  closed-loop base controller.
- Containerised quick-start via `Dockerfile`, `Dockerfile.mujoco` and
  `docker-compose.yml`; reproducible MuJoCo asset build under
  `garmi_description/mujoco/build/`.
- Licensing for open-source release under **Apache-2.0**, with `NOTICE` and
  `THIRD_PARTY_LICENSES` covering the bundled Franka (Apache-2.0) and
  Clearpath Ridgeback-derived (BSD-3-Clause) meshes.
- Continuous integration validating the MuJoCo model, building the container
  images, and `colcon`-building the ROS 2 package.

[Unreleased]: https://github.com/geriatronics/garmi_description/compare/v0.1.3...HEAD
[0.1.3]: https://github.com/geriatronics/garmi_description/compare/v0.1.2...v0.1.3
[0.1.2]: https://github.com/geriatronics/garmi_description/compare/v0.1.1...v0.1.2
[0.1.1]: https://github.com/geriatronics/garmi_description/compare/v0.1.0...v0.1.1
[0.1.0]: https://github.com/geriatronics/garmi_description/releases/tag/v0.1.0
