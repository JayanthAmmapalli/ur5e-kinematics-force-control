# Model provenance and licensing

## `ur5e_arm.xml`

A **modified copy** of `universal_robots_ur5e/ur5e.xml` from
[mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie), which is
itself derived from the publicly available
[UR5e URDF description](https://github.com/ros-industrial/universal_robot/tree/kinetic-devel/ur_e_description)
by the ROS Industrial Consortium.

**Licence:** BSD-3-Clause, Copyright 2018 ROS Industrial Consortium.
The full upstream licence text is kept verbatim in [`LICENSE.ur5e`](LICENSE.ur5e),
as clause 1 requires.

### Modifications made in this repository

| # | Change | Why |
|---|---|---|
| 1 | Six `<general>` position servos replaced by six `<motor>` torque actuators | Task-space impedance control needs torque-level command. Upstream's per-joint force limits (±150 N·m large joints, ±28 N·m wrist) are preserved. |
| 2 | Added `ft_sensor` body welded at the tool flange, carrying site `ft_site` | Wrist force/torque sensing. MuJoCo reports the wrench transmitted between a site's body and its parent, so the sensor body must exist explicitly. |
| 3 | Added `tool` body with a spherical `probe_tip` and the `tcp` site | A sphere gives one well-defined contact point, keeping normal-force control free of edge and face effects. `tcp` is the FK/IK target frame. |
| 4 | `keyframe` `home`: `ctrl` changed from position targets to zeros | Under torque actuators those numbers would be read as torques. Zeroed, so loading `home` and stepping demonstrates free fall. |
| 5 | Added `<force>`, `<torque>`, and four `frame*` sensors | Wrench plus end-effector pose and twist, for validation and control. |
| 6 | `<option timestep="0.002">` stated explicitly | It matches MuJoCo's default, but every number this project reports depends on it, so it is declared rather than inherited. |

Upstream's `<default>` block, `<asset>` list, inertial parameters, and body tree
are **unchanged**. Keeping them untouched is deliberate: the inertias are what
gravity compensation and the dynamics validation are checked against, so they
must remain the upstream values rather than anything hand-tuned here.

## `scene.xml`

Written for this project. The visual and ground-plane setup follows the pattern
of menagerie's own `scene.xml` (same Apache-2.0 / BSD-3-Clause upstream);
the table, its contact parameters, and the `approach` / `touch` keyframes are new.

## Meshes

**Not vendored.** The 30 MB of `.obj` meshes stay in your `mujoco_menagerie`
clone; `models/assets` is a gitignored symlink created by
`scripts/setup_assets.py`. This keeps the repository small and avoids
redistributing upstream binary assets.

## `mujoco_menagerie` itself

Apache License 2.0, Copyright 2022 DeepMind Technologies Limited.
