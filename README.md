# 6-DOF Manipulator: Kinematics and Force Control in MuJoCo

Manipulator kinematics and contact-force control implemented from scratch in Python/NumPy,
validated against MuJoCo as ground truth.

**Model:** Universal Robots UR5e, from [`mujoco_menagerie`](https://github.com/google-deepmind/mujoco_menagerie).

**Status:** in progress — environment setup. Nothing here is validated yet, and the results
table below is deliberately unfilled.

---

## Approach

Implement the math by hand, and use MuJoCo's own computation as the ground truth to validate
against. The validation tests are the substance of this repository, not the demo video:

- Hand-derived forward kinematics, checked against `data.xpos` / `mj_kinematics`
- Analytically derived geometric Jacobian, checked against finite-differencing the hand-written
  FK, and cross-checked against `mj_jac`
- Measured contact wrench, checked against a known applied static load

## Scope

| Stage | Content |
|---|---|
| Kinematics | DH-based FK, geometric Jacobian, resolved-rate control, damped least-squares IK, manipulability index along a near-singular path, Monte-Carlo workspace analysis |
| Force sensing | Wrist force/torque sensor, contact detection, gravity compensation so the sensor reports external contact force only |
| Force control | Task-space impedance control, admittance variant, hybrid force/position control, constant normal force while sliding along a surface |
| Stability | Empirical stiffness–damping stability boundary for contact, with the passivity argument |

## Simulation parameters

Held fixed so that reported numbers have a stated basis.

| Parameter | Value |
|---|---|
| Simulation timestep | 2 ms |
| Control rate | 500 Hz |
| Actuation | torque (`motor`) actuators |
| Contact model | MuJoCo `solref` / `solimp`, values recorded per experiment |

## Results

To be filled in as each is measured. No number appears here until it is reproducible.

| Metric | Value |
|---|---|
| Max FK error vs. MuJoCo | — |
| Max Jacobian error vs. finite-difference | — |
| Joint velocity near singularity, DLS vs. raw pseudo-inverse | — |
| Commanded normal force | — |
| RMS normal-force tracking error while sliding | — |
| Stable stiffness–damping region | — |

## What this is not

- **Simulation only.** No hardware was involved. This is not experience running a real UR5e.
- **Classical control, not learned.** No reinforcement learning, no policy learning.
- **MuJoCo contact is a model.** Force control in simulation is meaningfully easier than on
  hardware: there is no sensor noise floor, no drivetrain friction, and no unmodelled
  compliance. Contact stiffness here is a solver parameter (`solref`/`solimp`), not a material
  property, so the measured stability boundary is qualitative with respect to real hardware.

## References

- Siciliano, Sciavicco, Villani & Oriolo, *Robotics: Modelling, Planning and Control*, Springer, 2009
- Lynch & Park, *Modern Robotics: Mechanics, Planning, and Control*, Cambridge, 2017
- Hogan, "Impedance Control: An Approach to Manipulation, Parts I–III", 1985
- Raibert & Craig, "Hybrid Position/Force Control of Manipulators", 1981
- Colgate & Hogan, "Robust Control of Dynamically Interacting Systems", 1988
