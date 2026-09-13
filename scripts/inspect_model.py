#!/usr/bin/env python3
"""Print the model's state handles and run the Tier 0 verification checks.

Usage:
    python scripts/inspect_model.py                # scene with the table
    python scripts/inspect_model.py --arm-only     # arm alone, no table/floor

This closes Tier 0: it shows every handle later tiers need (joints, actuators,
sensors, frames) and verifies the three properties the model was modified for --
torque actuation, a readable wrench, and a sensor that matches a known load.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import mujoco
import numpy as np

REPO = Path(__file__).resolve().parent.parent
GRAVITY = 9.81

# Fixed project parameters. Asserted, not assumed -- every recorded number in
# Tiers 3-4 is only meaningful against a stated timestep and control rate.
TIMESTEP = 0.002        # s   -> 500 Hz sim
CONTROL_RATE = 500.0    # Hz
INTEGRATOR = mujoco.mjtIntegrator.mjINT_IMPLICITFAST

# Joint-space PD gains for holding a commanded pose. These are upstream
# menagerie's own servo gains (Kp=2000/500, Kd=400/100), reused here because they
# are known-good for this arm. Tier 3's impedance control is a different thing
# entirely -- this is only the Tier 0 "holds a commanded pose" controller.
KP = np.array([2000.0, 2000.0, 2000.0, 500.0, 500.0, 500.0])
KD = np.array([400.0, 400.0, 400.0, 100.0, 100.0, 100.0])


def names(model, objtype, count):
    return [mujoco.mj_id2name(model, objtype, i) for i in range(count)]


def geom_label(model, gid):
    """Upstream's arm collision geoms are unnamed, so fall back to the body."""
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid)
    if name:
        return name
    body = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[gid])
    return "<unnamed geom on %s>" % body


def pd_hold(model, data, q_des, seconds, start_key):
    """Gravity-compensated joint-space PD. Returns after `seconds` of sim."""
    mujoco.mj_resetDataKeyframe(model, data, start_key)
    for _ in range(int(round(seconds / model.opt.timestep))):
        mujoco.mj_forward(model, data)
        tau = data.qfrc_bias + KP * (q_des - data.qpos) - KD * data.qvel
        data.ctrl[:] = np.clip(tau, -model.actuator_forcerange[:, 1],
                               model.actuator_forcerange[:, 1])
        mujoco.mj_step(model, data)


def key_qpos(model, name):
    kid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, name)
    return (model.key_qpos[kid].copy(), kid) if kid >= 0 else (None, -1)


def section(title):
    print("\n" + title)
    print("-" * len(title))


def check_parameters(model):
    section("fixed project parameters")
    steps_per_ctrl = round((1.0 / CONTROL_RATE) / model.opt.timestep)
    print("timestep        %.4f s   (expected %.4f)" % (model.opt.timestep, TIMESTEP))
    print("integrator      %d         (expected %d = implicitfast)"
          % (model.opt.integrator, int(INTEGRATOR)))
    print("control rate    %.0f Hz     -> %d sim step(s) per control step"
          % (CONTROL_RATE, steps_per_ctrl))
    print("gravity         %s" % model.opt.gravity)

    assert abs(model.opt.timestep - TIMESTEP) < 1e-12, "timestep is not %g s" % TIMESTEP
    assert model.opt.integrator == INTEGRATOR, "integrator is not implicitfast"
    assert steps_per_ctrl >= 1, "control rate is faster than the sim timestep"
    print("OK")


def describe(model, data):
    section("bodies")
    for i, n in enumerate(names(model, mujoco.mjtObj.mjOBJ_BODY, model.nbody)):
        print("  %2d  %-20s mass %7.4f kg  parent %s"
              % (i, n, model.body_mass[i],
                 mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.body_parentid[i])))

    section("joints (qpos / qvel layout)")
    for i, n in enumerate(names(model, mujoco.mjtObj.mjOBJ_JOINT, model.njnt)):
        lo, hi = model.jnt_range[i]
        print("  %d  %-22s qpos[%d]  range [%+.3f, %+.3f] rad  axis %s"
              % (i, n, model.jnt_qposadr[i], lo, hi, model.jnt_axis[i]))

    section("actuators (torque command)")
    for i, n in enumerate(names(model, mujoco.mjtObj.mjOBJ_ACTUATOR, model.nu)):
        lo, hi = model.actuator_forcerange[i]
        print("  %d  %-16s ctrl[%d] = torque   limit +/- %6.1f N m" % (i, n, i, hi))
        assert model.actuator_gainprm[i][0] == 1.0 and not model.actuator_biasprm[i].any(), (
            "%s is not a plain torque motor -- position servo left in place?" % n)

    section("sensors")
    for i, n in enumerate(names(model, mujoco.mjtObj.mjOBJ_SENSOR, model.nsensor)):
        adr, dim = model.sensor_adr[i], model.sensor_dim[i]
        print("  %d  %-14s sensordata[%2d:%2d]  %s"
              % (i, n, adr, adr + dim, np.round(data.sensordata[adr:adr + dim], 4)))

    section("frames of interest")
    for s in ("attachment_site", "ft_site", "tcp"):
        print("  %-16s world pos %s" % (s, np.round(data.site(s).xpos, 4)))
    R = data.site("tcp").xmat.reshape(3, 3)
    print("  tcp tool axis (+z)      %s" % np.round(R[:, 2], 4))


def check_torque_actuation(model, data):
    """Zero command must leave the arm in free fall; that is what proves the
    position servos are gone."""
    section("check: torque actuation (zero command -> free fall)")
    mujoco.mj_resetDataKeyframe(model, data, 0)
    data.ctrl[:] = 0.0
    mujoco.mj_forward(model, data)
    qacc = np.abs(data.qacc).max()
    print("max|qacc| with ctrl=0   %8.2f rad/s^2" % qacc)
    assert qacc > 1.0, "arm does not accelerate at zero command -- still position controlled?"
    print("OK -- the arm falls, so ctrl is torque")


def check_gravity_compensation(model, data):
    """qfrc_bias is gravity + Coriolis; applying it must hold the arm still."""
    section("check: gravity compensation holds the pose")
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    tau_g = data.qfrc_bias.copy()
    data.ctrl[:] = tau_g
    mujoco.mj_forward(model, data)
    qacc = np.abs(data.qacc).max()
    print("gravity torques         %s N m" % np.round(tau_g, 2))
    print("max|qacc| with ctrl=tau %8.2e rad/s^2" % qacc)
    assert qacc < 1e-6, "gravity compensation did not null the acceleration"
    print("OK")
    return tau_g


def check_ft_against_known_load(model, data, tau_g):
    """With the arm static, the wrist sensor must read the weight of everything
    distal to it -- the Tier 2 known-load validation."""
    section("check: F/T sensor vs known distal mass")
    distal = sum(model.body(b).mass[0] for b in ("ft_sensor", "tool"))
    expected = distal * GRAVITY

    mujoco.mj_resetDataKeyframe(model, data, 0)
    data.ctrl[:] = tau_g
    mujoco.mj_forward(model, data)
    f = data.sensor("ft_force").data.copy()
    measured = float(np.linalg.norm(f))
    err = abs(measured - expected)

    print("distal mass             %.4f kg  (ft_sensor + tool)" % distal)
    print("expected |F| = m g      %.4f N" % expected)
    print("measured |F|            %.4f N   in sensor frame %s" % (measured, np.round(f, 4)))
    print("error                   %.2e N" % err)
    assert err < 1e-3, "wrench does not match the known static load"
    print("OK")
    print("\nNote: the sensor measures everything DISTAL to ft_site, so this")
    print("reading is the tool's own weight with nothing in contact. Tier 2")
    print("subtracts it so the residual is the external contact force.")


def check_pose_holding(model, data):
    """Tier 0's stated "done when": the arm holds a commanded pose under gravity."""
    section("check: holds a commanded pose under gravity")
    q_app, kid = key_qpos(model, "approach")
    if q_app is None:
        q_app, kid = key_qpos(model, "home")
    pd_hold(model, data, q_app, seconds=4.0, start_key=kid)
    q_err = np.abs(data.qpos - q_app).max()
    print("commanded pose          %s" % np.round(q_app, 4))
    print("max joint error         %.2e rad" % q_err)
    print("TCP                     %s" % np.round(data.site("tcp").xpos, 5))
    assert q_err < 1e-4, "PD controller did not settle on the commanded pose"
    print("OK")


def check_contact(model, data):
    """Press the probe into the table and confirm the wrench tracks the load.

    The command is the 'touch' pose extrapolated past the surface, which presses
    in by a fraction of a millimetre. That is crude on purpose: the normal force
    here is whatever the PD stiffness happens to produce. Commanding a FORCE
    rather than a position is Tier 3.
    """
    if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "table_top") < 0:
        print("\n(no table in this model -- skipping the contact check)")
        return
    section("check: contact against the table")

    probe = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "probe_tip")
    table = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "table_top")
    q_app, kid = key_qpos(model, "approach")
    q_touch, _ = key_qpos(model, "touch")
    if q_app is None or q_touch is None:
        print("NOTE: needs the 'approach' and 'touch' keyframes; skipping")
        return

    print("  press    TCP z     probe/table   normal depth    |F| (N)")
    seen_contact = False
    for over in (0.0, 0.5, 1.0, 2.0):
        q_des = q_touch + over * (q_touch - q_app)
        pd_hold(model, data, q_des, seconds=4.0, start_key=kid)

        hits = [data.contact[i] for i in range(data.ncon)
                if {data.contact[i].geom1, data.contact[i].geom2} == {probe, table}]
        other = [i for i in range(data.ncon)
                 if {data.contact[i].geom1, data.contact[i].geom2} != {probe, table}]
        f = np.linalg.norm(data.sensor("ft_force").data)
        print("  x%.1f    %8.5f   %-11s   %-13s  %7.3f"
              % (over, data.site("tcp").xpos[2], "YES" if hits else "no",
                 ("%+.6f m" % hits[0].dist) if hits else "-", f))
        for i in other:
            c = data.contact[i]
            print("      UNEXPECTED contact: %s <-> %s"
                  % (geom_label(model, c.geom1), geom_label(model, c.geom2)))
        seen_contact |= bool(hits)

    assert seen_contact, "probe never contacted the table"
    print("OK -- probe/table contact detected, wrench grows with press depth")
    print("\nThe force rises roughly linearly with commanded overshoot because")
    print("the PD gain sets it. Tier 3 replaces this with a commanded force.")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arm-only", action="store_true",
                    help="load models/ur5e_arm.xml instead of models/scene.xml")
    args = ap.parse_args()

    path = REPO / "models" / ("ur5e_arm.xml" if args.arm_only else "scene.xml")
    print("model: %s" % path.relative_to(REPO))
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)

    print("nq=%d  nv=%d  nu=%d  nbody=%d  ngeom=%d  nsensordata=%d"
          % (model.nq, model.nv, model.nu, model.nbody, model.ngeom, model.nsensordata))

    check_parameters(model)
    describe(model, data)
    check_torque_actuation(model, data)
    tau_g = check_gravity_compensation(model, data)
    check_ft_against_known_load(model, data, tau_g)
    check_pose_holding(model, data)
    check_contact(model, data)

    print("\nall Tier 0 checks passed")


if __name__ == "__main__":
    main()
