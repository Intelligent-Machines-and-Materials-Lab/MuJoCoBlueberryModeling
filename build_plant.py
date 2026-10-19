"""Build a MuJoCo model of the plant directly from the USD file (version 2).

Changes from v1:
  * Branch roots (Level0, Level1, ...) are attached to the nearest segment of a
    LOWER-level chain (Trunk = level -1) instead of being fixed to the world.
    Only the Trunk root is fixed to the world.
  * Hinge axes are perpendicular to the CHILD cylinder (bending of the child).
  * Timestep is chosen automatically from the stiffest joint (override: --dt).

Design:
  * One body + one cylinder geom per USD cylinder; no collisions.
  * Each joint (and each branch attachment) = TWO hinges at the junction.
  * Damping: same on all joints (default 0.15).
  * Stiffness: K = 2*E*I/L from the CHILD cylinder, I = pi*r^4/4, L = length.
    SI units; E default 4 GPa.

Needs, in the same venv:  pip install mujoco usd-core numpy
(Use plain pip `mujoco`, NOT the source-built one.)

Usage:
    python build_plant_v2.py file.usda [--out plant.xml] [--view]
        [--E 4e9] [--damping 0.15] [--density 1000] [--dt SECONDS]
        [--branch-attach joint|weld|fixed] [--sim-seconds 0.1]
"""
import argparse
import re
import statistics
import sys
from collections import defaultdict

import numpy as np
import mujoco
from pxr import Gf, Usd, UsdGeom

AXES = {"X": (1.0, 0.0, 0.0), "Y": (0.0, 1.0, 0.0), "Z": (0.0, 0.0, 1.0)}


def quat_z_to(d):
    z = np.array([0.0, 0.0, 1.0])
    c = float(np.dot(z, d))
    if c > 1 - 1e-9:
        return np.array([1.0, 0.0, 0.0, 0.0])
    if c < -1 + 1e-9:
        return np.array([0.0, 1.0, 0.0, 0.0])
    q = np.array([1.0 + c, *np.cross(z, d)])
    return q / np.linalg.norm(q)


def perpendicular_axes(d):
    helper = np.array([1.0, 0.0, 0.0]) if abs(d[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    a1 = np.cross(d, helper)
    a1 /= np.linalg.norm(a1)
    a2 = np.cross(d, a1)
    a2 /= np.linalg.norm(a2)
    return a1, a2


def rank_of(path):
    """Trunk = -1, LevelN = N, anything else = None."""
    if "/Trunk/" in path:
        return -1
    m = re.search(r"/Level(\d+)/", path)
    return int(m.group(1)) if m else None


def endpoints(info):
    h = info["dir"] * info["length"] / 2
    return info["center"] - h, info["center"] + h


def seg_closest(p, a, b):
    ab = b - a
    t = float(np.clip(np.dot(p - a, ab) / np.dot(ab, ab), 0.0, 1.0))
    return a + t * ab


def cylinder_info(prim, xc, mpu):
    cyl = UsdGeom.Cylinder(prim)
    axis_tok = str(cyl.GetAxisAttr().Get() or "Z")
    height = cyl.GetHeightAttr().Get()
    radius = cyl.GetRadiusAttr().Get()
    m = xc.GetLocalToWorldTransform(prim)
    wa = m.TransformDir(Gf.Vec3d(*AXES[axis_tok]))
    h_scale = wa.GetLength()
    others = [k for k in "XYZ" if k != axis_tok]
    r_scale = float(np.mean([m.TransformDir(Gf.Vec3d(*AXES[k])).GetLength() for k in others]))
    return {
        "center": np.array(m.Transform(Gf.Vec3d(0, 0, 0)), dtype=float) * mpu,
        "dir": np.array(wa, dtype=float) / h_scale,
        "length": height * h_scale * mpu,
        "radius": radius * r_scale * mpu,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("usd_path")
    ap.add_argument("--out", default="plant.xml")
    ap.add_argument("--view", action="store_true")
    ap.add_argument("--E", type=float, default=4e9, help="elastic modulus, Pa")
    ap.add_argument("--damping", type=float, default=0.15)
    ap.add_argument("--density", type=float, default=1000.0, help="kg/m^3 (assumed)")
    ap.add_argument("--dt", type=float, default=None, help="timestep; default: auto from stiffest joint")
    ap.add_argument("--branch-attach", choices=["joint", "weld", "fixed"], default="joint")
    ap.add_argument("--sim-seconds", type=float, default=0.1, help="length of the test run")
    args = ap.parse_args()

    stage = Usd.Stage.Open(args.usd_path)
    if not stage:
        sys.exit("Could not open USD file")
    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    up = UsdGeom.GetStageUpAxis(stage)
    print(f"Stage up axis: {up}, meters per unit: {mpu}")

    xc = UsdGeom.XformCache()
    cyls = {}
    for prim in stage.Traverse():
        if prim.GetTypeName() == "Cylinder":
            info = cylinder_info(prim, xc, mpu)
            if info["radius"] > 0 and info["length"] > 0:
                cyls[str(prim.GetPath())] = info

    children = defaultdict(list)
    parent_of = {}
    for prim in stage.Traverse():
        if "Joint" not in prim.GetTypeName():
            continue
        r0 = prim.GetRelationship("physics:body0").GetTargets()
        r1 = prim.GetRelationship("physics:body1").GetTargets()
        if not r0 or not r1 or str(r0[0]) not in cyls or str(r1[0]) not in cyls:
            print(f"  joint {prim.GetPath()} not linked to two cylinders; skipping")
            continue
        p, c = str(r0[0]), str(r1[0])
        if c not in parent_of:
            parent_of[c] = p
            children[p].append(c)

    roots = [p for p in cyls if p not in parent_of]
    fixed_roots = [r for r in roots if rank_of(r) in (-1, None)]
    branch_roots = [r for r in roots if r not in fixed_roots]
    print(f"Cylinders: {len(cyls)} | joints linked: {len(parent_of)} | roots: {len(roots)} "
          f"(trunk/unclassified: {len(fixed_roots)}, branch bases: {len(branch_roots)})")
    for r in fixed_roots:
        print(f"  fixed to world: {r}" + ("  [unclassified level!]" if rank_of(r) is None else ""))

    pivot_override, attach_dists = {}, []
    if args.branch_attach != "fixed":
        for r in branch_roots:
            rk = rank_of(r)
            best = None
            for q in cyls:
                qr = rank_of(q)
                if qr is None or qr >= rk:
                    continue
                qa, qb = endpoints(cyls[q])
                for e in endpoints(cyls[r]):
                    pt = seg_closest(e, qa, qb)
                    d = float(np.linalg.norm(pt - e))
                    if best is None or d < best[0]:
                        best = (d, q, pt)
            if best is None:
                print(f"  no lower-level parent for {r}; fixing to world")
                fixed_roots.append(r)
                continue
            d, q, pt = best
            parent_of[r] = q
            children[q].append(r)
            pivot_override[r] = pt
            attach_dists.append(d)
        if attach_dists:
            print(f"Branch attachment distance (branch base to parent segment): "
                  f"max={max(attach_dists):.3g} m, median={statistics.median(attach_dists):.3g} m, "
                  f"min={min(attach_dists):.3g} m   <- should be ~0 if branches start on their parent")
    else:
        fixed_roots = roots

    spec = mujoco.MjSpec()
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    spec.option.gravity = [0, -9.81, 0] if up == "Y" else [0, 0, -9.81]

    bodies, origin, stack = {}, {}, []
    gaps, stiffs, omegas = [], [], []
    for r in fixed_roots:
        info = cyls[r]
        b = spec.worldbody.add_body(name=f"b{len(bodies):04d}", pos=list(info["center"]))
        b.add_geom(name=f"g{len(bodies):04d}", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                   size=[info["radius"], info["length"] / 2, 0], pos=[0, 0, 0],
                   quat=list(quat_z_to(info["dir"])), density=args.density,
                   contype=0, conaffinity=0)
        bodies[r], origin[r] = b, info["center"]
        stack.append(r)

    while stack:
        p = stack.pop()
        pi = cyls[p]
        for c in children[p]:
            ci = cyls[c]
            if c in pivot_override:
                pivot, is_branch = pivot_override[c], True
            else:
                ends = endpoints(pi)
                pivot = min(ends, key=lambda e: np.linalg.norm(e - ci["center"]))
                is_branch = False
                gaps.append(min(np.linalg.norm(pivot - e) for e in endpoints(ci)))

            idx = len(bodies)
            b = bodies[p].add_body(name=f"b{idx:04d}", pos=list(pivot - origin[p]))
            if not (is_branch and args.branch_attach == "weld"):
                r_c, L_c = ci["radius"], ci["length"]
                I = np.pi * r_c ** 4 / 4.0
                K = 2.0 * args.E * I / L_c
                m = args.density * np.pi * r_c ** 2 * L_c
                I_h = m * (L_c ** 2 / 3.0 + r_c ** 2 / 4.0)  # child's own inertia about its base
                stiffs.append(K)
                omegas.append(float(np.sqrt(K / I_h)))
                a1, a2 = perpendicular_axes(ci["dir"])
                for k, ax in enumerate((a1, a2)):
                    b.add_joint(name=f"j{idx:04d}_{k}", type=mujoco.mjtJoint.mjJNT_HINGE,
                                axis=list(ax), stiffness=K, damping=args.damping)
            b.add_geom(name=f"g{idx:04d}", type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                       size=[ci["radius"], ci["length"] / 2, 0],
                       pos=list(ci["center"] - pivot),
                       quat=list(quat_z_to(ci["dir"])), density=args.density,
                       contype=0, conaffinity=0)
            bodies[c], origin[c] = b, pivot
            stack.append(c)

    unreached = len(cyls) - len(bodies)
    if unreached:
        print(f"WARNING: {unreached} cylinders were not reached from a fixed root (cycle or missing link)")

    w_max = max(omegas) if omegas else 1.0
    dt_auto = min(1e-4, 0.5 / w_max)
    dt = args.dt if args.dt else dt_auto
    spec.option.timestep = dt

    model = spec.compile()
    data = mujoco.MjData(model)
    with open(args.out, "w") as f:
        f.write(spec.to_xml())

    print(f"\nModel: nbody={model.nbody} njnt={model.njnt} nv={model.nv} ngeom={model.ngeom}")
    print(f"Total mass: {model.body_mass.sum():.4g} kg (density assumed {args.density} kg/m^3)")
    if stiffs:
        print(f"Stiffness K (N*m/rad): min={min(stiffs):.3g} median={statistics.median(stiffs):.3g} max={max(stiffs):.3g}")
        print(f"Fastest spring mode (estimate): {w_max:.3g} rad/s -> suggested dt <= {dt_auto:.3g} s; using dt = {dt:.3g} s")
    if gaps:
        print(f"Chain junction gap: max={max(gaps):.3g} m, median={statistics.median(gaps):.3g} m")
    print(f"Saved MJCF to {args.out}")

    if args.view:
        from mujoco import viewer
        viewer.launch(model, data)
    else:
        nsteps = int(args.sim_seconds / dt)
        print(f"Running test: {nsteps} steps ({args.sim_seconds} s of sim time)...")
        for _ in range(nsteps):
            mujoco.mj_step(model, data)
        ok = bool(np.isfinite(data.qpos).all()) and data.time > 0.9 * args.sim_seconds
        print(f"sim time = {data.time:.4f} s | max |qpos| = {np.abs(data.qpos).max():.3g} rad | "
              f"{'looks stable' if ok else 'UNSTABLE (data was reset or has NaNs)'}")


if __name__ == "__main__":
    main()