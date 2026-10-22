"""Summarize what a USD file defines that MuJoCo's USD decoder could turn into
bodies and joints: UsdPhysics prims, applied API schemas, UsdSkel prims, and
mjc:* attributes.

Run in a venv that has `usd-core` installed (NOT the MuJoCo-USD venv):
    python inspect_usd_physics.py /path/to/file.usda
"""
import sys
from collections import Counter

from pxr import Usd

path = sys.argv[1]
stage = Usd.Stage.Open(path)
if not stage:
    sys.exit(f"Could not open {path}")

type_counts = Counter()
api_counts = Counter()
joint_prims, body_prims, skel_prims, mjc_attrs = [], [], [], []

for prim in stage.Traverse():
    t = prim.GetTypeName() or "(untyped)"
    type_counts[t] += 1
    for api in prim.GetAppliedSchemas():
        api_counts[api] += 1
        if "RigidBodyAPI" in api:
            body_prims.append(prim.GetPath())
    if "Joint" in t:
        joint_prims.append((prim.GetPath(), t))
    if t.startswith("Skel") or t.startswith("Skeleton"):
        skel_prims.append((prim.GetPath(), t))
    for attr in prim.GetAttributes():
        if attr.GetName().startswith("mjc:") and attr.HasAuthoredValue():
            mjc_attrs.append((prim.GetPath(), attr.GetName(), attr.Get()))

print(f"File: {path}")
print(f"Total prims: {sum(type_counts.values())}\n")

print("Prim types:")
for t, n in type_counts.most_common():
    print(f"  {n:6d}  {t}")

print("\nApplied API schemas:")
if api_counts:
    for a, n in api_counts.most_common():
        print(f"  {n:6d}  {a}")
else:
    print("  (none)")

print(f"\nJoint prims: {len(joint_prims)}")
for p, t in joint_prims[:15]:
    print(f"  {p}  [{t}]")
if len(joint_prims) > 15:
    print(f"  ... and {len(joint_prims) - 15} more")

print(f"\nPrims with RigidBodyAPI: {len(body_prims)}")
for p in body_prims[:15]:
    print(f"  {p}")
if len(body_prims) > 15:
    print(f"  ... and {len(body_prims) - 15} more")

print(f"\nSkeleton-related prims (UsdSkel): {len(skel_prims)}")
for p, t in skel_prims[:15]:
    print(f"  {p}  [{t}]")

print(f"\nAuthored mjc:* attributes: {len(mjc_attrs)}")
for p, name, val in mjc_attrs[:20]:
    print(f"  {p}.{name} = {val}")
if len(mjc_attrs) > 20:
    print(f"  ... and {len(mjc_attrs) - 20} more")