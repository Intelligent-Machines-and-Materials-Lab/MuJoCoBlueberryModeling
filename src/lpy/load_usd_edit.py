"""Load a USD(.usda) file into MuJoCo from Python and edit joint stiffness/damping.

Usage:
    python load_usd_edit.py /path/to/file.usda [--view]

Loading USD needs the USD decoder plugin (libusd_decoder_plugin.so) to be
registered in this Python process. See the setup notes for how to do that.
"""
import argparse
import os
import re
import sys
from math import pi

import mujoco


def extract_cylinder_properties(usda_path):
    """Extract radius and height for each cylinder from USDA file."""
    with open(usda_path, 'r') as f:
        content = f.read()

    cylinders = {}
    # Match: def Cylinder "NAME" followed by height and radius (order may vary)
    pattern = r'def Cylinder "([^"]+)"[^{]*?\{[^}]*?double height = ([^\n]+)[^}]*?double radius = ([^\n]+)'

    for match in re.finditer(pattern, content, re.DOTALL):
        cyl_name = match.group(1)
        height = float(match.group(2).strip())
        radius = float(match.group(3).strip())
        cylinders[cyl_name] = {'radius': radius, 'height': height}

    return cylinders


def calculate_stiffness_for_joint(joint_name, cylinders, E_trunk_level0=4e9, E_higher_levels=0.05e9):
    """Parse joint name to extract body1 (next) cylinder, calculate K = 2*E*I/L.
    For attachment joints (HingeJoint_parent), use cyl_1 of the attached branch.
    Use lower E for Level 1+ branches (thinner, more flexible)."""
    # Extract level from joint name path: Level0, Level1, etc., or Trunk
    level = 0  # default to Level 0
    if '/Level1/' in joint_name or '/Level2/' in joint_name or '/Level3/' in joint_name:
        level = 1  # Use lower E for Level 1+

    E = E_higher_levels if level >= 1 else E_trunk_level0

    # First try: regular hinge joint like HingeJoint_N_M_X
    m = re.search(r'HingeJoint_(\d+)_(\d+)_[XY]', joint_name)
    if m:
        body1_idx = int(m.group(2))
        body1_name = f"cyl_{body1_idx}"
    else:
        # Second try: attachment joint like HingeJoint_parent_0_X (uses cyl_1 of the branch)
        m = re.search(r'HingeJoint_parent_0_[XY]', joint_name)
        if m:
            body1_name = "cyl_1"  # Branch's cyl_1 is body1
        else:
            return 0.0

    if body1_name not in cylinders:
        return 0.0

    r = cylinders[body1_name]['radius']
    L = cylinders[body1_name]['height']
    I = pi * (r ** 4) / 4
    K = 2 * E * I / L
    return K


def load_spec(path, damping=0.2, E_trunk_level0=0, E_higher_levels=0, dt=0.0001):
    """Load USD file directly, remove free joints, add damping and stiffness.

    Args:
        path: Path to USDA file
        damping: Damping on all hinges
        E_trunk_level0: Elastic modulus for trunk and Level 0 branches (4 GPa default)
        E_higher_levels: Elastic modulus for Level 1+ branches (0.05 GPa default, much softer)
        dt: Timestep for simulation
    """
    spec = mujoco.MjSpec.from_file(path)

    # Remove all free joints - they prevent proper loading and aren't needed
    # The structure is maintained through hinge joints between cylinders
    free_joints = [j for j in spec.joints if j.type.name == "mjJNT_FREE"]
    for joint in free_joints:
        spec.delete(joint)

    if free_joints:
        print(f"✓ Removed {len(free_joints)} free joints from root and branches")

    # Extract cylinder properties for stiffness calculation
    cylinders = extract_cylinder_properties(path)
    print(f"✓ Extracted properties for {len(cylinders)} cylinders")

    # Set damping and stiffness on all hinge joints
    for joint in spec.joints:
        joint.damping = [damping, 0, 0]
        K = calculate_stiffness_for_joint(joint.name, cylinders, E_trunk_level0, E_higher_levels)
        joint.stiffness = [K, 0, 0]

    spec.option.timestep = dt
    print(f"✓ Set damping={damping}, stiffness (E_L0={E_trunk_level0/1e9:.2f}GPa, E_L1+={E_higher_levels/1e9:.3f}GPa), dt={dt}")
    return spec


def describe(model):
    print(f"\nnbody={model.nbody} njnt={model.njnt} ngeom={model.ngeom} nq={model.nq} nv={model.nv}")
    print(f"{'joint':32s} {'type':8s} {'stiffness':>10s} {'damping':>10s}")
    for j in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j) or f"joint_{j}"
        jtype = mujoco.mjtJoint(model.jnt_type[j]).name.replace("mjJNT_", "")
        dof = model.jnt_dofadr[j]
        if mujoco.mjtJoint(model.jnt_type[j]).name != "mjJNT_FREE":  # Skip free joints
            print(f"{name:32s} {jtype:8s} {model.jnt_stiffness[j]:10.4g} {model.dof_damping[dof]:10.4g}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("usd_path")
    ap.add_argument("--view", action="store_true", help="open the interactive viewer")
    args = ap.parse_args()

    spec = load_spec(args.usd_path)
    model = spec.compile()
    data = mujoco.MjData(model)
    describe(model)

    # --- To edit joint properties on the compiled model: -----------
    # Example: set damping on a specific joint's DOF
    # dof_idx = model.jnt_dofadr[0]  # first joint
    # model.dof_damping[dof_idx] = 0.5
    #
    # Example: set stiffness on a joint
    # model.jnt_stiffness[0] = 10.0

    if args.view:
        from mujoco import viewer
        viewer.launch(model, data)
    else:
        for _ in range(1000):
            mujoco.mj_step(model, data)
        print(f"\nstepped 1000 times, sim time = {data.time:.3f}s")


if __name__ == "__main__":
    sys.exit(main())