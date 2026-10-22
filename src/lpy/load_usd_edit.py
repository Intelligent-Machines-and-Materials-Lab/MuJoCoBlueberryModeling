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
    """Extract radius and height for each cylinder from USDA file.
    Store with full path to avoid name conflicts between trunk and branch cylinders."""
    with open(usda_path, 'r') as f:
        content = f.read()

    cylinders = {}
    # Match: def Cylinder "NAME" followed by height and radius (order may vary)
    # Capture the hierarchy path for disambiguation (e.g., Trunk/cyl_trunkId_0/cyl_1)
    pattern = r'def Cylinder "([^"]+)"[^{]*?\{[^}]*?double height = ([^\n]+)[^}]*?double radius = ([^\n]+)'

    for match in re.finditer(pattern, content, re.DOTALL):
        cyl_name = match.group(1)
        height = float(match.group(2).strip())
        radius = float(match.group(3).strip())
        
        # Also try to find the context (hierarchy) to create a unique key
        # We'll use the local cyl_name for now, but we can improve this if needed
        cylinders[cyl_name] = {'radius': radius, 'height': height}

    return cylinders


def calculate_cylinder_mass(radius, height, density_kg_m3=580):
    """Calculate mass of a cylinder from volume and density.
    
    Args:
        radius: cylinder radius in meters
        height: cylinder height in meters
        density_kg_m3: plant material density in kg/m³ (using 580 from blueberries)
    
    Returns:a
        mass in kg
    """
    volume = pi * (radius ** 2) * height
    mass = volume * density_kg_m3
    return mass


def calculate_stiffness_for_joint(joint, cylinders, usda_path, E_trunk_level0=4e9, E_higher_levels=0.05e9):
    """Calculate K = 2*E*I/L using actual body connections from USDA file.
    Use the child body (body1) of the joint to determine stiffness.
    Use lower E for Level 1+ branches (thinner, more flexible)."""
    # Extract level from joint path: Level0, Level1, etc.
    level = 0  # default to Level 0
    joint_name = joint.name if hasattr(joint, 'name') else str(joint)
    if '/Level1/' in joint_name or '/Level2/' in joint_name:
        level = 1  # Use lower E for Level 1+

    E = E_higher_levels if level >= 1 else E_trunk_level0

    # Parse USDA to find body1 for this joint
    with open(usda_path, 'r') as f:
        content = f.read()
    
    # Extract just the joint name from the full path (last component)
    joint_short_name = joint_name.split('/')[-1] if '/' in joint_name else joint_name
    
    # Find this joint in the USDA and extract body1
    # Pattern: def PhysicsRevoluteJoint "JOINT_NAME" ... prepend rel physics:body1 = <...>
    pattern = rf'def PhysicsRevoluteJoint "{joint_short_name}"[^}}]*?prepend rel physics:body0[^}}]*?prepend rel physics:body1 = <([^>]+)>'
    m = re.search(pattern, content, re.DOTALL)
    
    if not m:
        # Try reverse order
        pattern = rf'def PhysicsRevoluteJoint "{joint_short_name}"[^}}]*?prepend rel physics:body1 = <([^>]+)>'
        m = re.search(pattern, content, re.DOTALL)
    
    if not m:
        return 0.0
    
    body1_path = m.group(1)
    body1_name = body1_path.split('/')[-1]
    
    # Now find THIS cylinder's properties in the USDA
    # We need to search for the cylinder that is at this specific path
    # Pattern: look for the cylinder definition after the parent path
    cyl_pattern = rf'def Xform "[^"]*".*?def Cylinder "{body1_name}"[^}}]*?double height = ([^\n]+)[^}}]*?double radius = ([^\n]+)'
    cyl_m = re.search(cyl_pattern, content, re.DOTALL)
    
    if not cyl_m:
        # Simpler pattern: just find any cylinder with this name and get its properties
        cyl_pattern = rf'def Cylinder "{body1_name}"[^}}]*?double height = ([^\n]+)[^}}]*?double radius = ([^\n]+)'
        cyl_m = re.search(cyl_pattern, content, re.DOTALL)
    
    if not cyl_m:
        return 0.0
    
    L = float(cyl_m.group(1).strip())
    r = float(cyl_m.group(2).strip())
    I = pi * (r ** 4) / 4
    K = 2 * E * I / L
    return K


def load_spec(path, damping=0.15, E_trunk_level0=15e9, E_higher_levels=.1e9, dt=0.00001, plant_density_kg_m3=600):
    """Load USD file directly, remove free joints, add damping, stiffness, and mass.

    Args:
        path: Path to USDA file
        damping: Damping on all hinges
        E_trunk_level0: Elastic modulus for trunk and Level 0 branches (8 GPa default)
        E_higher_levels: Elastic modulus for Level 1+ branches (1 GPa default)
        dt: Timestep for simulation
        plant_density_kg_m3: Density of plant material for mass calculation (600 kg/m³ default)
    """
    spec = mujoco.MjSpec.from_file(path)

    # Remove all free joints - they prevent proper loading and aren't needed
    # The structure is maintained through hinge joints between cylinders
    free_joints = [j for j in spec.joints if j.type.name == "mjJNT_FREE"]
    for joint in free_joints:
        spec.delete(joint)

    if free_joints:
        print(f"✓ Removed {len(free_joints)} free joints from root and branches")

    # Extract cylinder properties for stiffness and mass calculation
    cylinders = extract_cylinder_properties(path)
    print(f"✓ Extracted properties for {len(cylinders)} cylinders")

    # Set mass on all cylinders based on volume and plant density
    total_mass = 0
    for body in spec.bodies:
        # Try to find this body in cylinders dict
        for cyl_name, props in cylinders.items():
            if cyl_name in body.name:
                mass = calculate_cylinder_mass(props['radius'], props['height'], plant_density_kg_m3)
                body.mass = mass
                total_mass += mass
                break
    
    print(f"✓ Set mass for cylinders (density={plant_density_kg_m3} kg/m³, total={total_mass:.3f} kg)")

    # Set damping and stiffness on all hinge joints
    for joint in spec.joints:
        joint.damping = [damping, 0, 0]
        K = calculate_stiffness_for_joint(joint, cylinders, path, E_trunk_level0, E_higher_levels)
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

    model.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICIT
    model.opt.solver = mujoco.mjtSolver.mjSOL_NEWTON 
    model.opt.tolerance = 1e-8

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
        for _ in range(10):
            mujoco.mj_step(model, data)
        print(f"\nstepped 10 times, sim time = {data.time:.6f}s")


if __name__ == "__main__":
    sys.exit(main())