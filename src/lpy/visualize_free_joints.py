"""Visualize which bodies have/had free joints by coloring them differently."""
import mujoco
import sys

def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <usda_file>")
        sys.exit(1)

    # Load the spec and compile to check for free joints
    spec = mujoco.MjSpec.from_file(sys.argv[1])
    temp_model = spec.compile()

    # Track which bodies have free joints in the compiled model
    free_joint_body_names = set()
    for i in range(temp_model.njnt):
        if mujoco.mjtJoint(temp_model.jnt_type[i]).name == "mjJNT_FREE":
            body_id = temp_model.jnt_bodyid[i]
            body_name = mujoco.mj_id2name(temp_model, mujoco.mjtObj.mjOBJ_BODY, body_id)
            if body_name:
                free_joint_body_names.add(body_name)

    print(f"Found {len(free_joint_body_names)} bodies with free joints:")
    for name in sorted(list(free_joint_body_names))[:5]:
        print(f"  {name}")
    if len(free_joint_body_names) > 5:
        print(f"  ... and {len(free_joint_body_names) - 5} more")

    # Reload spec and color those bodies red
    spec = mujoco.MjSpec.from_file(sys.argv[1])
    for body in spec.bodies:
        if body.name in free_joint_body_names:
            for geom in body.geoms:
                geom.rgba = [1.0, 0.2, 0.2, 0.8]  # Red

    # Convert free joints to fixed by changing type to HINGE with no axis constraint
    # Actually, just remove them - they'll be implicit through parent structure
    joints_to_remove = [j for j in spec.joints if j.type.name == "mjJNT_FREE"]

    for joint in joints_to_remove:
        spec.delete(joint)

    print(f"Removed {len(joints_to_remove)} free joints")

    # Compile and view
    model = spec.compile()
    data = mujoco.MjData(model)

    print(f"Compiled: {model.nbody} bodies, {model.njnt} joints")
    print("Launching viewer (red cylinders had free joints)...")

    from mujoco import viewer
    viewer.launch(model, data)

if __name__ == "__main__":
    main()

