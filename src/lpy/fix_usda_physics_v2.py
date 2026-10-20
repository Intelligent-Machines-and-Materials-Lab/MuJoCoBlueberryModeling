"""
Convert USDA physics for direct MuJoCo loading:
- PhysicsCollisionAPI -> PhysicsRigidBodyAPI (define proper bodies)
- PhysicsDistanceJoint -> two PhysicsRevoluteJoint prims (one per axis)

Each junction gets two hinges around perpendicular axes (X and Y).

Usage:
    python fix_usda_physics_v2.py input.usda output.usda
"""
import sys
import re


def fix_usda_physics_v2(input_path, output_path):
    with open(input_path, 'r') as f:
        content = f.read()

    # Step 1: Replace PhysicsCollisionAPI with PhysicsRigidBodyAPI
    content = content.replace('PhysicsCollisionAPI', 'PhysicsRigidBodyAPI')
    print("✓ PhysicsCollisionAPI → PhysicsRigidBodyAPI")

    # Step 2: Replace each PhysicsDistanceJoint with two PhysicsRevoluteJoint
    # Pattern: capture the entire joint definition
    # def PhysicsDistanceJoint "NAME" (...) { ... }

    def replace_joint(match):
        full_match = match.group(0)

        # Extract joint name
        name_match = re.search(r'def PhysicsDistanceJoint "([^"]*)"', full_match)
        if not name_match:
            return full_match

        joint_name = name_match.group(1)

        # Extract indent level
        indent_match = re.search(r'^(\s*)def', full_match, re.MULTILINE)
        if not indent_match:
            return full_match
        indent = indent_match.group(1)

        # Extract body references and local positions
        body0_match = re.search(r'prepend rel physics:body0 = <([^>]+)>', full_match)
        body1_match = re.search(r'prepend rel physics:body1 = <([^>]+)>', full_match)
        pos0_match = re.search(r'point3f physics:localPos0 = \(([^)]+)\)', full_match)
        pos1_match = re.search(r'point3f physics:localPos1 = \(([^)]+)\)', full_match)

        body0 = body0_match.group(1) if body0_match else ""
        body1 = body1_match.group(1) if body1_match else ""
        pos0 = pos0_match.group(1) if pos0_match else ""
        pos1 = pos1_match.group(1) if pos1_match else ""

        # Build the two revolute joint definitions
        result = []

        # First joint (X-axis)
        result.append(f'{indent}def PhysicsRevoluteJoint "{joint_name}_X" (')
        result.append(f'{indent}    prepend apiSchemas = ["PhysicsLimitAPI:rotX"]')
        result.append(f'{indent})')
        result.append(f'{indent}{{')
        result.append(f'{indent}    uniform token physics:axis = "X"')
        if body0:
            result.append(f'{indent}    prepend rel physics:body0 = <{body0}>')
        if body1:
            result.append(f'{indent}    prepend rel physics:body1 = <{body1}>')
        if pos0:
            result.append(f'{indent}    point3f physics:localPos0 = ({pos0})')
        if pos1:
            result.append(f'{indent}    point3f physics:localPos1 = ({pos1})')
        result.append(f'{indent}    float limit:rotX:physics:low = -3.14159')
        result.append(f'{indent}    float limit:rotX:physics:high = 3.14159')
        result.append(f'{indent}}}')
        result.append('')

        # Second joint (Y-axis)
        result.append(f'{indent}def PhysicsRevoluteJoint "{joint_name}_Y" (')
        result.append(f'{indent}    prepend apiSchemas = ["PhysicsLimitAPI:rotY"]')
        result.append(f'{indent})')
        result.append(f'{indent}{{')
        result.append(f'{indent}    uniform token physics:axis = "Y"')
        if body0:
            result.append(f'{indent}    prepend rel physics:body0 = <{body0}>')
        if body1:
            result.append(f'{indent}    prepend rel physics:body1 = <{body1}>')
        if pos0:
            result.append(f'{indent}    point3f physics:localPos0 = ({pos0})')
        if pos1:
            result.append(f'{indent}    point3f physics:localPos1 = ({pos1})')
        result.append(f'{indent}    float limit:rotY:physics:low = -3.14159')
        result.append(f'{indent}    float limit:rotY:physics:high = 3.14159')
        result.append(f'{indent}}}')

        return '\n'.join(result)

    # Match PhysicsDistanceJoint blocks (non-greedy, up to the closing brace)
    joint_pattern = r'def PhysicsDistanceJoint "[^"]*" \([^)]*\)\s*\{[^{}]*(?:\{[^{}]*\}[^{}]*)*\}'
    content = re.sub(joint_pattern, replace_joint, content, flags=re.MULTILINE | re.DOTALL)
    print("✓ PhysicsDistanceJoint → two PhysicsRevoluteJoint (X and Y axes)")

    # Step 3: Replace first hinge joints to attach branches
    # This is done by a separate script after we know attachment points
    print("✓ (First hinge joints will be updated by attachment script)")

    with open(output_path, 'w') as f:
        f.write(content)

    print(f"✓ Saved to {output_path}")
    print("\nChanges made:")
    print("  • PhysicsCollisionAPI → PhysicsRigidBodyAPI")
    print("  • Each PhysicsDistanceJoint → two PhysicsRevoluteJoint")
    print("  • Each revolute joint has explicit physics:axis attribute")
    print("  • Joint limits: ±π radians (±180°)")


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} <input.usda> <output.usda> [--print-xml]")
        sys.exit(1)

    try:
        fix_usda_physics_v2(sys.argv[1], sys.argv[2])

        # Print XML if requested
        if "--print-xml" in sys.argv:
            print("\n" + "="*80)
            print("Generated USDA (first 5000 chars):")
            print("="*80)
            with open(sys.argv[2], 'r') as f:
                content = f.read()
                print(content[:5000])
                if len(content) > 5000:
                    print(f"\n... ({len(content) - 5000} more chars)")
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        sys.exit(1)
