"""
Convert USDA physics for direct MuJoCo loading:
- PhysicsCollisionAPI -> PhysicsRigidBodyAPI (define proper bodies)
- PhysicsDistanceJoint -> two PhysicsRevoluteJoint prims (one per axis)

Each junction gets two hinges around perpendicular axes (X and Y). The pivot is placed at the
shared end of the two cylinders: +height/2 of body0 (parent) and -height/2 of body1 (child).

Usage:
    python fix_usda_physics_v2.py input.usda output.usda
"""
import sys
import re


def find_cylinder_heights(content):
    """Map each Cylinder prim's full path to its height by tracking prim nesting."""
    heights = {}
    stack = []      # prim names for each open brace
    pending = None  # (name, is_cylinder) waiting for its opening brace
    current_cyl = None
    for line in content.splitlines():
        m = re.match(r'\s*def\s+(?:(\w+)\s+)?"([^"]+)"', line)
        if m:
            pending = (m.group(2), m.group(1) == 'Cylinder')
        h = re.match(r'\s*double height = ([^\s]+)', line)
        if h and current_cyl:
            heights[current_cyl] = float(h.group(1))
        for ch in line:
            if ch == '{':
                stack.append(pending if pending else (None, False))
                pending = None
                if stack[-1][1]:
                    current_cyl = '/' + '/'.join(n for n, _ in stack if n)
            elif ch == '}':
                popped = stack.pop() if stack else None
                if popped and popped[1]:
                    current_cyl = None
    return heights


def fix_usda_physics_v2(input_path, output_path):
    with open(input_path, 'r') as f:
        content = f.read()

    cylinder_heights = find_cylinder_heights(content)
    unplaced = []

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

        # Pivot at the shared end of the two cylinders (cylinders are centered, axis along Z)
        h0 = cylinder_heights.get(body0)
        h1 = cylinder_heights.get(body1)
        junction_pos = None
        if h0 is not None and h1 is not None:
            junction_pos = (f'0, 0, {h0 / 2:.9g}', f'0, 0, {-h1 / 2:.9g}')
        else:
            unplaced.append(joint_name)
        if junction_pos:
            pos0, pos1 = junction_pos

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
    print(f"✓ Pivots set at cylinder junctions ({len(cylinder_heights)} cylinders found)")
    if unplaced:
        print(f"! {len(unplaced)} joints kept their original pivots (cylinder not found): {unplaced[:5]}")

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
