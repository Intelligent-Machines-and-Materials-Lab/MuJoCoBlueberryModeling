"""
Update USDA to connect branches to their parent cylinders.
Replaces the first hinge joint (cyl_0→cyl_1) with attachment joint (cyl_0→parent).
"""
import re
from pathlib import Path


def extract_branch_info(usda_content):
    """Extract all branches with cyl_0 positions and levels.
    
    Only includes branches that have a cyl_0 cylinder (non-empty branches).
    Empty branches without any cylinders are skipped.
    
    Searches only within each Xform block to avoid matching cylinders
    from subsequent branches.
    """
    branches = {}
    empty_branches = []

    branch_pattern = r'def Xform "((cyl_branch)L_(\d+)_Id_(\d+)|(cyl_trunk[^"]*|cyl_rootstock))"'

    for match in re.finditer(branch_pattern, usda_content):
        full_name = match.group(1)
        is_trunk = 'trunk' in full_name.lower() or 'rootstock' in full_name.lower()
        # For branch levels, extract the level number; for trunk/rootstock, use -1
        level = -1 if is_trunk else int(match.group(3))

        start_idx = match.end()
        
        # Find the opening brace of this Xform block
        open_brace = usda_content.find('{', start_idx)
        if open_brace == -1:
            continue
        
        # Find the matching closing brace for this Xform
        brace_count = 1
        search_pos = open_brace + 1
        while brace_count > 0 and search_pos < len(usda_content):
            if usda_content[search_pos] == '{':
                brace_count += 1
            elif usda_content[search_pos] == '}':
                brace_count -= 1
            search_pos += 1
        
        if brace_count != 0:
            continue  # Malformed, skip
        
        # Search only within this Xform block
        search_region = usda_content[open_brace:search_pos]

        cyl0_match = re.search(
            r'def Cylinder "cyl_0"[^}]*?xformOp:translate = \(([^)]+)\)',
            search_region,
            re.DOTALL
        )

        if cyl0_match:
            pos_str = cyl0_match.group(1)
            try:
                pos = tuple(float(x.strip()) for x in pos_str.split(','))
                branches[full_name] = {
                    'level': level,
                    'cyl_0_pos': pos,
                }
            except ValueError:
                pass
        else:
            # Track empty branches (no cyl_0 found within their block)
            if not is_trunk:
                empty_branches.append(full_name)
    
    if empty_branches:
        print(f"\nSkipping {len(empty_branches)} empty branches (no cylinders):")
        for b in sorted(empty_branches)[:5]:
            print(f"  - {b}")
        if len(empty_branches) > 5:
            print(f"  ... and {len(empty_branches) - 5} more")

    return branches


def extract_all_cylinders(usda_content):
    """Extract all cylinder positions with their full paths."""
    cylinders = {}

    pattern = r'def Cylinder "([^"]+)"[^}]*?xformOp:translate = \(([^)]+)\)'

    matches = list(re.finditer(pattern, usda_content, re.DOTALL))

    for i, match in enumerate(matches):
        cyl_name = match.group(1)
        pos_str = match.group(2)

        try:
            pos = tuple(float(x.strip()) for x in pos_str.split(','))

            before = usda_content[:match.start()]
            branch_matches = list(
                re.finditer(r'def Xform "([^"]*cyl_[^"]*)"', before)
            )

            if branch_matches:
                branch_name = branch_matches[-1].group(1)
                full_path = f"{branch_name}/{cyl_name}"
                cylinders[full_path] = pos
            else:
                cylinders[cyl_name] = pos

        except ValueError:
            pass

    return cylinders


def extract_orients(usda_content):
    """Map full cylinder key (branch/cyl_N) to its (w, x, y, z) orient."""
    orients = {}
    pattern = (r'def Cylinder "([^"]+)"[^}]*?xformOp:orient = \(([^)]+)\)'
               r'[^}]*?xformOp:translate')
    for match in re.finditer(pattern, usda_content, re.DOTALL):
        before = usda_content[:match.start()]
        branch_matches = list(re.finditer(r'def Xform "([^"]*cyl_[^"]*)"', before))
        key = (f"{branch_matches[-1].group(1)}/{match.group(1)}"
               if branch_matches else match.group(1))
        orients[key] = tuple(float(x) for x in match.group(2).split(','))
    return orients


def rotate_by_inverse(q, v):
    """Rotate vector v by the inverse of unit quaternion q = (w, x, y, z)."""
    w, x, y, z = q
    n = (w * w + x * x + y * y + z * z) ** 0.5
    w, x, y, z = w / n, -x / n, -y / n, -z / n
    vx, vy, vz = v
    tx = 2 * (y * vz - z * vy)
    ty = 2 * (z * vx - x * vz)
    tz = 2 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def level_folder(branch_name):
    """Return the folder under center_lines holding this branch Xform."""
    if 'rootstock' in branch_name.lower():
        return "Rootstock"
    m = re.match(r'cyl_branchL_(\d+)_', branch_name)
    return f"Level{m.group(1)}" if m else "Trunk"


def distance(p1, p2):
    """Euclidean distance."""
    import math
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(p1, p2)))


def find_attachment_targets(branches, cylinders):
    """Find attachment target for each branch's cyl_0.
    
    For each branch, finds the closest cylinder from either:
    - The same level (for branch-to-branch attachments)
    - The parent level (for branch-to-trunk attachments)
    Excludes cyl_0 cylinders as they are attachment points, not targets.
    
    Detects and reports duplicate branches (branches with identical cyl_0 positions).
    All branches, including duplicates, are attached to their nearest target.
    """
    attachments = {}
    
    # First, detect duplicate branches with identical positions
    positions_to_branches = {}
    duplicates = {}
    
    for branch_name, branch_info in branches.items():
        pos = branch_info['cyl_0_pos']
        pos_key = tuple(round(x, 10) for x in pos)  # Round to avoid float precision issues
        
        if pos_key not in positions_to_branches:
            positions_to_branches[pos_key] = []
        positions_to_branches[pos_key].append(branch_name)
    
    # Find duplicates
    for pos_key, branch_list in positions_to_branches.items():
        if len(branch_list) > 1:
            duplicates[pos_key] = branch_list
    
    if duplicates:
        print(f"\nWARNING: Found {len(duplicates)} duplicate branch positions!")
        for pos_key, branch_list in sorted(duplicates.items()):
            print(f"  Position {pos_key}: {len(branch_list)} branches")
            for branch in sorted(branch_list):
                print(f"    - {branch}")
        print(f"  → All duplicates will be attached to their nearest target (may create constraint stacks).")

    for branch_name, branch_info in sorted(branches.items()):
        level = branch_info['level']
        cyl_0_pos = branch_info['cyl_0_pos']

        closest_cyl = None
        closest_dist = float('inf')

        for cyl_path, cyl_pos in cylinders.items():
            # Check if this cylinder belongs to the same level or parent level
            if level == -1:
                # Trunk/root level - only attach to parent level (if it exists)
                continue
            else:
                # For regular branches, consider same level and parent level
                same_level = f'L_{level}_' in cyl_path
                parent_level_match = f'L_{level - 1}_' in cyl_path
                
                if level == 0:
                    # Level 0 can attach to other L0 branches or to trunk
                    if not (same_level or 'cyl_trunk' in cyl_path):
                        continue
                else:
                    # Higher levels only attach to same level or parent level
                    if not (same_level or parent_level_match):
                        continue

            # Skip cyl_0 cylinders (they are attachment points, not targets)
            if '/cyl_0' in cyl_path:
                continue
            
            # Skip attaching a branch to itself
            if branch_name in cyl_path:
                continue

            dist = distance(cyl_0_pos, cyl_pos)
            if dist < closest_dist:
                closest_dist = dist
                closest_cyl = cyl_path

        if closest_cyl:
            attachments[branch_name] = closest_cyl

    return attachments


def update_first_hinge_joints(usda_content, attachments, branches, cylinders, orients):
    """Insert an X+Y double hinge (parent cylinder -> branch cyl_0) per branch.
    
    This connects each branch's cyl_0 to its attachment target (parent branch or trunk).
    The original HingeJoint_1_2_X/Y joints connecting cyl_0 to cyl_1 are preserved,
    creating a proper kinematic chain: parent → cyl_0 → cyl_1 → cyl_2 → ...
    """
    root = 'lpy_ufo_00000_skeleton/skeleton/center_lines'

    for branch_name, target_path in attachments.items():
        target_branch, target_cyl_name = target_path.rsplit('/', 1)

        target_full_path = f"</{root}/{level_folder(target_branch)}/{target_path}>"
        cyl0_full_path = f"</{root}/{level_folder(branch_name)}/{branch_name}/cyl_0>"

        body0_str = f'prepend rel physics:body0 = {cyl0_full_path}'
        pos = usda_content.find(body0_str)
        if pos != -1:
            joint_start = usda_content.rfind('def PhysicsRevoluteJoint', 0, pos)
            line_start = usda_content.rfind('\n', 0, joint_start) + 1
            ind = ' ' * (joint_start - line_start)
        else:
            # Single-cylinder branch: nest the joints inside the cyl_0 prim.
            xf = usda_content.find(f'def Xform "{branch_name}"')
            c0 = usda_content.find('def Cylinder "cyl_0"', xf) if xf != -1 else -1
            nxt = usda_content.find('def Xform', xf + 1) if xf != -1 else -1
            if c0 == -1 or (nxt != -1 and c0 > nxt):
                print(f"Warning: {branch_name} has no cyl_0; skipped")
                continue
            c0_line = usda_content.rfind('\n', 0, c0) + 1
            c0_ind = ' ' * (c0 - c0_line)
            close = usda_content.find(f'\n{c0_ind}}}\n', c0)
            joint_start = close + 1
            ind = c0_ind + '    '
            usda_content = (usda_content[:joint_start] + '\n'
                            + usda_content[joint_start:])
            joint_start += 1

        # Pivot at cyl_0's center, expressed in the parent cylinder's frame.
        p0 = branches[branch_name]['cyl_0_pos']
        pt = cylinders[target_path]
        delta = tuple(a - b for a, b in zip(p0, pt))
        lp0 = rotate_by_inverse(orients[target_path], delta)
        lp0_str = f"({lp0[0]:.9g}, {lp0[1]:.9g}, {lp0[2]:.9g})"

        def joint(axis):
            return (
                f'{ind}def PhysicsRevoluteJoint "HingeJoint_parent_0_{axis}" (\n'
                f'{ind}    prepend apiSchemas = ["PhysicsLimitAPI:rot{axis}"]\n'
                f'{ind})\n'
                f'{ind}{{\n'
                f'{ind}    uniform token physics:axis = "{axis}"\n'
                f'{ind}    prepend rel physics:body0 = {target_full_path}\n'
                f'{ind}    prepend rel physics:body1 = {cyl0_full_path}\n'
                f'{ind}    point3f physics:localPos0 = {lp0_str}\n'
                f'{ind}    point3f physics:localPos1 = (0, 0, 0)\n'
                f'{ind}    float limit:rot{axis}:physics:low = -3.14159\n'
                f'{ind}    float limit:rot{axis}:physics:high = 3.14159\n'
                f'{ind}}}\n\n'
            )

        usda_content = (usda_content[:joint_start] + joint('X') + joint('Y')
                        + usda_content[joint_start:])

    return usda_content


def main():
    usda_path = "urdf/mod_lpy_ufo_split.usda"

    with open(usda_path, 'r') as f:
        content = f.read()

    print("Analyzing branches...")
    branches = extract_branch_info(content)
    print(f"Found {len(branches)} branches")

    print("Extracting cylinders...")
    cylinders = extract_all_cylinders(content)
    print(f"Found {len(cylinders)} cylinders")

    orients = extract_orients(content)

    print("Finding attachment points...")
    attachments = find_attachment_targets(branches, cylinders)
    print(f"Found {len(attachments)} attachments")

    print("\nSample attachments:")
    for branch in sorted(attachments.keys())[:5]:
        target = attachments[branch]
        print(f"  {branch} → {target}")

    print("\nUpdating first hinge joints...")
    updated_content = update_first_hinge_joints(
        content, attachments, branches, cylinders, orients)

    output_path = "urdf/mod_lpy_ufo_with_attachments.usda"
    with open(output_path, 'w') as f:
        f.write(updated_content)

    print(f"\n✓ Saved to {output_path}")


if __name__ == "__main__":
    main()
