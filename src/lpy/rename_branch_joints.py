"""
Rename internal branch joints (HingeJoint_N_M_X/Y) to include branch ID for uniqueness.
This prevents MuJoCo constraint conflicts from duplicate joint names.

Pattern: HingeJoint_1_2_X → HingeJoint_cyl_branchL_0_Id_000_1_2_X

Usage: python rename_branch_joints.py [input.usda] [output.usda]
"""
import re
import sys
from pathlib import Path


def rename_branch_joints(usda_content):
    """Rename internal branch joints to include the branch ID, keeping the _X/_Y axis suffix.

    HingeJoint_1_2_X -> HingeJoint_cyl_branchL_0_Id_000_1_2_X. Each rename is applied only
    inside its own branch block; attachment joints (HingeJoint_parent_*) are untouched.
    """
    branch_pattern = re.compile(r'def Xform "(cyl_branchL_\d+_Id_\d+)"')
    joint_pattern = re.compile(r'(def PhysicsRevoluteJoint ")HingeJoint_(\d+)_(\d+)_([XY])(")')

    out = []
    pos = 0
    renamed = 0
    for match in branch_pattern.finditer(usda_content):
        branch_name = match.group(1)
        open_brace = usda_content.find('{', match.end())
        depth, end = 1, open_brace + 1
        while depth and end < len(usda_content):
            if usda_content[end] == '{':
                depth += 1
            elif usda_content[end] == '}':
                depth -= 1
            end += 1
        if depth:
            continue

        block, n = joint_pattern.subn(
            lambda m: f'{m.group(1)}HingeJoint_{branch_name}_{m.group(2)}_{m.group(3)}_{m.group(4)}{m.group(5)}',
            usda_content[open_brace:end])
        out.append(usda_content[pos:open_brace])
        out.append(block)
        pos = end
        renamed += n
    out.append(usda_content[pos:])

    print(f"Renamed {renamed} internal branch joints")
    return "".join(out)


def main():
    input_file = Path(sys.argv[1] if len(sys.argv) > 1 else 'urdf/mod_lpy_ufo_with_attachments.usda')
    output_file = Path(sys.argv[2] if len(sys.argv) > 2 else 'urdf/mod_lpy_ufo_with_attachments_unique_joints.usda')
    
    print(f"Reading {input_file}...")
    with open(input_file, 'r') as f:
        content = f.read()
    
    print("Renaming internal branch joints...")
    content = rename_branch_joints(content)
    
    print(f"\nSaving to {output_file}...")
    with open(output_file, 'w') as f:
        f.write(content)
    
    # Verify
    import re
    duplicate_check = {}
    for match in re.finditer(r'def PhysicsRevoluteJoint "([^"]+)"', content):
        name = match.group(1)
        duplicate_check[name] = duplicate_check.get(name, 0) + 1
    
    duplicates = [name for name, count in duplicate_check.items() if count > 1]
    
    if duplicates:
        print(f"\n✗ WARNING: Still have {len(duplicates)} duplicate joint names:")
        for name in sorted(duplicates)[:5]:
            print(f"    {name}: {duplicate_check[name]} times")
    else:
        print(f"\n✓ All joint names are now unique! ({len(duplicate_check)} total joints)")
    
    print(f"✓ Done! Saved to {output_file}")


if __name__ == '__main__':
    main()
