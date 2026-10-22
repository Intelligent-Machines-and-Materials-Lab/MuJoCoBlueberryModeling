"""
Rename internal branch joints (HingeJoint_N_M_X/Y) to include branch ID for uniqueness.
This prevents MuJoCo constraint conflicts from duplicate joint names.

Pattern: HingeJoint_1_2_X → HingeJoint_branchL_0_Id_000_1_2_X
"""
import re
from pathlib import Path


def rename_branch_joints(usda_content):
    """Rename internal branch joints to include branch identifier."""
    
    # Find all branch definitions and their internal joints
    branch_pattern = r'def Xform "(cyl_branchL_\d+_Id_\d+)"'
    
    replacements = []
    
    for match in re.finditer(branch_pattern, usda_content):
        branch_name = match.group(1)
        start_pos = match.end()
        
        # Find the closing brace of this branch's Xform
        open_brace = usda_content.find('{', start_pos)
        if open_brace == -1:
            continue
        
        brace_count = 1
        search_pos = open_brace + 1
        while brace_count > 0 and search_pos < len(usda_content):
            if usda_content[search_pos] == '{':
                brace_count += 1
            elif usda_content[search_pos] == '}':
                brace_count -= 1
            search_pos += 1
        
        if brace_count != 0:
            continue
        
        # Extract the branch content
        branch_content = usda_content[open_brace:search_pos]
        
        # Find all HingeJoint_N_M_X/Y joints in this branch
        # These are the INTERNAL joints (cyl_0→cyl_1, cyl_1→cyl_2, etc.)
        # NOT the attachment joints (HingeJoint_parent)
        joint_pattern = r'def PhysicsRevoluteJoint "(HingeJoint_(\d+)_(\d+)_[XY])"'
        
        for joint_match in re.finditer(joint_pattern, branch_content):
            old_name = joint_match.group(1)
            # Skip attachment joints
            if 'parent' in old_name:
                continue
            
            # Create unique name: HingeJoint_<branch>_N_M_X
            new_name = f"HingeJoint_{branch_name}_{joint_match.group(2)}_{joint_match.group(3)}"
            
            replacements.append({
                'old': old_name,
                'new': new_name,
                'branch': branch_name,
            })
    
    # Apply replacements (in reverse order to avoid position shifts)
    print(f"Renaming {len(replacements)} internal branch joints for uniqueness...")
    
    for repl in replacements:
        old_name = repl['old']
        new_name = repl['new']
        
        # Replace all occurrences of the joint name
        old_pattern = rf'"{old_name}"'
        new_str = f'"{new_name}"'
        
        count_before = usda_content.count(old_pattern)
        usda_content = usda_content.replace(old_pattern, new_str)
        count_after = usda_content.count(old_pattern)
        
        if count_before > count_after:
            occurrences = count_before - count_after
            print(f"  ✓ {repl['branch']}: {old_name} → {new_name} ({occurrences} occurrences)")
    
    return usda_content


def main():
    input_file = Path('urdf/mod_lpy_ufo_with_attachments.usda')
    output_file = Path('urdf/mod_lpy_ufo_with_attachments_unique_joints.usda')
    
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
