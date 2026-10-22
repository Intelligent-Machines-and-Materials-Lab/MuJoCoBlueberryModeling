"""
Remove joint definitions that reference non-existent cylinders.
This handles cases where the USDA file has malformed joints pointing to cylinders that don't exist.
"""
import re
from pathlib import Path


def find_all_cylinders(usda_content):
    """Extract all cylinder names that actually exist in the USDA."""
    cylinders = set()
    
    # Find all cylinder definitions
    pattern = r'def Cylinder "([^"]+)"'
    for match in re.finditer(pattern, usda_content):
        cyl_name = match.group(1)
        cylinders.add(cyl_name)
    
    return cylinders


def extract_cylinder_from_path(path_str):
    """Extract cylinder name from a physics body path like </...../cyl_35>."""
    match = re.search(r'/(cyl_\d+)>', path_str)
    if match:
        return match.group(1)
    return None


def find_invalid_joints(usda_content, existing_cylinders):
    """Find all joints that reference non-existent cylinders."""
    invalid_joints = []
    
    # Pattern to match joint definitions with their body references
    joint_pattern = r'def PhysicsRevoluteJoint "([^"]+)"[^{]*?\{[^}]*?prepend rel physics:body0 = ([^>]+>)[^}]*?prepend rel physics:body1 = ([^>]+>)[^}]*?\}'
    
    for match in re.finditer(joint_pattern, usda_content, re.DOTALL):
        joint_name = match.group(1)
        body0_path = match.group(2)
        body1_path = match.group(3)
        
        cyl0 = extract_cylinder_from_path(body0_path)
        cyl1 = extract_cylinder_from_path(body1_path)
        
        if (cyl0 and cyl0 not in existing_cylinders) or (cyl1 and cyl1 not in existing_cylinders):
            invalid_joints.append({
                'name': joint_name,
                'body0': cyl0,
                'body1': cyl1,
                'full_match': match.group(0)
            })
    
    return invalid_joints


def remove_joints_from_usda(usda_content, invalid_joints):
    """Remove invalid joint definitions from USDA content."""
    for joint in invalid_joints:
        # Remove the entire joint definition block
        # Look for the def PhysicsRevoluteJoint and remove everything until the closing }
        pattern = rf'def PhysicsRevoluteJoint "{re.escape(joint["name"])}"[^{{]*\{{[^}}]*\}}[^}}]*\}}\s*\n\n'
        
        before_len = len(usda_content)
        usda_content = re.sub(pattern, '\n', usda_content, flags=re.DOTALL)
        after_len = len(usda_content)
        
        if before_len != after_len:
            print(f"  ✓ Removed {joint['name']} ({joint['body0']} → {joint['body1']})")
        else:
            # Try a slightly different pattern (joint might be nested differently)
            pattern = rf'def PhysicsRevoluteJoint "{re.escape(joint["name"])}".*?\n{{\n.*?\n}}\n\n'
            before_len = len(usda_content)
            usda_content = re.sub(pattern, '\n', usda_content, flags=re.DOTALL)
            after_len = len(usda_content)
            
            if before_len != after_len:
                print(f"  ✓ Removed {joint['name']} ({joint['body0']} → {joint['body1']}) [alternate pattern]")
            else:
                print(f"  ✗ Failed to remove {joint['name']} ({joint['body0']} → {joint['body1']})")
    
    return usda_content


def main():
    input_file = Path('urdf/mod_lpy_ufo_split.usda')
    output_file = Path('urdf/mod_lpy_ufo_cleaned.usda')
    
    print(f"Reading {input_file}...")
    with open(input_file, 'r') as f:
        content = f.read()
    
    print("Finding all existing cylinders...")
    cylinders = find_all_cylinders(content)
    print(f"  Found {len(cylinders)} cylinders")
    
    print("\nScanning for invalid joint references...")
    invalid_joints = find_invalid_joints(content, cylinders)
    
    if not invalid_joints:
        print("  ✓ No invalid joints found!")
        return
    
    print(f"  Found {len(invalid_joints)} invalid joints:")
    for joint in invalid_joints:
        cyl0_status = "✗" if joint['body0'] and joint['body0'] not in cylinders else "✓"
        cyl1_status = "✗" if joint['body1'] and joint['body1'] not in cylinders else "✓"
        print(f"    - {joint['name']}: {cyl0_status} {joint['body0']} → {cyl1_status} {joint['body1']}")
    
    print("\nRemoving invalid joints...")
    content = remove_joints_from_usda(content, invalid_joints)
    
    print(f"\nSaving to {output_file}...")
    with open(output_file, 'w') as f:
        f.write(content)
    
    print(f"✓ Done! Cleaned USDA saved to {output_file}")


if __name__ == '__main__':
    main()
