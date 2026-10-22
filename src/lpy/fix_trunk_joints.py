"""
Fix trunk cylinder joint pivot positions to align correctly at cylinder ends.
Ensures that joints are positioned at the actual connection points between cylinders.
"""
import re
from pathlib import Path


def extract_cylinder_height(content, cyl_num):
    """Extract the height of a specific cylinder."""
    pattern = rf'def Cylinder "cyl_{cyl_num}"[^}}]*?double height = ([\d.e+-]+)'
    match = re.search(pattern, content, re.DOTALL)
    if match:
        return float(match.group(1))
    return None


def fix_trunk_joints(usda_content):
    """Fix joint pivot positions for trunk cylinder connections."""
    
    # Extract all cylinder heights
    cyl_heights = {}
    for match in re.finditer(r'def Cylinder "cyl_(\d+)"[^}]*?double height = ([\d.e+-]+)', usda_content, re.DOTALL):
        cyl_num = int(match.group(1))
        height = float(match.group(2))
        cyl_heights[cyl_num] = height
    
    print(f"Found {len(cyl_heights)} cylinders with heights")
    
    # Define corrections for known problematic trunk joints
    # Each entry: (joint_name, cyl_from, cyl_to, expected_body0_half_height, expected_body1_half_height)
    corrections = []
    
    # HingeJoint_34_35 connects cyl_33 → cyl_34
    # body0 should be at cyl_33's end: -height_33/2
    # body1 should be at cyl_34's start: +height_34/2
    if 33 in cyl_heights and 34 in cyl_heights:
        h33 = cyl_heights[33]
        h34 = cyl_heights[34]
        corrections.append({
            'name': 'HingeJoint_34_35',
            'body0_half': -h33 / 2,
            'body1_half': h34 / 2,
            'description': f"cyl_33→cyl_34: {h33/2:.9f} → {h34/2:.9f}"
        })
    
    # HingeJoint_35_36 connects cyl_34 → cyl_35
    # body0 should be at cyl_34's end: -height_34/2
    # body1 should be at cyl_35's start: +height_35/2
    if 34 in cyl_heights and 35 in cyl_heights:
        h34 = cyl_heights[34]
        h35 = cyl_heights[35]
        corrections.append({
            'name': 'HingeJoint_35_36',
            'body0_half': -h34 / 2,
            'body1_half': h35 / 2,
            'description': f"cyl_34→cyl_35: {h34/2:.9f} → {h35/2:.9f}"
        })
    
    print(f"\nApplying {len(corrections)} joint position corrections:")
    
    # Apply corrections
    for correction in corrections:
        joint_name = correction['name']
        body0_z = correction['body0_half']
        body1_z = correction['body1_half']
        
        # Find and replace localPos for both X and Y variants
        for axis in ['X', 'Y']:
            full_joint_name = f"{joint_name}_{axis}"
            
            # Pattern to find the joint and its localPos definitions
            pattern = (
                rf'(def PhysicsRevoluteJoint "{re.escape(full_joint_name)}"[^{{]*\{{[^}}]*?)'
                rf'point3f physics:localPos0 = \([^)]+\)'
                rf'(.*?)'
                rf'point3f physics:localPos1 = \([^)]+\)'
            )
            
            def replacer(match):
                prefix = match.group(1)
                middle = match.group(2)
                return (
                    f'{prefix}point3f physics:localPos0 = (0, 0, {body0_z:.9g}){middle}'
                    f'point3f physics:localPos1 = (0, 0, {body1_z:.9g})'
                )
            
            before_len = len(usda_content)
            usda_content = re.sub(pattern, replacer, usda_content, flags=re.DOTALL)
            after_len = len(usda_content)
            
            if before_len != after_len:
                print(f"  ✓ Fixed {full_joint_name}: localPos0=(0,0,{body0_z:.9g}), localPos1=(0,0,{body1_z:.9g})")
            else:
                print(f"  ✗ Could not find {full_joint_name}")
    
    return usda_content


def main():
    input_file = Path('urdf/mod_lpy_ufo_with_attachments.usda')
    output_file = Path('urdf/mod_lpy_ufo_with_attachments_fixed.usda')
    
    print(f"Reading {input_file}...")
    with open(input_file, 'r') as f:
        content = f.read()
    
    print("Fixing trunk joint positions...")
    content = fix_trunk_joints(content)
    
    print(f"\nSaving to {output_file}...")
    with open(output_file, 'w') as f:
        f.write(content)
    
    print(f"✓ Done! Fixed USDA saved to {output_file}")


if __name__ == '__main__':
    main()
