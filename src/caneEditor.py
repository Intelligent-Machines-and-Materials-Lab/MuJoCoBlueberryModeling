"""
Let's build the tree!

This code block has everything we need to create an MJDF of a blueberry cane and view it. 

build_branch
- note that the y joint is technically before the x joint when building the system, though there is no space between them. 
- the y and x joints are both in reference to the body frame of the previous link, located at the end of the previous link. (I think.)
"""


import mujoco
import numpy as np
import mediapy as media
import matplotlib.pyplot as plt
import pygments
print_style = 'lovelace'
from IPython.display import HTML, display
from scipy.spatial.transform import Rotation as R 

BROWN = np.array([0.4, 0.24, 0.0, 1])

class CaneEditor():
    def __init__(self, xml_name, flex_mod = 4.9e9):
        self.spec = mujoco.MjSpec.from_string(xml_name)
        self.E = flex_mod  # Flexural modulus from average of 6 tested canes

        # default geom properties for wood branch
        self.spec.default.geom.density = 580 # kg/m^3 from Zhang
        self.spec.default.geom.solref = [0.1, 1]
        self.spec.default.geom.solimp = [0.95, 1, 0.0025, 1, 0.5]
        self.spec.default.geom.friction = [0, 0, 0]
        # self.model = self.spec.compile()

         # joint defaults
        self.spec.default.joint.type = mujoco.mjtJoint.mjJNT_HINGE

        self.spec.default.joint.springref = 0
        # self.spec.default.joint.damping = 1.0
        self.spec.default.joint.damping = 0.15

        # site defaults
        self.spec.default.site.type =mujoco.mjtGeom.mjGEOM_SPHERE
        self.spec.default.site.size = np.array([0.01, 0.01, 0.01])
        self.spec.default.site.rgba = np.array([0, 0, 0, 1])

    def build_branch_from_lengths(self, lengths, radii, def_stiff=295, verbose=False):
        """
        Procedurally builds the MJDF with the specified segment lengths and radii. Assumes no branching. 
        Calculates bending stiffnes based on beam bending theory and sets the joint stiffness accordingly.
        lengths: list of segment length in meters
        radii: list of segment radius in meters (assumes circular cross-section)
        """
        self.num_segments = len(lengths)
        self.spec.default.joint.stiffness = def_stiff
        self.total_length = sum(lengths)

        self.inverted_k_list = []

        # find the base body
        base_body = None
        for body in self.spec.bodies:
            if body.name == "branch_base":
                base_body = body
                break
        if base_body is None:
            raise ValueError("Branch base body not found in the model.")

        parent_body = base_body
        for i in range(self.num_segments):
            if verbose:
                print(f"Constructing segment {i} with length {lengths[i]:.3f} m")
                print(f"Radius of segment would be {radii[i]:.6f} m")
            body_name = f"branch_body_{i}"
            joint_name_y = f"branch_joint_y{i}"
            joint_name_x = f"branch_joint_x{i}"
            segment_geom_name = f"branch_geom{i}"
            site_name = f"joint_site{i}"

            seg_length = lengths[i]

            # Generate a random RGBA color for the segment
            rgba = np.random.uniform(size=4)
            rgba[3] = 1
            brown_variant = (rgba + BROWN*2) / 3

            # add child body to parent
            if parent_body == base_body:
                # start the first segment at the base
                child_body = parent_body.add_body(name=body_name, 
                                              pos=[0,0,0])
                # print(f"Adding body: {child_body.name} at position: {child_body.pos}")
            else:
                # start the subsequent segments at the end of the previous segment
                child_body = parent_body.add_body(name=body_name, 
                                                  pos=[0,0,lengths[i-1]])
                # print(f"Adding body: {child_body.name} at position: {child_body.pos}")
            # print(f"Adding body: {child_body.name} at position: {child_body.pos}")
            # add hinge to child body
            second_moment_area = (np.pi/4) * (radii[i]**4)
            k = 3*self.E*second_moment_area / seg_length
            self.inverted_k_list.append(1/k)
            if verbose:
                print(f"Estimated bending stiffness k via beam bending: {k:.1f} Nm/rad")
            child_body.add_joint(name=joint_name_x, axis=[1, 0, 0], stiffness=k)
            child_body.add_joint(name=joint_name_y, axis=[0, 1, 0], stiffness=k)
            child_body.add_site(name=site_name)  
            # add geometry to child body
            child_body.add_geom(name=segment_geom_name,
                                 pos=[0, 0, seg_length/2], 
                                 type=mujoco.mjtGeom.mjGEOM_BOX,
                                 size=[radii[i], radii[i], seg_length/2],
                                 rgba=brown_variant)
            parent_body = child_body

        self.model = self.spec.compile()
        self.k_eq = (sum(self.inverted_k_list))**(-1)

        # print("\nFinal body inertias:")
        # for b in range(self.model.nbody):
        #     m = self.model.body_mass[b]
        #     if m > 0:
        #         I_xx, I_yy, I_zz = self.model.body_inertia[b]
        #         print(f"  body {b} ({self.model.body(b).name}): mass={m:.6g} kg, inertia=[{I_xx:.6f}, {I_yy:.6f}, {I_zz:.6f}]")

            
    def build_branch(self, total_length, num_segments=2, radius=0.006, def_stiff=295):
        """ 
        Builds a branch with the specified number of segments and total length. 
        Makes equidistant segments.
        """
        self.total_length = total_length
        self.num_segments = num_segments
        segment_length = total_length/num_segments
        self.spec.default.geom.type = mujoco.mjtGeom.mjGEOM_CYLINDER

        self.spec.default.joint.stiffness = def_stiff

        base_body = None
        for body in self.spec.bodies:
            if body.name == "branch_base":
                base_body = body
                break
        if base_body is None:
            raise ValueError("Branch base body not found in the model.")
        
        parent_body = base_body
        for i in range(num_segments):
            body_name = f"branch_body_{i}"
            joint_name_y = f"branch_joint_y{i}"
            joint_name_x = f"branch_joint_x{i}"
            segment_geom_name = f"branch_geom{i}"
            site_name = f"joint_site{i}"

            # Generate a random RGBA color for the segment
            rgba = np.random.uniform(size=4)
            rgba[3] = 1
            brown_variant = (rgba + BROWN*2) / 3

            # add child body to parent
            if parent_body == base_body:
                # start the first segment at the base
                child_body = parent_body.add_body(name=body_name, 
                                              pos=[0,0,0])
            else:
                # start the subsequent segments at the end of the previous segment
                child_body = parent_body.add_body(name=body_name, 
                                                  pos=[0,0,segment_length])
            # print(f"Adding body: {child_body.name} at position: {child_body.pos}")
            # add hinge to child body
            child_body.add_joint(name=joint_name_y, axis=[0, 1, 0])
            child_body.add_joint(name=joint_name_x, axis=[1, 0, 0])
            child_body.add_site(name=site_name)    
            # add geometry to child body
            child_body.add_geom(name=segment_geom_name,
                                 pos=[0, 0, segment_length/2], 
                                 type=mujoco.mjtGeom.mjGEOM_BOX,
                                 size=[radius, radius, segment_length/2],
                                 rgba=brown_variant)
            parent_body = child_body

        # **Force MuJoCo to auto-compute inertias from geometry + density**
        self.spec.compile()  # First compile to set up geometry
        self.model = self.spec.compile()

        # Now explicitly tell MuJoCo to recompute all body inertias from their geoms
        for body in self.spec.bodies:
            body.inertia.data[:] = 0  # Clear any stale inertia
            
        self.model = self.spec.compile()  # Recompile—this time inertias auto-compute from geoms

        print("Body inertias after compile:")
        for b in range(self.model.nbody):
            I = self.model.body_inertia[3*b:3*b+3]
            print(f"  body {b} ({self.model.body(b).name}): {I}")

    def offset_joint_by_name(self, joint_name, angle):
        """
        Offsets the joint angle of a specified joint by a given angle.
        Note: this also sets the reference position for the angle. 
        """
        joints = self.spec.worldbody.find_all("joint")
        found = False
        for joint in joints:
            if joint.name == joint_name:
                joint.springref = angle
                found = True
                break
        if not found:
            raise ValueError(f"Joint '{joint_name}' not found.")
        # compile the model again to apply changes
        self.model = self.spec.compile()

    def offset_joint_by_dir_and_number(self, joint_dir, joint_number, angle):
        """
        Offsets the joint angle of a specified joint by a given angle.
        joint_dir: 'x' or 'y'
        joint_number: 0-indexed number of the joint in the branch
        """
        if joint_dir not in ['x', 'y']:
            raise ValueError("joint_dir must be 'x' or 'y'")
        
        joints = self.spec.worldbody.find_all("joint")
        if joint_number < 0 or joint_number >= len(joints):
            raise ValueError("Invalid joint number.")
        
        joint_name = f"branch_joint_{joint_dir}{joint_number}"
        self.offset_joint_by_name(joint_name, angle)

    def offset_all_joints_in_direction(self,  direction, angles):
        """
        Offsets all joints by the specified angles.
        angles: list of angles to offset each joint
        """
        if direction not in ['x', 'y']:
            raise ValueError("joint_dir must be 'x' or 'y'")
        if self.num_segments == len(angles):
            for i, angle in enumerate(angles):
                joint_name = f"branch_joint_{direction}{i}"
                self.offset_joint_by_name(joint_name, angle)
            
            self.model = self.spec.compile()
        else:
            raise ValueError("Number of angles must match the number of x joints.")
        
    def randomize_joint_angles(self):
        """
        Randomizes the branch angles within the specified range.
        """
        self.offset_all_joints_in_direction('x', np.random.normal(0, 0.2, self.num_segments))
        self.offset_all_joints_in_direction('y', np.random.normal(0, 0.2, self.num_segments))

    def show_camera_position(self, cam_id=0):
        with mujoco.Renderer(self.model) as renderer:
            cam = renderer.scene.camera[cam_id]
            print(f"Camera position: {cam.pos}")

    def get_zero_springref_pos(self):
        """
        Creates a keyframe at the springref angle for each of the joints in the branch.
        """
        self.model = self.spec.compile()
        data = mujoco.MjData(self.model)
        for joint in self.spec.worldbody.find_all("joint"):
            if joint.type == mujoco.mjtJoint.mjJNT_HINGE:
                # print(f"Setting joint {joint.name} springref to {joint.springref:3f}")
                data.qpos[joint.id] = joint.springref
        return data.qpos
    
    def define_probe_site(self, probe_height, model_pos, verbose=False):
        """
        Identify the last site before the specified probe height.
        Adds a new site at the probe height along the branch body after that site."""
        # raise a value error if probe height is above the total length of the branch
        if probe_height > self.total_length:
            print(f"Probe height {probe_height:.3f} m is above total length of branch {self.total_length:.3f} m. Check heights.")
            # raise ValueError("Probe height is above total length of branch. Please set a lower probe height.")

        # Remove existing probe_contact_site if it already exists
        for body in self.spec.bodies:
            for site in body.find_all("site"):
                if site.name == "probe_contact_site":
                    site.delete()
                    break

        # set up the model to and data to be bent (probably)
        self.model = self.spec.compile()
        data = mujoco.MjData(self.model)
        data.qpos[:] = model_pos  # Set the model position
        mujoco.mj_forward(self.model, data)

        # Find the last site before the probe height
        for i in range(self.model.nsite):
            site_xpos = data.site_xpos[i]
            # print(f"Site {i} world position: {site_xpos}")
            if site_xpos[2] < probe_height:
                # print(f"Site {i} is below probe height: {site_xpos[2]} < {probe_height}")
                last_site_i = i

        print(f"Last site before probe height is {last_site_i} with position {data.site_xpos[last_site_i]}")

        # get information about the last site
        name = self.model.site(last_site_i).name
        body_id = self.model.site_bodyid[last_site_i]
        body_name = self.model.body(body_id).name
        site_xpos = data.site_xpos[last_site_i]
        rotmat = data.xmat[body_id].reshape(3, 3)
        euler = R.from_matrix(rotmat).as_euler('xyz', degrees=False)

        if verbose:
            print("Last site before probe height:")
            print(f"Body name of {name}: {body_name}")
            print(f"World position of {name}: {site_xpos}")
            print(f"Local position of {name}: {self.model.site_pos[last_site_i]}")
            print(f"Body {self.model.body(body_id).name} is at location {data.xpos[body_id]}")
            print(f"Body {self.model.body(body_id).name} is at Euler angles {euler}")
        
        # add new site to the body at the probe height
        z_remainder = probe_height - site_xpos[2]
        hyp1 = z_remainder / np.cos(euler[0])
        hyp2 = hyp1 / np.cos(euler[1])
        print(f"distance along branch to probe site: {hyp2:.3f} m")

        for body in self.spec.bodies:
            if body.name == body_name:
                geoms = body.find_all("geom")
                if geoms:
                    self.probe_contact_branch_radius = geoms[0].size[0]  # Assuming the first geom is the branch
                    contact_branch_length = geoms[0].size[2]
                    if hyp2 < contact_branch_length*2:
                        body.add_site(name="probe_contact_site",
                              pos=[0, 0, hyp2],
                              rgba=[1, 0, 0, 1])
                        if verbose:
                            print(f"Added probe contact site to body {body_name} at local position [0, 0, {hyp2:.3f}]")
                    else:
                        raise ValueError("Probe position exceeds segment length in bent position. Please set a lower probe height.")
                else:
                    raise ValueError(f"No geoms found in body {body_name}. Check XML definition.")
                break
        self.model = self.spec.compile()

    
    def move_probe_to_site(self, probe_height, model_pos, verbose=False):
        """ 
        Redefines the probe position to be a little to the left of the new site. 
        """
        
        self.model = self.spec.compile()
        data = mujoco.MjData(self.model)
        data.qpos[:] = model_pos  # Set the model position
        mujoco.mj_forward(self.model, data)

        site_id = self.model.site("probe_contact_site").id
        new_site_xpos = data.site_xpos[site_id]
        
        init_probe_x = new_site_xpos[0] - 0.056 - self.probe_contact_branch_radius # offset to the left of the site
        for body in self.spec.bodies: 
            if body.name == "probe_link":
                body.pos = [init_probe_x, new_site_xpos[1], probe_height]

        self.model = self.spec.compile()

    def show_model_at_pos(self, pos):
        """
        Displays the model at the given position
        """
        data = mujoco.MjData(self.model)
        data.qpos[:] = pos  
        with mujoco.Renderer(self.model, height=480, width=640) as renderer:
            mujoco.mj_forward(self.model, data)
            renderer.update_scene(data)
            # cam = renderer.scene.camera[0]
            # print(f"Default camera position: {cam.pos}")
            media.show_image(renderer.render())

    def show_model_at_pos_script(self, pos):
        """
        Displays the model at the given position using matplotlib.
        Use this instead of show_model_at_pos() when running in a normal Python script.
        """
        data = mujoco.MjData(self.model)
        data.qpos[:] = pos
        with mujoco.Renderer(self.model, height=480, width=640) as renderer:
            mujoco.mj_forward(self.model, data)
            renderer.update_scene(data)
            img = renderer.render()
        plt.close('all')
        fig, ax = plt.subplots()
        ax.imshow(img)
        ax.axis('off')
        fig.tight_layout()
        plt.show()

    def save_picture_of_model(self, pos, filename):
        """
        Saves an image of the model at the given position to filename
        """
        data = mujoco.MjData(self.model)
        data.qpos[:] = pos  
        with mujoco.Renderer(self.model, height=480, width=640) as renderer:
            mujoco.mj_forward(self.model, data)
            renderer.update_scene(data)
            media.write_image(filename, renderer.render(), fmt='pdf')


    def show_model_at_pos_with_camera(self, pos, cam_id=0):
        """
        Displays the model at the given position with a specified camera.
        """
        data = mujoco.MjData(self.model)
        data.qpos[:] = pos  
        with mujoco.Renderer(self.model) as renderer:
            mujoco.mj_forward(self.model, data)
            renderer.update_scene(data, camera="x_cam")
            media.show_image(renderer.render())

    def show_new_model(self):
        self.model = self.spec.compile()
        data = mujoco.MjData(self.model)
        with mujoco.Renderer(self.model) as renderer:
            mujoco.mj_forward(self.model, data)
            renderer.update_scene(data)
            media.show_image(renderer.render())

    def print_xml(self):
        formatter = pygments.formatters.HtmlFormatter(style=print_style)
        lexer = pygments.lexers.XmlLexer()
        highlighted = pygments.highlight(self.spec.to_xml(), lexer, formatter)
        display(HTML(f"<style>{formatter.get_style_defs()}</style>{highlighted}"))