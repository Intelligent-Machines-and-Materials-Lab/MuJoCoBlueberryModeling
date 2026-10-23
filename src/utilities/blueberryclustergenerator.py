"""
Generates rigid geometries for blueberry clusters of varying numbers and masses of blueberries. 
Saves the output geometries as xml files. 
"""
import random
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation as R 

BROWN = np.array([0.4, 0.24, 0.0, 1])
STEMGREEN = np.array([.70, .76, .45, 1])
BERRYBLUE1 = np.array([.3, .345, .557, 1])
BERRYBLUE2 = np.array([.2, .34, .47, 1])
BERRYBLUE3 = np.array([.145, .251, .365, 1])
UNRIPEPURPLE = np.array([.54, .46, .62, 1])
UNRIPERED = np.array([.89, .72, .7, 1])
UNRIPEGREEN = np.array([.7, .78, .67, 1])
BERRYCOLORS = [BERRYBLUE1, BERRYBLUE2, BERRYBLUE3, UNRIPEPURPLE, UNRIPERED, UNRIPEGREEN]
COLORWEIGHTS = [30, 30, 30, 6, 1, 3]

def xyz_to_mj_quat(x, y, z, deg=True):
    xyzw = R.from_euler('xyz', [x, y, z], degrees=deg).as_quat()
    quat_mj = [xyzw[3], xyzw[0], xyzw[1], xyzw[2]]
    return quat_mj

class BlueberryClusterGenerator:
    def __init__(self, def_mass=.0033, def_rad=.0065):
        self.def_blueberry_mass = def_mass # 3.3g
        self.def_blueberry_rad = def_rad # 6.5mm 
        self.stem_rad = .0005 # 0.5mm
        self.stem_len_bounds = [.002, .013]

        self.berry_counter = 0

    def change_berry_mass(self, new_mass=None):
        if new_mass is not None:
            old_mass = self.def_blueberry_mass
            self.def_blueberry_mass = new_mass
            # change volume proportionally to the new mass
            new_rad = (self.def_blueberry_rad**3 * new_mass / old_mass) ** (1/3)
            self.def_blueberry_rad = new_rad

    def gen_single_berry(self):
        spec = mujoco.MjSpec()
        root = spec.worldbody.add_body(name=f'root_cluster{self.berry_counter}')

        stem_len = random.uniform(self.stem_len_bounds[0], self.stem_len_bounds[1])
        root.add_geom(name=f"stem_geom{self.berry_counter}",
                      type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                      pos=[0, 0, stem_len/2],
                      size=[self.stem_rad, stem_len/2, 0],
                      rgba=STEMGREEN,
                      mass=.0005)
        blueberry_body=root.add_body(name=f"blueberry_body{self.berry_counter}", pos=[0, 0, stem_len])
        rand_blue = random.choices(BERRYCOLORS, weights=COLORWEIGHTS, k=1)[0]
        rand_size = random.gauss(self.def_blueberry_rad, self.def_blueberry_rad*0.1)
        # change the mass proportional to the size of the berry
        rand_mass = self.def_blueberry_mass * (rand_size / self.def_blueberry_rad) ** 3
        blueberry_body.add_geom(name=f"blueberry_geom{self.berry_counter}",
                               type=mujoco.mjtGeom.mjGEOM_SPHERE,
                               pos=[0, 0, rand_size],
                               size=[rand_size, 0, 0],
                               rgba=rand_blue,
                               mass=rand_mass)

        self.berry_counter += 1
        return spec

    def gen_two_berries(self):
        spec = mujoco.MjSpec()
        offset = 20 # degrees
        quat_list = [xyz_to_mj_quat(offset, 0, 0), xyz_to_mj_quat(-offset, 0, 0)]
        for i in range(2):
            single_spec = self.gen_single_berry()
            mount_frame = spec.worldbody.add_frame(name=f"berry_mount_{i}")
            mount_frame.quat = quat_list[i]
            spec.attach(single_spec, frame=mount_frame, prefix=f"berry{i}_")
        return spec

    def gen_three_berries(self):
        spec = mujoco.MjSpec()
        offset = 20 # degrees
        quat_list = [xyz_to_mj_quat(offset, 0, 0), xyz_to_mj_quat(-offset, 0, 0), xyz_to_mj_quat(0, 1.5*offset, 0)]
        for i in range(3):
            single_spec = self.gen_single_berry()
            mount_frame = spec.worldbody.add_frame(name=f"berry_mount_{i}")
            mount_frame.quat = quat_list[i]
            spec.attach(single_spec, frame=mount_frame, prefix=f"berry{i}_")
        return spec

    def gen_berry_cluster(self, num_berries):
        spec = mujoco.MjSpec()
        max_angle_change = 40 #degrees
        # generate a main stem proportional to the number of berries
        root = spec.worldbody.add_body(name=f'root_cluster{self.berry_counter}')
        main_stem_len = self.def_blueberry_rad*1.5 + num_berries*self.stem_rad*2*1.5
        root.add_geom(name=f"main_stem_geom{self.berry_counter}",
                      type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                      pos=[0, 0, main_stem_len/2],
                      size=[self.stem_rad, main_stem_len/2, 0],
                      rgba=STEMGREEN,
                      mass=.0005)

        berry_positions = np.linspace(self.def_blueberry_rad, main_stem_len, num_berries)
        stem_quats = []
        for i in range(num_berries):
            single_spec = self.gen_single_berry()
            mount_frame = spec.worldbody.add_frame(name=f"berry_mount_{i}")
            mount_frame.pos = [0, 0, berry_positions[i]]
            quat = xyz_to_mj_quat(random.uniform(-max_angle_change, max_angle_change), random.uniform(-max_angle_change, max_angle_change), random.uniform(-max_angle_change, max_angle_change))
            if i > 0:
                is_diff_dir = False
                while not is_diff_dir:
                    quat = xyz_to_mj_quat(random.uniform(-max_angle_change, max_angle_change), random.uniform(-max_angle_change, max_angle_change), random.uniform(-max_angle_change, max_angle_change))
                    previous_start = max(0, i - 2)
                    angle_changes = [
                        np.arccos(np.clip(np.dot(prev_quat, quat), -1.0, 1.0)) * 180 / np.pi
                        for prev_quat in stem_quats[previous_start:i]
                    ]
                    for previous_index, angle_change in enumerate(angle_changes, start=previous_start):
                        # print(f"Angle change between berry {previous_index} and berry {i}: {angle_change:.2f} degrees")
                        pass
                    is_diff_dir = all(angle_change > 30 for angle_change in angle_changes)
                    if not is_diff_dir:
                        # print("Not different enough, retrying...")
                        pass
            mount_frame.quat = quat
            stem_quats.append(quat)                
            spec.attach(single_spec, frame=mount_frame, prefix=f"berry{i}_")

        return spec


    