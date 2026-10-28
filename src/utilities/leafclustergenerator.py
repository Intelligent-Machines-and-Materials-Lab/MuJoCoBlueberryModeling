"""
Generates rigid geometries for leaf clusters of varying numbers of leaves.
Attaches the generated clusters to branches.
"""

import os
import logging
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation as R
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra

STEMGREEN = STEMGREEN = np.array([.70, .76, .45, 1]) # from blueberryclustergenerator
LEAF_GREEN = np.array([0.25, 0.45, 0.15, 1])
# for debugging: color leaves by variant index (red, blue, yellow, magenta, cyan, ...)
DEBUG_VARIANT_COLORS = [np.array([0.9, 0.1, 0.1, 1]), np.array([0.1, 0.3, 0.9, 1]), np.array([0.95, 0.85, 0.1, 1]),
                        np.array([0.8, 0.1, 0.8, 1]), np.array([0.1, 0.8, 0.8, 1])]

DEFAULT_LEAF_OBJ = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../urdf/leaves_decimated.obj')

logger = logging.getLogger(__name__)

def surface_distances(verts, faces, source):
    """
    Returns the distance from vertex `source` to every vertex, walking along the mesh edges.
    Disconnected vertices get inf.
    """
    edges = np.vstack([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]])
    lengths = np.linalg.norm(verts[edges[:, 0]] - verts[edges[:, 1]], axis=1)
    graph = coo_matrix((lengths, (edges[:, 0], edges[:, 1])), shape=(len(verts), len(verts))).tocsr()
    return dijkstra(graph, directed=False, indices=source)

def find_stem_tip(verts, faces, blade_tip):
    """
    Returns the index of the vertex farthest from blade_tip when walking along the mesh surface,
    i.e. the free end of the petiole, even if the petiole bends away from the blade.
    """
    dist = surface_distances(verts, faces, blade_tip)
    return np.argmax(np.where(np.isfinite(dist), dist, -1))  # ignore any disconnected vertices

def find_petiole_dir(verts, faces, stem_tip, frac=0.1):
    """
    Returns the unit direction the petiole points out of the leaf at its free end: from the stem tip
    toward the centroid of vertices within `frac` of the leaf's surface length of the stem tip.
    """
    dist = surface_distances(verts, faces, stem_tip)
    total = dist[np.isfinite(dist)].max()
    near = (dist > 0) & (dist < frac * total)
    direction = verts[near].mean(axis=0) - verts[stem_tip]
    return direction / np.linalg.norm(direction)

def load_leaf_variants(obj_path, flipped_variants=()):
    """
    Reads a Blender OBJ containing one or more leaves (one 'o' object each) and returns a list of
    (verts, faces) arrays, one per leaf, in a leaf frame:
        - petiole (stem) tip at the origin
        - +z points along the petiole, so attaching along +z keeps the petiole in line with its parent stem
        - +x points toward the side the blade bends to, so tilting the leaf toward +x (outward) bends the
          blade away from the parent stem instead of back across it. Leaves that barely bend use
          -z_b (the lower surface in Blender) as +x instead.
        - stem tip to blade tip distance is normalized to 1
    Assumes the Blender export has each leaf's blade tip at its -y end.
    The stem tip is found by find_stem_tip, so the petiole can stick out at an angle.
    flipped_variants: indices of leaves to turn over (180 deg about +z), which makes the blade
                      bend toward the parent stem instead of away from it.
    """
    verts, objects, faces = [], [], {}
    current = None
    with open(obj_path, 'r') as f:
        for line in f:
            if line.startswith('o '):
                current = line[2:].strip()
                faces[current] = []
            elif line.startswith('v '):
                verts.append([float(x) for x in line.split()[1:4]])
                objects.append(current)
            elif line.startswith('f '):
                idx = [int(tok.split('/')[0]) for tok in line.split()[1:]]
                idx = [i - 1 if i > 0 else len(verts) + i for i in idx]
                # fan-triangulate quads/polygons
                faces[current] += [[idx[0], idx[k], idx[k+1]] for k in range(1, len(idx) - 1)]
    verts = np.array(verts)
    objects = np.array(objects)

    variants = []
    # 180 deg about the leaf axis (z): normal and width directions flip, stem-to-tip direction doesn't
    turn_over = np.diag([-1, -1, 1])
    for i, (obj_name, obj_faces) in enumerate(faces.items()):
        global_ids = np.flatnonzero(objects == obj_name)
        v = verts[global_ids]
        remap = {g: j for j, g in enumerate(global_ids)}
        fc = np.array([[remap[j] for j in tri] for tri in obj_faces])

        blade_tip = np.argmin(v[:, 1])
        stem_tip = find_stem_tip(v, fc, blade_tip)
        # leaf frame (rows, in blender coords): z_leaf = petiole direction,
        # x_leaf = direction the blade bends to (perpendicular to z_leaf), y_leaf = z_leaf x x_leaf
        z_axis = find_petiole_dir(v, fc, stem_tip)
        chord = v[blade_tip] - v[stem_tip]
        leaf_len = np.linalg.norm(chord)
        x_axis = chord - chord.dot(z_axis) * z_axis
        if np.linalg.norm(x_axis) < 0.15 * leaf_len:  # blade nearly in line with petiole (< ~9 deg)
            x_axis = np.array([0.0, 0.0, -1.0])
            x_axis -= x_axis.dot(z_axis) * z_axis
        x_axis /= np.linalg.norm(x_axis)
        blender_to_leaf = np.vstack([x_axis, np.cross(z_axis, x_axis), z_axis])

        v = (v - v[stem_tip]) @ blender_to_leaf.T / leaf_len  # stem tip to blade tip = 1
        if i in flipped_variants:
            v = v @ turn_over.T
        fc = np.vstack([fc, fc[:, ::-1]])  # add reversed faces so the leaf renders from both sides
        variants.append((v, fc))
    logger.debug(f"Loaded {len(variants)} leaf variants from {obj_path}")
    return variants

class LeafClusterGenerator:
    def __init__(self, spec, obj_path=DEFAULT_LEAF_OBJ):
        """
        spec: the editor's MjSpec. Leaf meshes are added to it once and shared by every leaf geom,
              rather than attaching a separate spec (and mesh copy) per leaf.
        """
        # https://leafandstem.website/blueberry-leaf-identification/
        # northern highbush leaves are 1 to 2.5" long --> 0.0254 to 0.0635 m
        self.avg_leaf_len = .04445
        self.leaf_len_dev = .00635 # creates 3 std devs to cover typical leaf length variation
        self.leaf_mass = .0003
        self.stem_rad = .001 # 1mm
        self.stem_mass_per_len = .003 # kg/m, ~ green wood (density ~900 kg/m^3) at 1mm radius
        # mesh scale is fixed per mesh asset, so sampled lengths are snapped to these bins
        # (one mesh per variant per bin) instead of making a new mesh for every leaf
        self.leaf_len_bins = self.avg_leaf_len + self.leaf_len_dev * np.array([-2, -1, 0, 1, 2])

        self.spec = spec
        self.obj_path = obj_path
        self.leaf_variants = None  # loaded lazily from the OBJ on first gen_single_leaf call
        # leaves to turn over so they bend toward their parent stem; empty = every leaf bends away
        self.flipped_variants = set()
        self.leaf_meshes = set()   # names of leaf meshes already added to the spec
        self.debug_variant_colors = False  # set True to color leaves by variant (see DEBUG_VARIANT_COLORS)

        self.leaf_counter = 0
        self.leaf_stem_counter = 0

    def get_leaf_mesh(self, variant, leaf_len):
        """
        Returns the name of the mesh for this variant and length bin, adding it to the spec if it isn't there yet.
        """
        mesh_name = f"leaf_mesh_v{variant}_{leaf_len*1000:.0f}mm"
        if mesh_name not in self.leaf_meshes:
            verts, faces = self.leaf_variants[variant]
            self.spec.add_mesh(name=mesh_name,
                               uservert=(verts * leaf_len).flatten(),
                               userface=faces.flatten(),
                               inertia=mujoco.mjtMeshInertia.mjMESH_INERTIA_SHELL)  # leaf is a surface, not a solid
            self.leaf_meshes.add(mesh_name)
            logger.debug(f"Added leaf mesh {mesh_name}")
        return mesh_name

    def gen_single_leaf(self, parent_body, pos, quat, variant=None):
        """
        Adds one leaf geom to parent_body. pos/quat place the leaf frame (stem tip at the origin,
        blade along +z) in the parent body's frame.
        variant: index of the leaf shape in the OBJ; random if None
        """
        if self.leaf_variants is None:
            self.leaf_variants = load_leaf_variants(self.obj_path, self.flipped_variants)
        if variant is None:
            variant = np.random.randint(len(self.leaf_variants))

        sampled_len = np.random.normal(self.avg_leaf_len, self.leaf_len_dev)
        leaf_len = self.leaf_len_bins[np.argmin(np.abs(self.leaf_len_bins - sampled_len))]
        # change the mass proportional to the area of the leaf
        leaf_mass = self.leaf_mass * (leaf_len / self.avg_leaf_len) ** 2
        if self.debug_variant_colors:
            rgba = DEBUG_VARIANT_COLORS[variant % len(DEBUG_VARIANT_COLORS)]
        else:
            rgba = LEAF_GREEN

        geom = parent_body.add_geom(name=f"leaf_geom{self.leaf_counter}",
                                    type=mujoco.mjtGeom.mjGEOM_MESH,
                                    meshname=self.get_leaf_mesh(variant, leaf_len),
                                    pos=pos,
                                    quat=quat,
                                    mass=leaf_mass,
                                    rgba=rgba,
                                    contype=0, conaffinity=0)  # visual + mass only, no collisions

        self.leaf_counter += 1
        return geom

    def gen_leaf_cluster(self, parent_body, pos, quat, num_leaves):
        """
        Generates a cluster of leaves on a welded stem body under parent_body.
        pos/quat place the stem base in the parent body's frame; the stem extends along its +z.
        num_leaves: number of leaves to generate in the cluster
        Modeled off BlueberryGen.gen_berry_cluster
        """
        # create a stem that is the parent for all leaves in the cluster. Let's start with somewhere between 1 and 3cm per leaf.
        tip_cluster_size = min(np.random.randint(2, 5), num_leaves)  # last 2-4 leaves will be clustered near the tip
        stem_length_multiplier = np.random.uniform(.005, .03)
        logger.debug(f"Stem length multiplier: {stem_length_multiplier}")
        stem_length = stem_length_multiplier * (num_leaves - tip_cluster_size)
        # no joint, so the stem is welded to parent_body
        stem_body = parent_body.add_body(name=f"leaf_stem_{self.leaf_stem_counter}", pos=pos, quat=quat)
        stem_body.add_geom(name=f"leaf_stem_geom{self.leaf_stem_counter}",
                           type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                           pos=[0, 0, stem_length/2],
                           size=[self.stem_rad, stem_length/2, 0],
                           rgba=STEMGREEN,
                           mass=self.stem_mass_per_len * stem_length,
                           contype=0, conaffinity=0)  # visual + mass only, no collisions
        
        azimuth_start_pt = np.random.uniform(0, 360)
        # pick one variant to use for all leaves in this cluster
        if self.leaf_variants is None:
            self.leaf_variants = load_leaf_variants(self.obj_path, self.flipped_variants)
        this_cluster_variant = np.random.randint(len(self.leaf_variants))

        # position the leaves approximately equidistant along the stem, with 2-4 near the tip clustered closer together on the z axis
        # generate quaternions such that leaves alternate sides along the stem
        for i in range(num_leaves):
            tilt = np.random.normal(45.0, 10.0) 
            azimuth = azimuth_start_pt if i % 2 == 0 else azimuth_start_pt + 180.0  # alternate sides
          
            # start the leaf on the stem surface (on the side it leans toward) rather than the stem center
            if i >= num_leaves - tip_cluster_size:  # last 2-4 leaves
                azimuth = np.random.uniform(0, 360)
                stem_z = stem_length - np.random.uniform(0, (tip_cluster_size/num_leaves) * stem_length*.5)  # cluster closer together near the tip
            else:
                azimuth += np.random.uniform(-30, 30.0)  # add some randomness to the azimuth
                stem_z = stem_length * (i + 1) / (num_leaves-tip_cluster_size)

            pos = [self.stem_rad * np.cos(np.deg2rad(azimuth)), self.stem_rad * np.sin(np.deg2rad(azimuth)),
                   stem_z]
            quat = R.from_euler("z", azimuth, degrees=True) * R.from_euler("y", tilt, degrees=True)
            xyzw = quat.as_quat()
            self.gen_single_leaf(stem_body,
                                 pos=pos,
                                 quat=[xyzw[3], xyzw[0], xyzw[1], xyzw[2]],
                                 variant=this_cluster_variant)

        self.leaf_stem_counter += 1
        return stem_body
