'''
Using caneEditor and BranchSim to simulate several canes in one simulation
Started 9/21/26 by Hannah

Avg Legacy blueberry mass: 3.3g (https://www.ckdeliciouskitchen.com/en/products/Legacy-Blueberry.html)
Diameter: 10-16mm (https://eathealthy365.com/a-complete-guide-to-blueberry-sizes-wild-vs-cultivated/)
'''


import os
import argparse
import contextlib
import pickle
import time
import json

os.environ['MUJOCO_GL'] = 'glfw'
os.environ['PYOPENGL_PLATFORM'] = 'glx'
os.environ['LIBGL_ALWAYS_SOFTWARE'] = '0'

import numpy as np
import pandas as pd
import random
import matplotlib.pyplot as plt
import logging

try:
  print('Checking that the installation succeeded:')
  import mujoco
  import mujoco.viewer
  mujoco.MjModel.from_xml_string('<mujoco/>')
except Exception as e:
  raise e from RuntimeError(
      'Something went wrong during installation. Check the shell output above '
      'for more information.\n'
      'If using a hosted Colab runtime, make sure you enable GPU acceleration '
      'by going to the Runtime menu and selecting "Choose runtime type".')

print('Installation successful.')

import caneEditor
import splineconverter
from caneEditor import CaneEditor
from caneSimulator import BranchSim, flip_segs_from_curve, get_midpoints

def build_bush(editor, branch_origins_df, bush_num):
    B = BranchSim(bush_num, 1, editor=editor, xy=branch_origins_df[bush_num][1])
    B2 = BranchSim(bush_num, 2, editor=editor, xy=branch_origins_df[bush_num][2])
    # B3 = BranchSim(bush_num, 3, editor=editor, xy=branch_origins_df[bush_num][3])
    return B, B2#, B3

def make_berry_config(editor, bush_num, file_name):
    # Blueberry setup
    NUM_CLUSTERS_LOWER = 4
    NUM_CLUSTERS_UPPER = 10
    NUM_BERRIES_LOWER = 3
    NUM_BERRIES_UPPER = 16
    HEIGHT_LOWER = 0.35
    HEIGHT_UPPER = 0.75
    
    BushConfig = {"num_canes": editor.num_canes}

    for i in range(editor.num_canes):
        num_clusters = random.randint(NUM_CLUSTERS_LOWER, NUM_CLUSTERS_UPPER)
        BushConfig[f"B1_B{i+1}"] = {"num_clusters": num_clusters}
        for j in range(num_clusters):
            num_berries = random.randint(NUM_BERRIES_LOWER, NUM_BERRIES_UPPER)
            height = random.uniform(HEIGHT_LOWER, HEIGHT_UPPER)
            BushConfig[f"B1_B{i+1}"][f"cluster_{j+1}"] = {"num_berries": num_berries, "height": height}
            editor.add_hanging_berries_to_branch_at_height(height, f"B1_B{i+1}", num_berries)

    BERRY_CONFIGS_PATH = os.path.join(PATH_HERE, '../data/berry_configs')
    os.makedirs(BERRY_CONFIGS_PATH, exist_ok=True)
    with open(os.path.join(BERRY_CONFIGS_PATH, file_name), 'w') as f:
        json.dump(BushConfig, f, indent=4)

def load_berry_config(file_name):
    BERRY_CONFIGS_PATH = os.path.join(PATH_HERE, '../data/berry_configs')
    with open(os.path.join(BERRY_CONFIGS_PATH, file_name), 'r') as f:
        config = json.load(f)

    for i in range(config["num_canes"]):
        branch_key = f"B1_B{i+1}"
        if branch_key in config:
            for j in range(config[branch_key]["num_clusters"]):
                cluster_key = f"cluster_{j+1}"
                if cluster_key in config[branch_key]:
                    cluster = config[branch_key][cluster_key]
                    height = cluster["height"]
                    num_berries = cluster["num_berries"]
                    editor.add_hanging_berries_to_branch_at_height(height, branch_key, num_berries)

def plot_tip_positions(results, axis=2):
    '''
    Plot tip position along one axis (0=x, 1=y, 2=z) over time.
    One subplot per cane, one line per berry mass.
    results: {avg_mass: {"times": (N,) array, "tips": {cane_id: (N, 3) array}}}
    '''
    cane_ids = list(next(iter(results.values()))["tips"].keys())
    fig, axes = plt.subplots(len(cane_ids), 1, sharex=True, squeeze=False)
    for ax, b in zip(axes[:, 0], cane_ids):
        for avg_mass, run in results.items():
            ax.plot(run["times"], run["tips"][b][:, axis], label=f"{avg_mass*1000:.2f} g")
        ax.set_title(f"Cane {b}")
        ax.set_ylabel(f"Tip {'xyz'[axis]} (m)")
        ax.legend(title="Avg berry mass")
    axes[-1, 0].set_xlabel("Time (s)")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true", help="run without the MuJoCo viewer")
    args = parser.parse_args()

    np.set_printoptions(precision=3, suppress=True)

    PATH_HERE = os.path.dirname(os.path.abspath(__file__))
    XML_PATH = os.path.join(PATH_HERE, '../urdf/branch_enviro.xml')
    with open(XML_PATH, 'r') as f:
        BRANCH_XML = f.read()
    BRANCH_ORIGINS_PATH =os.path.join(PATH_HERE, '../data/ccCurvesOGPos/origin_pts.pkl')
    with open(BRANCH_ORIGINS_PATH, 'rb') as f:
        BRANCH_ORIGINS_DF = pickle.load(f)

    logger = logging.getLogger(__name__)
    logging.basicConfig(level=logging.INFO, 
                        format="%(name)s | %(levelname)s | %(message)s")
    logging.getLogger("caneEditor").setLevel(logging.INFO)
    logging.getLogger("caneSimulator").setLevel(logging.INFO)
    logging.getLogger("splineconverter").setLevel(logging.WARNING)
    logging.getLogger("leafclustergenerator").setLevel(logging.DEBUG)

    # {avg_mass: {"times": (N,) array, "tips": {cane_id: (N, 3) array}}}
    results = {}

    # for avg_mass in [.001, .00325, .0065]:
    for avg_mass in [.00325]:
        editor = CaneEditor(BRANCH_XML)
        # editor.LeafGen.debug_variant_colors = True  # enable debug coloring for leaves
        # Bush 23's origin wasn't in the right place, so either fix that or use other ones
        B1, B2 = build_bush(editor, BRANCH_ORIGINS_DF, 1)
        # make_berry_config(editor, 1, f'bush_{1}_config.json')
        logger.info(f"Building bush with average berry mass {avg_mass*1000:.2f} g")
        editor.BerryGen.change_berry_mass(avg_mass)  # example of changing the default berry mass before loading the config
        load_berry_config('bush_1_config.json')
        editor.add_leaf_cluster_to_branch_at_height(height=0.45, branch_id="B1_B1", tilt=45.0, num_leaves=10)
        editor.add_leaf_cluster_to_branch_at_height(height=0.6, branch_id="B1_B1", tilt=55.0, num_leaves=6)   

        # editor.print_xml_to_console()

        model = editor.model
        data = mujoco.MjData(model)
        mujoco.mj_resetData(model, data)
        # data.qpos[:] = B2.zero_pos  # Set the model position to the initial zero position
        mujoco.mj_forward(model, data)

        model.opt.timestep = .00002
        model.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICIT
        model.opt.solver = mujoco.mjtSolver.mjSOL_NEWTON 
        model.opt.tolerance = 1e-8

        init_warn_count = data.warning[mujoco.mjtWarning.mjWARN_BADQACC].number
        tip_ids = {b: model.site(f"tip_site_{b}").id for b in editor.cane_ids}
        # each step (or every Nth):

        DATA_CAP_RATE = 20 # Hz
        # .copy() is required: site_xpos rows are views into live MjData memory
        tip_positions = {b: [data.site_xpos[tip_ids[b]].copy()] for b in editor.cane_ids}
        cap_times = [data.time]
        next_cap_time = 1 / DATA_CAP_RATE
        
        # for i in range(data.ncon):
        #     contact = data.contact[i]
        #     geom1 = model.geom(contact.geom1).name
        #     geom2 = model.geom(contact.geom2).name
        #     print(geom1, geom2, "distance:", contact.dist)

        SETTLE_TIME = 10  # seconds to settle under gravity (no applied force)
        settle_steps = 0
        logger.info("Starting settle phase...")
        # nullcontext yields viewer=None, so headless runs skip all viewer calls
        viewer_ctx = contextlib.nullcontext() if args.headless else mujoco.viewer.launch_passive(model, data)
        with viewer_ctx as viewer:
            mujoco.mj_forward(model, data)
            if viewer:
                viewer.sync()
            while data.time < SETTLE_TIME and (viewer is None or viewer.is_running()):
                step_start = time.time()
                mujoco.mj_step(model, data)
                if data.warning[mujoco.mjtWarning.mjWARN_BADQACC].number > init_warn_count:
                    logger.error(f"Bad acceleration at settle step {settle_steps}")
                    raise RuntimeError("Simulation unstable during settle phase: bad acceleration detected")

                if data.time >= next_cap_time:
                    for b in editor.cane_ids:
                        tip_positions[b].append(data.site_xpos[tip_ids[b]].copy())
                    cap_times.append(data.time)
                    next_cap_time += 1 / DATA_CAP_RATE

                # real-time pacing only matters when watching; headless runs as fast as possible
                if viewer:
                    viewer.sync()
                    time.sleep(max(0, model.opt.timestep - (time.time() - step_start)))
            if data.time >= SETTLE_TIME:
                logger.info(f"Settle phase complete at time {data.time} seconds")
                data.time = 0.0  # reset clock so DURATION counts from settled state
            while viewer and viewer.is_running():
                viewer.sync()
                time.sleep(0.01)

        results[avg_mass] = {
            "times": np.array(cap_times),
            "tips": {b: np.array(tip_positions[b]) for b in editor.cane_ids},
        }

        mass_str = f"{avg_mass*1000:.2f}_g".replace(".", "_")
        editor.save_picture_of_model_script(data.qpos, f"images/berryloadpics/bush_{mass_str}.png")

    RESULTS_PATH = os.path.join(PATH_HERE, '../data/tip_positions')
    os.makedirs(RESULTS_PATH, exist_ok=True)
    with open(os.path.join(RESULTS_PATH, 'bush_1_mass_sweep.pkl'), 'wb') as f:
        pickle.dump(results, f)

    plot_tip_positions(results)