'''
Using caneEditor and BranchSim to simulate several canes in one simulation
Started 9/21/26 by Hannah
'''

import os
import pickle
import time

os.environ['MUJOCO_GL'] = 'glfw'
os.environ['PYOPENGL_PLATFORM'] = 'glx'
os.environ['LIBGL_ALWAYS_SOFTWARE'] = '0'

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

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

if __name__ == "__main__":
    np.set_printoptions(precision=3, suppress=True)

    xml_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../urdf/branch_enviro.xml')
    with open(xml_path, 'r') as f:
        branch_xml = f.read()
    editor = CaneEditor(branch_xml)
    B = BranchSim(1, 1, editor=editor, xy=[.1, 0])
    B2 = BranchSim(1, 2, editor=editor, xy=[-.1, 0])
    B3 = BranchSim(1, 3, editor=editor, xy=[0, .1])
    # B2.editor.show_model_at_pos_script(B2.zero_pos)

    editor.print_xml_to_console()

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
    

    SETTLE_TIME = 0.1  # seconds to settle under gravity (no applied force)
    settle_steps = 0
    print("Starting settle phase...")
    with mujoco.viewer.launch_passive(model, data) as viewer:
        mujoco.mj_forward(model, data)
        viewer.sync()
        while data.time < SETTLE_TIME and settle_steps < 10000 and viewer.is_running():
            step_start = time.time()
            mujoco.mj_step(model, data)
            if data.warning[mujoco.mjtWarning.mjWARN_BADQACC].number > init_warn_count:
                print(f"Bad acceleration at settle step {settle_steps}")
                raise RuntimeError("Simulation unstable during settle phase: bad acceleration detected")
            settle_steps += 1
            viewer.sync()
            time.sleep(max(0, model.opt.timestep - (time.time() - step_start)))
        if settle_steps >= 10000:
            raise RuntimeError("Settle phase exceeded maximum iterations - possible infinite loop")
        if data.time >= SETTLE_TIME:
            print(f"Settle phase complete after {settle_steps} steps")
            data.time = 0.0  # reset clock so DURATION counts from settled state
        while viewer.is_running():
            viewer.sync()
            time.sleep(0.01)