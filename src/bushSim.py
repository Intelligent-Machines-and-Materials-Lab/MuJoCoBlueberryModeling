'''
Using caneEditor and BranchSim to simulate several canes in one simulation
Started 9/21/26 by Hannah
'''

import os 
import pickle

os.environ['MUJOCO_GL'] = 'glfw'
os.environ['PYOPENGL_PLATFORM'] = 'glx'
os.environ['LIBGL_ALWAYS_SOFTWARE'] = '0'

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

try:
  print('Checking that the installation succeeded:')
  import mujoco
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
    editor.add_branch_to_editor("B1_B1")
    editor.add_branch_to_editor("B1_B2")
    B = BranchSim(1, 1, editor=editor)
    B2 = BranchSim(1, 2, editor=editor)
    B2.editor.show_model_at_pos_script(B2.zero_pos)