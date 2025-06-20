import distutils.util
import os

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

# Other imports and helper functions
import time
import itertools
import numpy as np

# Graphics and plotting.
import mediapy as media
import matplotlib.pyplot as plt

# More legible printing from numpy.
np.set_printoptions(precision=3, suppress=True, linewidth=100)

from IPython.display import clear_output
clear_output()

from pyvirtualdisplay import Display 
display = Display(visible = 0, size=(1400, 900))
display.start()

xml = """
<mujoco model="single_pendulum">
  <compiler angle="radian"/>

  <worldbody>
    <geom size="0.025 0.025 0.025" type="box" rgba="0 0 1 1"/>
    <body name="branch_link">
      <inertial pos="0 0 0.395" mass="0.05052" diaginertia="0.01051 0.01051 9.338e-07"/>
      <joint name="base_branch_joint" pos="0 0 0" axis="0 1 0"/>
      <geom size="0.00608 0.395" pos="0 0 0.395" type="cylinder" rgba="0 1 0 1"/>
    </body>
  </worldbody>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

mujoco.viewer.launch(model, data)

# ctx = mujoco.GLContext(1000, 1000)
# ctx.make_current()

# with mujoco.Renderer(model) as renderer:
#   mujoco.mj_forward(model, data)
#   renderer.update_scene(data)

#   media.show_image(renderer.render())