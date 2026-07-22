'''
Hannah's attempt to roll branchSim.ipynb into a script so I 
can wrap it in a function and call it for each of the Morris samples.

Started 7/21/26.
'''

import os
import pickle

os.environ['MUJOCO_GL'] = 'glfw'
os.environ['PYOPENGL_PLATFORM'] = 'glx'
os.environ['LIBGL_ALWAYS_SOFTWARE'] = '0'

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

import numpy as np
import pandas as pd
import datetime
import matplotlib.pyplot as plt
from IPython.display import clear_output 
from simple_pid import PID
import splineconverter
from caneEditor import CaneEditor

from SALib.sample.morris import sample as morris_sample


def flip_segs_from_curve(segs, print_output=False):
    # Switch coordinate frames from spline (camera) to MuJoCo and center at origin
    trans_segs = np.zeros_like(segs)
    trans_segs[:,0] = segs[:,2].copy()-segs[-1,2]  # +x in mujoco is +z in camera ("forwards")
    trans_segs[:,1] = -segs[:,0].copy()+segs[-1,0]  # +y in mujoco is -x in camera ("left")
    trans_segs[:,2] = -segs[:,1].copy()+segs[-1,1]  # +z in mujoco is -y in camera ("up") (y was down in camera)
    segs_flipped = trans_segs[::-1]
    if print_output:
        print(segs_flipped)
    return segs_flipped

def get_midpoints(segs, print_output=False):
    midpoint_zs = []
    for i in range(len(segs)-1):
        midpoint = (segs[i] + segs[i+1]) / 2
        midpoint_zs.append(midpoint[2])
    midpoint_zs = np.array(midpoint_zs)
    if print_output:
        print(midpoint_zs)
    return midpoint_zs

class TrialSim():
    def __init__(self, BranchSim, TRIAL_NUM, force_angle=0):
        self.Branch = BranchSim
        self.TRIAL_NUM = TRIAL_NUM
        self.force_angle = force_angle
        self.metadata = {
            'sim_idx': [],
            'bush': [],
            'branch': [],
            'trial': [],
            'flex_modulus': [],
            'probe_height': [],
            'field_stiffness': [],
            'sim_stiffness': [],
            'force_error_10mm': [],
            'force_error_15mm': [],
            'force_error_20mm': [],
            'force_error_25mm': [],
            'force_error_30mm': [],
            'num_links': [],
            'discretization_RMSE': []
        }

        self.PROBE_HEIGHT = self.Branch.field_measurements.query('Location == ' + str(TRIAL_NUM))['Height'].values[0]/1000
        print(f"Setting probe height to: {self.PROBE_HEIGHT}")
        self.Branch.editor.define_probe_site(self.PROBE_HEIGHT, self.Branch.zero_pos, verbose=False)        
        
        # import forces from csv
        filename = os.path.join(DATA_DIR, 'imu_cropped_push_data/bush_' + str(BUSH_DICT[self.Branch.BUSH_NUM]) + '_branch_' + str(self.Branch.BRANCH_NUM) + '_trial_' + str(self.TRIAL_NUM) + '.csv')
        self.pushdata = pd.read_csv(filename)
        self.TRIAL_LENGTH = self.pushdata.shape[0]
        print(f"Loaded push data from {filename} with {self.TRIAL_LENGTH} rows.")

        self.run_trial()
        # self.plot_probe_steps()

        # Get linear fits for push data and simulation data
        self.fd_linearfit = np.polyfit(self.pushdata['actuator_displacement'], self.pushdata['Load (N)'], 1)
        self.sim_fd_linearfit = np.polyfit(self.discrete_results['Probe (mm)'].values, self.discrete_results['Force (N)'].values, 1)
        print(f"Effective stiffness from sim data: {self.sim_fd_linearfit[0]:.3f} N/mm")
        
        self.stiffness_error = self.get_stiffness_percentage_error()
        print(f"Stiffness percentage error: {self.stiffness_error*100:.2f}%")

        self.plot_force_displacement_comparison()

    def plot_probe_steps(self):
        dpi=120 
        width=1200 
        height=400
        figsize=(width/dpi, height/dpi)
        _, ax = plt.subplots(figsize=figsize, dpi=dpi)

        probevals = self.full_results['probevals']
        timevals = self.full_results['timevals']

        probearray = np.array(probevals)*1000  # convert from m to mm for easier comparison to push data
        ax.plot(timevals, probearray, label='Simulation Probe Displacement', color='blue')
        ax.set_title(f'Bush {self.Branch.BUSH_NUM}, Branch {self.Branch.BRANCH_NUM}, Trial {self.TRIAL_NUM}')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Probe Displacement (mm)')

        # Add a reference line that steps up by 0.001 every 1 second
        step_height = 0.5
        step_interval = .5
        step_times = np.arange(0, timevals[-1]+step_interval, step_interval)
        step_values = np.arange(0, step_height*len(step_times), step_height)
        # Interpolate to match the timevals for plotting as a step function
        step_ref = np.zeros_like(timevals)
        for i, t in enumerate(timevals):
            idx = np.searchsorted(step_times, t, side='right')
            step_ref[i] = step_values[idx-1] 
        ax.step(timevals, step_ref, where='post', linestyle='--', color='black', label='0.001 step every 1s')
        # ax.legend()
        ax.grid(True)
        plt.show()
    
    def calculate_absolute_errors(self, pushdata, metadata):
        pre_control_forcevals_array = self.discrete_results['Force (N)'].values
        pre_control_probeposes_array = self.discrete_results['Probe (mm)'].values
        
        # I'm lame and bad at code so this is how we're doing this
        try:
            sim_force_10mm = pre_control_forcevals_array[np.where(pre_control_probeposes_array >= 10)[0]]
            real_disp_10mm_force = pushdata.query('`actuator_displacement` >= 10')['Load (N)'].values[0]
            metadata['force_error_10mm'].append(abs(sim_force_10mm - real_disp_10mm_force))
        except:
            metadata['force_error_10mm'].append(np.nan)
        try:
            sim_force_15mm = pre_control_forcevals_array[np.where(pre_control_probeposes_array >= 15)[0]]
            real_disp_15mm_force = pushdata.query('`actuator_displacement` >= 15')['Load (N)'].values[0]
            metadata['force_error_15mm'].append(abs(sim_force_15mm - real_disp_15mm_force))
        except:
            metadata['force_error_15mm'].append(np.nan)
        try:
            sim_force_20mm = pre_control_forcevals_array[np.where(pre_control_probeposes_array >= 20)[0][0]]
            real_disp_20mm_force = pushdata.query('`actuator_displacement` >= 20')['Load (N)'].values[0]
            metadata['force_error_20mm'].append(abs(sim_force_20mm - real_disp_20mm_force))
        except:
            metadata['force_error_20mm'].append(np.nan)
        try:
            sim_force_25mm = pre_control_forcevals_array[np.where(pre_control_probeposes_array >= 25)[0][0]]
            real_disp_25mm_force = pushdata.query('`actuator_displacement` >= 25')['Load (N)'].values[0]
            metadata['force_error_25mm'].append(abs(sim_force_25mm - real_disp_25mm_force))
        except:
            metadata['force_error_25mm'].append(np.nan)
        try:
            sim_force_30mm = pre_control_forcevals_array[np.where(pre_control_probeposes_array >= 30)[0][0]]
            real_disp_30mm_force = pushdata.query('`actuator_displacement` >= 30')['Load (N)'].values[0]
            metadata['force_error_30mm'].append(abs(sim_force_30mm - real_disp_30mm_force))
        except:
            metadata['force_error_30mm'].append(np.nan)

    def run_trial(self):
        model = self.Branch.editor.model
        data = mujoco.MjData(model)
        data.qpos[:] = self.Branch.zero_pos
        mujoco.mj_forward(model, data)

        # simulation-specific settings
        model.opt.timestep = .00002
        model.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICIT
        model.opt.solver = mujoco.mjtSolver.mjSOL_NEWTON 
        model.opt.tolerance = 1e-8

        DURATION = 1 # TRIAL_LENGTH
        DATACAP_RATE = 20 # Hz
        init_warn_count = data.warning[mujoco.mjtWarning.mjWARN_BADQACC].number

        # variables to put data into lists for later plotting
        timevals = []
        forcevals = []
        probevals = []

        pre_control_timevals = []
        pre_control_forcevals = []
        pre_control_probeposes = []

        # -- Displacement controller setup --
        probe_site_id = model.site("probe_contact_site").id
        probe_body_id = int(model.site_bodyid[probe_site_id])
        probe_init_xpos = data.site_xpos[probe_site_id].copy()

        # -- Controller setup --
        pid = PID(Kp=150, Ki=22, Kd=10, setpoint=0)
        pid.output_limits = (-10, 50)  # force limits in Newtons

        pid.setpoint = 0 # mm, total displacement
        CTRL_POS_UPDATE_RATE = 2 # Hz, how often to update the control position
        last_force = 0.0
        print("Starting simulation...")

        with mujoco.Renderer(model, width=640, height=480) as renderer:
            while data.time < DURATION:
                    # -- measure current x-displacement of probe contact site --
                current_disp_m = data.site_xpos[probe_site_id][0] - probe_init_xpos[0]

                # -- PID: drive current displacement toward target --
                force = last_force + pid(current_disp_m)

                # -- apply force at probe contact site --
                data.qfrc_applied[:] = 0
                force_x = force*np.cos(self.Branch.editor.init_probe_angle)
                force_z = -force*np.sin(self.Branch.editor.init_probe_angle)
                mujoco.mj_applyFT(model, data,
                                np.array([force_x, 0.0, force_z]),  # force [N]
                                np.zeros(3),                     # torque
                                data.site_xpos[probe_site_id],  # point of application (world frame)
                                probe_body_id,
                                data.qfrc_applied)

                # step the simulation
                mujoco.mj_step(model, data)
                if data.warning[mujoco.mjtWarning.mjWARN_BADQACC].number > init_warn_count:
                    print("Acceleration is too damn high! Moving on...")
                    break
                
                # -- save data from sim  -- 
                if len(timevals) < data.time * DATACAP_RATE:
                    timevals.append(data.time)
                    forcevals.append(force)
                    probevals.append(current_disp_m)
                
                # update the control position at the specified rate
                if int(data.time * CTRL_POS_UPDATE_RATE) > int((data.time - model.opt.timestep) * CTRL_POS_UPDATE_RATE):
                    pre_control_timevals.append(data.time)
                    pre_control_forcevals.append(force)
                    pre_control_probeposes.append(current_disp_m)
                    last_force = force
                    print(f"Time: {data.time:.3f}s, Current Displacement: {current_disp_m*1000:.3f} mm, Force Applied: {force:.3f} N")
                    print(f"Probe angle: {np.degrees(self.Branch.editor.init_probe_angle):.2f}; Force components: x={force_x:.3f} N, z={force_z:.3f} N")
                    pid.setpoint += 0.0005  # increment by 0.5 mm (0.0005 m)

        self.full_results = {
            'timevals': np.array(timevals),
            'forcevals': np.array(forcevals),
            'probevals': np.array(probevals)
        }
        self.discrete_results = pd.DataFrame({
            'Time (s)': np.array(pre_control_timevals),
            'Force (N)': np.array(pre_control_forcevals),
            'Probe (mm)': np.array(pre_control_probeposes)*1000
        })

    def pickle_sim_data(self):
        pickle_filename = results_folder + '/sim_data_df_bush' + str(self.Branch.BUSH_NUM) + '_branch'+ str(self.Branch.BRANCH_NUM) + '_trial' + str(self.TRIAL_NUM) + '.pkl'
        with open(pickle_filename, 'wb') as f:
            pickle.dump(self.discrete_results, f)

    def get_real_force_at_disp(self, disp_in_mm):
        force = self.fd_linearfit[0]*disp_in_mm + self.fd_linearfit[1]
        return force
    
    def get_sim_force_at_disp(self, disp_in_mm):
        force = self.sim_fd_linearfit[0]*disp_in_mm + self.sim_fd_linearfit[1]
        return force
    
    def plot_force_displacement_comparison(self):
        fig = plt.figure()
        plt.plot(self.pushdata['actuator_displacement'], self.pushdata['Load (N)'], label='Measured Data', color='#377eb8')
        plt.plot(self.pushdata['actuator_displacement'], self.get_real_force_at_disp(self.pushdata['actuator_displacement']), label='Linear Fit', linestyle='--', color='#ff7f00')
        plt.plot(self.discrete_results['Probe (mm)'].values, self.discrete_results['Force (N)'].values, label='Simulator probe', linestyle='-', color='#4daf4a')
        plt.plot(self.discrete_results['Probe (mm)'].values, self.get_sim_force_at_disp(self.discrete_results['Probe (mm)'].values), label='Simulator Linear Fit', linestyle='--', color='#f781bf')
        plt.xlabel("Displacement (mm)")
        plt.ylabel("Load (N)")
        plt.legend()
        plt.grid()
        plt.show()

    def get_stiffness_percentage_error(self):
        real_stiffness = self.fd_linearfit[0]
        sim_stiffness = self.sim_fd_linearfit[0]
        error = abs(sim_stiffness - real_stiffness) / real_stiffness
        return error # between 0 and 1, where 0 is perfect match and 1 is 100% error

class BranchSim():
    def __init__(self, BUSH_NUM, BRANCH_NUM, flex_mod=4.9e9, num_segs=8):
        self.BUSH_NUM = BUSH_NUM
        self.BRANCH_NUM = BRANCH_NUM

        # Handle the one exceeption where the branch bends back on itself 
        if self.BUSH_NUM == 9 and self.BRANCH_NUM == 2:
            segs_mm, RMSE = self.deal_with_branch_9_2(num_segs)
        else: 
            segs_mm, RMSE = splineconverter.segment_curve_from_cloudcompare(os.path.join(DATA_DIR, 'ccCurves/B' + str(self.BUSH_NUM) + '_branch' + str(self.BRANCH_NUM) + '_smoothpolyline_minbb.txt'), 
                                                    make_plot=True, strictly_increasing=True, num_segs=num_segs)
        
        segs = np.array(segs_mm)/1000
        # Correctly orient the segments for MuJoCo (they're upside down from the camera)
        self.segs_flipped = flip_segs_from_curve(segs, print_output=False)
        # get the z-value of the midpoints of each segment
        midpoint_zs = get_midpoints(self.segs_flipped, print_output=True)

        # Get lengths of segments and angles between them (for setting up the MJCF model) 
        self.seg_lengths = splineconverter.get_segment_lengths(self.segs_flipped)
        self.seg_angles = splineconverter.get_angles_between_segments(self.segs_flipped)

        # get the diameter data for this particular cane and generate a linear fit
        self.field_measurements = diameter_data_df.query('Bush == ' + str(self.BUSH_NUM) + ' and Branch == ' + str(self.BRANCH_NUM))
        print(self.field_measurements)
        self.diam_linearfit = np.polyfit(self.field_measurements['Height'], self.field_measurements['Diameter'], 1)
        
        # Get the radii at the segment midpoints 
        self.radii = self.get_rad_at_height(midpoint_zs)
        print("Radii at segment midpoints (m): ", self.radii)

        self.build_mjcf_model(flex_modulus=flex_mod)

        self.editor.show_model_at_pos_script(self.zero_pos)
        
    def deal_with_branch_9_2(self, num_segs):
        angles_validated = False
        attempt = 1
        while not angles_validated:
            segs_mm, RMSE = splineconverter.segment_curve_from_cloudcompare(os.path.join(DATA_DIR, 'ccCurves/B' + str(self.BUSH_NUM) + '_branch' + str(self.BRANCH_NUM) + '_smoothpolyline_minbb.txt'),
                                                make_plot=True, strictly_increasing=False, num_segs=num_segs)
            segs_flipped = flip_segs_from_curve(segs_mm, print_output=False)
            seg_angles = splineconverter.get_angles_between_segments(segs_flipped) 
            if abs(max(seg_angles.min(), seg_angles.max(), key=abs))<np.pi/2:
                print(f"Valid segmentation found!!")
                angles_validated = True
            else:
                print(f"Invalid angles in segmentation attempt {attempt}; trying again.")
                attempt += 1
        return segs_mm, RMSE
    
    def get_rad_at_height(self, height_in_m):
        diameter = self.diam_linearfit[0]*height_in_m*1000 + self.diam_linearfit[1]
        return diameter/2/1000 # convert mm to m
    
    def build_mjcf_model(self, flex_modulus=4.9e9):        # read in the base XML for the branch model (this has the world and the probe, but not the segments of the branch yet)
        xml_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../urdf/branch_base.xml')
        with open(xml_path, 'r') as f:
            branch_xml = f.read()

        self.editor = CaneEditor(branch_xml, flex_mod=flex_modulus)
        self.editor.build_branch_from_lengths(self.seg_lengths, self.radii, verbose=False)
        self.editor.offset_all_joints_in_direction('x', self.seg_angles[:,0])
        self.editor.offset_all_joints_in_direction('y', self.seg_angles[:,1])
        self.zero_pos = self.editor.get_zero_springref_pos()
        self.editor.total_length = self.segs_flipped[-1][2]

    def save_mujoco_render(self):
        filename = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../images/mujocoRenders/' + time_now_str + '_bush' + str(self.BUSH_NUM) + '_branch'+ str(self.BRANCH_NUM) + '_' + str(len(self.segs_flipped)-1) + 'segs' + '.pdf')
        self.editor.save_picture_of_model(self.zero_pos, filename)

if __name__ == "__main__":
    clear_output()
    np.set_printoptions(precision=3, suppress=True, linewidth=100)

    DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../data/')
    BUSH_DICT = {1:1, 3:2, 5:3, 9:4, 14:5, 23:6}
    USING_CLOUDCOMPARE = True
    USING_SPLINE = False
    OVERRIDE_DATA = False
    RECORDING_DATA = True
    time_now_str = str(datetime.date.today()) + '_' + str(datetime.datetime.now().hour) + '-' + str(datetime.datetime.now().minute)
    results_folder = os.path.join(DATA_DIR, 'results', time_now_str)
    if RECORDING_DATA:
        os.mkdir(results_folder)
        print("Data is being recorded to folder: ", results_folder)
    else:
        print("Data is NOT being recorded. Set RECORDING_DATA to True to save data to a folder.")
    total_sim_idx = 0

    diamdata = pd.read_csv(os.path.join(DATA_DIR, 'diameters/offset_branch_diameter_data.csv'))
    diameter_data_df = pd.DataFrame(diamdata)

    # morris placeholders
    test_mod = 4.9e9
    num_segs = 5
    probe_angle = np.pi/8

    problem = {
        'num_vars': 3,
        'names': ['flex_modulus', 'num_segments', 'probe_angle'],
        'bounds': [[1.68e9, 7.31e9], 
                   [1, 12], 
                   [-np.pi/6, np.pi/6]]}
    # samples = morris_sample(problem, N=500, num_levels=4, optimal_trajectories=2)

    # Have two outputs: 
    # the raw simulation stiffness (how much do the parameters affect the raw output)
    # the MAPE (how much do the parameters affect the error)

    for BUSH_NUM in [1, 3, 5, 9, 14, 23]:
        print ("----------------------------------------")
        print ("Starting bush number: ", BUSH_NUM)
        print ("----------------------------------------")

        for BRANCH_NUM in [1, 2, 3]:
            print ("----------------------------------------")
            print ("Starting bush :", BUSH_NUM, " branch: ", BRANCH_NUM)
            print ("----------------------------------------")
            
            for TRIAL_NUM in [1, 2, 3]:
                # Check to see if it's one of the exceptions we're skipping.. 
                if BUSH_NUM ==14:
                    if BRANCH_NUM == 2 and TRIAL_NUM == 3:
                        print("Excluding trial 14/2/3 due to too many spikes")
                        continue
                    elif BRANCH_NUM == 3 and TRIAL_NUM ==1:
                        print("Excluding trial 14/3/1 because camera data did not capture push point")
                        continue
                
                # samples = morris_sample(problem, N=500, num_levels=4, optimal_trajectories=2)
                # output_stiffnesses = np.zeros(samples.shape[0])
                # output_errors = np.zeros(samples.shape[0])
                # for i, x in enumerate(samples):
                #     test_mod = x[0]
                #     num_segs = int(x[1])
                #     probe_angle = x[2]
                #     print ("----------------------------------------")
                #     print (f"Starting Morris method {i} of {len(samples)} using flex modulus: {test_mod:.2e}, num_segs: {num_segs}, probe_angle: {probe_angle:.3f} rad")
                #     print ("----------------------------------------")
                        
                Branch = BranchSim(BUSH_NUM, BRANCH_NUM, flex_mod=test_mod, num_segs=num_segs)
                Trial = TrialSim(Branch, TRIAL_NUM, force_angle=probe_angle)
                # output_stiffnesses[i] = Trial.sim_fd_linearfit[0]
                # output_errors[i] = Trial.stiffness_error

