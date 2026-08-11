'''
Hannah's attempt to roll branchSim.ipynb into a script so I 
can wrap it in a function and call it for each of the Morris samples.

Started 7/21/26.
'''

import os
import pickle
import json

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

from SALib.sample import morris
import numpy as np
import pandas as pd
import datetime
import matplotlib.pyplot as plt
from IPython.display import clear_output 
from simple_pid import PID
import cmcrameri.cm as cm
import splineconverter
from caneEditor import CaneEditor

from SALib.sample.morris import sample as morris_sample
from SALib.analyze.morris import analyze as morris_analyze
from SALib.sample.latin import sample as lhc_sample
from SALib.analyze.rsa import analyze as rsa_analyze


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
    def __init__(self, BranchSim, TRIAL_NUM, force_angle=0, error_model='MAPE'):
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

        self.stiffness_error = self.get_stiffness_percentage_error(model=error_model)
        print(f"Stiffness percentage error: {self.stiffness_error*100:.2f}%")

        # self.plot_force_displacement_comparison()

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

        DURATION = 5 #self.TRIAL_LENGTH
        DATACAP_RATE = 10 # Hz
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

        # Capture position at zero_pos for comparison
        zero_pos_probe_xpos = data.site_xpos[probe_site_id].copy()
        print(f"Probe at zero_pos:   x={zero_pos_probe_xpos[0]*1000:.1f} mm, z={zero_pos_probe_xpos[2]*1000:.1f} mm")

        # Let the branch settle briefly before capturing the initial probe position.
        # Gravity only deflects the branch ~1mm from zero_pos, so a short settle suffices.
        SETTLE_TIME = 0.1  # seconds to settle under gravity (no applied force)
        while data.time < SETTLE_TIME:
            mujoco.mj_step(model, data)
        data.time = 0.0  # reset clock so DURATION counts from settled state

        probe_init_xpos = data.site_xpos[probe_site_id].copy()
        print(f"Probe after settle:  x={probe_init_xpos[0]*1000:.1f} mm, z={probe_init_xpos[2]*1000:.1f} mm")
        dx_settle = probe_init_xpos[0] - zero_pos_probe_xpos[0]
        dz_settle = probe_init_xpos[2] - zero_pos_probe_xpos[2]
        print(f"Gravitational shift: dx={dx_settle*1000:.1f} mm, dz={dz_settle*1000:.1f} mm  (Euclidean={np.sqrt(dx_settle**2+dz_settle**2)*1000:.1f} mm)")

        # -- Controller setup --
        pid = PID(Kp=750, Ki=100, Kd=20, setpoint=0, sample_time=None)
        pid.output_limits = (-10, 50)  # force limits in Newtons

        pid.setpoint = 0 # mm, total displacement
        CTRL_POS_UPDATE_RATE = 2 # Hz, how often to update the control position
        last_force = 0.0
        print("Starting simulation...")

        with mujoco.Renderer(model, width=640, height=480) as renderer:
            while data.time < DURATION:
                # -- measure current displacement of probe contact site --
                # Project displacement onto the push direction (signed, so PID can push and resist)
                push_angle = self.Branch.editor.init_probe_angle + self.force_angle
                current_disp_x = data.site_xpos[probe_site_id][0] - probe_init_xpos[0]
                current_disp_z = data.site_xpos[probe_site_id][2] - probe_init_xpos[2]
                current_disp_m = current_disp_x * np.cos(push_angle) + current_disp_z * (-np.sin(push_angle))

                # -- PID: drive current displacement toward target --
                # Pass dt explicitly so PID uses simulation time, not wall-clock time
                force = last_force + pid(current_disp_m, dt=model.opt.timestep)

                # -- apply force at probe contact site --
                data.qfrc_applied[:] = 0
                force_x = force*np.cos(self.Branch.editor.init_probe_angle + self.force_angle)
                force_z = -force*np.sin(self.Branch.editor.init_probe_angle + self.force_angle)
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
                    # print(f"Displacement: {current_disp_m*1000:.1f} mm, setpoint: {pid.setpoint*1000:.1f} mm, error: {pid._last_error*1000:.1f} mm")
                    # print(f"PID force: {force:.3f} N")
                
                # update the control position at the specified rate
                if int(data.time * CTRL_POS_UPDATE_RATE) > int((data.time - model.opt.timestep) * CTRL_POS_UPDATE_RATE):
                    pre_control_timevals.append(data.time)
                    pre_control_forcevals.append(force)
                    pre_control_probeposes.append(current_disp_m)
                    last_force = force  # save the last applied force for the next PID update
                    print(f"Time: {data.time:.3f}s, Current Displacement: {current_disp_m*1000:.1f} mm, Force Applied: {force:.3f} N")       
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

    def get_stiffness_percentage_error(self, model='MAPE'):
        real_stiffness = self.fd_linearfit[0]
        sim_stiffness = self.sim_fd_linearfit[0]
        if model == 'MAPE':
            error = abs(sim_stiffness - real_stiffness) / real_stiffness
        elif model == 'SMAPE':
            error = abs(sim_stiffness - real_stiffness) / ((abs(sim_stiffness) + abs(real_stiffness))/2)
        else:
            raise ValueError("Unknown error model: " + model)
        return error 

class BranchSim():
    def __init__(self, BUSH_NUM, BRANCH_NUM, flex_mod=4.9e9, num_segs=8, diam_func_factor=1):
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
        midpoint_zs = get_midpoints(self.segs_flipped, print_output=False)

        # Get lengths of segments and angles between them (for setting up the MJCF model) 
        self.seg_lengths = splineconverter.get_segment_lengths(self.segs_flipped)
        self.seg_angles = splineconverter.get_angles_between_segments(self.segs_flipped)

        slope, measured_centroid = self.create_linearfit_from_diam_data()

        # Get the radii at the segment midpoints 
        self.radii = self.get_rad_at_height(midpoint_zs, slope, measured_centroid, diam_func_factor=diam_func_factor)
        print("Radii at segment midpoints (m): ", self.radii)

        self.build_mjcf_model(flex_modulus=flex_mod)

        # self.editor.show_model_at_pos_script(self.zero_pos)

    def create_linearfit_from_diam_data(self):
        # get the diameter data for this bush and branch
        self.field_measurements = diameter_data_df.query('Bush == ' + str(self.BUSH_NUM) + ' and Branch == ' + str(self.BRANCH_NUM))
        # print(self.field_measurements)
        # Get the center of the 3 measurements we actually took
        centroid = (self.field_measurements['Height'].mean(), self.field_measurements['Diameter'].mean())
        # Get a nice linear fit for those measurements
        diam_linearfit = np.polyfit(self.field_measurements['Height'], self.field_measurements['Diameter'], 1)
        # This is a linear fit that goes through the centroid but uses the slope from the linear fit
        return diam_linearfit[0], centroid

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
    
    def get_rad_at_height(self, height_in_m, slope, pt, diam_func_factor=1):
        diameter = slope*diam_func_factor*(height_in_m*1000-pt[0]) + pt[1]
        return diameter/2/1000 # convert mm to m
    
    def build_mjcf_model(self, flex_modulus=4.9e9):        # read in the base XML for the branch model (this has the world and the probe, but not the segments of the branch yet)
        xml_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../urdf/branch_base.xml')
        with open(xml_path, 'r') as f:
            branch_xml = f.read()

        self.editor = CaneEditor(branch_xml, flex_mod=flex_modulus)
        self.editor.build_branch_from_lengths(self.seg_lengths, self.radii,
                                               angles_x=self.seg_angles[:,0],
                                               angles_y=self.seg_angles[:,1],
                                               verbose=False)
        self.zero_pos = self.editor.get_zero_springref_pos()
        self.editor.total_length = self.segs_flipped[-1][2]

    def save_mujoco_render(self):
        filename = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../images/mujocoRenders/' + time_now_str + '_bush' + str(self.BUSH_NUM) + '_branch'+ str(self.BRANCH_NUM) + '_' + str(len(self.segs_flipped)-1) + 'segs' + '.pdf')
        self.editor.save_picture_of_model(self.zero_pos, filename)

if __name__ == "__main__":
    plt.close('all')
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
    num_segs = 8
    probe_angle = 0
    diam_func_factor = 1

    morris_problem = {
        'num_vars': 4,
        'names': ['flex_modulus', 'num_segments', 'diam_func_factor', 'probe_angle'],
        'bounds': [[1.82e9, 8.05e9], 
        # 'bounds': [[1.68e9, 7.31e9],
                   [3, 10], 
                   [0, 1],
                   [-np.pi/6, np.pi/6]]}

    Stiffness_Sis = []
    Error_Sis = []
    all_trial_data = []

    for BUSH_NUM in [1, 3, 5, 9, 14, 23]:
    # for BUSH_NUM in [1]:
        print ("----------------------------------------")
        print ("Starting bush number: ", BUSH_NUM)
        print ("----------------------------------------")

        for BRANCH_NUM in [1, 2, 3]:
        # for BRANCH_NUM in [1]:
            print ("----------------------------------------")
            print ("Starting bush :", BUSH_NUM, " branch: ", BRANCH_NUM)
            print ("----------------------------------------")
            
            for TRIAL_NUM in [1, 2, 3]:
            # for TRIAL_NUM in [1]:
                # Check to see if it's one of the exceptions we're skipping.. 
                if BUSH_NUM ==14:
                    if BRANCH_NUM == 2 and TRIAL_NUM == 3:
                        print("Excluding trial 14/2/3 due to too many spikes")
                        continue
                    elif BRANCH_NUM == 3 and TRIAL_NUM ==1:
                        print("Excluding trial 14/3/1 because camera data did not capture push point")
                        continue

                trial_key = f"bush{BUSH_NUM}_branch{BRANCH_NUM}_trial{TRIAL_NUM}"
                
                samples = morris_sample(morris_problem, N=500, num_levels=6, optimal_trajectories=4)
                # lhc_samples = lhc_sample(rsa_problem, N=32)

                output_stiffnesses = np.zeros(samples.shape[0])
                output_errors = np.zeros(samples.shape[0])
                # for i, x in enumerate(lhc_samples):
                for i, x in enumerate(samples):
                    test_mod = x[0]
                    num_segs = int(x[1])
                    diam_func_factor = x[2]
                    probe_angle = x[3]
                    print ("----------------------------------------")
                    print(f"On Bush {BUSH_NUM} Branch {BRANCH_NUM} Trial {TRIAL_NUM}")
                    print (f"Starting Morris method {i} of {len(samples)} using flex modulus: {test_mod:.2e}, num_segs: {num_segs}, probe_angle: {probe_angle:.3f} rad, diam_func_factor: {diam_func_factor:.3f}")
                    # print (f"Starting RSA sample {i} of {len(lhc_samples)} using flex modulus: {test_mod:.2e}, num_segs: {num_segs}, diam_func_factor: {diam_func_factor:.3f}")
                    print ("----------------------------------------")
                        
                    try:
                        Branch = BranchSim(BUSH_NUM, BRANCH_NUM, flex_mod=test_mod, num_segs=num_segs, diam_func_factor=diam_func_factor)
                        Trial = TrialSim(Branch, TRIAL_NUM, force_angle=probe_angle, error_model='SMAPE')
                        output_stiffnesses[i] = Trial.sim_fd_linearfit[0]
                        output_errors[i] = Trial.stiffness_error
                    except Exception as e:
                        print(f"Sample {i} failed: {e}. Will fill with mean after loop.")
                        output_stiffnesses[i] = np.nan
                        output_errors[i] = np.nan   

                # Replace failed samples with the mean of successful ones
                output_stiffnesses = np.where(np.isnan(output_stiffnesses), np.nanmean(output_stiffnesses), output_stiffnesses)
                output_errors = np.where(np.isnan(output_errors), np.nanmean(output_errors), output_errors)

                # Si = rsa_analyze(rsa_problem, lhc_samples, output_errors, bins=5, print_to_console=True)
                # Si.plot()
                Si = morris_analyze(morris_problem, samples, output_stiffnesses, scaled=True, print_to_console=True)
                Stiffness_Sis.append(Si)

                Ei = morris_analyze(morris_problem, samples, output_errors, print_to_console=True)
                Error_Sis.append(Ei)

                all_trial_data.append({
                'problem': morris_problem,
                'samples': samples,
                'output_stiffnesses': output_stiffnesses,
                'output_errors': output_errors,
                'stiffness_Si': Si,
                'stiffness_Si_df': Si.to_df(),
                'error_Si': Ei,
                'error_Si_df': Ei.to_df(),
                'bush': BUSH_NUM,
                'branch': BRANCH_NUM,
                'trial': TRIAL_NUM,
                })
                # pickle Stiffness_Sis and Error_Sis to results folder
                with open(os.path.join(results_folder, 'Stiffness_Sis.pkl'), 'wb') as f:
                    pickle.dump(Stiffness_Sis, f)
                with open(os.path.join(results_folder, 'Error_Sis.pkl'), 'wb') as f:
                    pickle.dump(Error_Sis, f)
                with open(os.path.join(results_folder, 'all_trial_data.pkl'), 'wb') as f:
                    pickle.dump(all_trial_data, f)

    # make a scatterplot with mu star on the x axis and sigma on the y axis for each parameter, for both stiffness and error
    # each parameter should be a different color. The stiffness and error will be on different plots. 
    plt.close('all')
    fig, ax = plt.subplots(1, 2, figsize=(12, 6))
    colors = [cm.batlow(x) for x in np.linspace(0, 1, len(morris_problem['names']))]
    for i, name in enumerate(morris_problem['names']):
        for j in range(len(Stiffness_Sis)):
            ax[0].scatter(Stiffness_Sis[j]['mu_star'][i], Stiffness_Sis[j]['sigma'][i],
                        color=colors[i], label=name if j == 0 else None)
            ax[1].scatter(Error_Sis[j]['mu_star'][i], Error_Sis[j]['sigma'][i],
                        color=colors[i], label=name if j == 0 else None)
    ax[0].set_title('Stiffness Sensitivity')
    ax[0].set_xlabel('Mu Star')
    ax[0].set_ylabel('Sigma')
    ax[1].set_title('Error Sensitivity')
    ax[1].set_xlabel('Mu Star')
    ax[1].set_ylabel('Sigma')
    handles, labels = ax[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='center left', bbox_to_anchor=(0.83, 0.5))
    fig.tight_layout(rect=[0, 0, 0.82, 1])  # leave room on the right for the legend
    plt.show()

    # pickle Stiffness_Sis and Error_Sis to results folder
    with open(os.path.join(results_folder, 'Stiffness_Sis.pkl'), 'wb') as f:
        pickle.dump(Stiffness_Sis, f)
    with open(os.path.join(results_folder, 'Error_Sis.pkl'), 'wb') as f:
        pickle.dump(Error_Sis, f)
    with open(os.path.join(results_folder, 'all_trial_data.pkl'), 'wb') as f:
        pickle.dump(all_trial_data, f)
    # save the plot to the results folder
    fig.savefig(os.path.join(results_folder, 'sensitivity_analysis.png'))

