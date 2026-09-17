import pickle
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# import pickled data from data/results/2026-08-28_20-35/metadata.pkl
# with open('data/results/2026-09-02_10-52/metadata.pkl', 'rb') as f:
with open('data/results/2026-08-28_20-35_3EIoverL/metadata.pkl', 'rb') as f:
    metadata_3EI = pickle.load(f)
    df_3EI = pd.DataFrame(metadata_3EI)
with open('data/results/2026-09-01_12-22_EIoverL/metadata.pkl', 'rb') as f:
    metadata_EI = pickle.load(f)
    df_EI = pd.DataFrame(metadata_EI)
with open('data/results/2026-09-02_10-52/metadata.pkl', 'rb') as f:
    metadata_default = pickle.load(f)
    df_2EI = pd.DataFrame(metadata_default)

segs_list = [4, 6, 8, 10, 12, 14, 16, 18]
plot_len = len(segs_list)
tip_disp_better_ct = 0
angle_better_ct = 0
moment_disp_better_ct = 0

# for BUSH_NUM in [1, 3, 5, 9, 14, 23]:
#     for BRANCH_NUM in [1, 2, 3]:
#         for TRIAL_NUM in [1, 2, 3]:
#             print(f"Processing B{BUSH_NUM} Branch {BRANCH_NUM} Trial {TRIAL_NUM}")
#             # query the field stiffness from the metadata
#             field_stiffness_EI = df_EI[(df_EI['bush'] == BUSH_NUM) & (df_EI['branch'] == BRANCH_NUM) & (df_EI['trial'] == TRIAL_NUM)]['field_stiffness'].values
#             sim_stiffness_3EI = df_3EI[(df_3EI['bush'] == BUSH_NUM) & (df_3EI['branch'] == BRANCH_NUM) & (df_3EI['trial'] == TRIAL_NUM)]['sim_stiffness'].values
#             sim_stiffness_EI = df_EI[(df_EI['bush'] == BUSH_NUM) & (df_EI['branch'] == BRANCH_NUM) & (df_EI['trial'] == TRIAL_NUM)]['sim_stiffness'].values
#             sim_stiffness_2EI = df_2EI[(df_2EI['bush'] == BUSH_NUM) & (df_2EI['branch'] == BRANCH_NUM) & (df_2EI['trial'] == TRIAL_NUM)]['sim_stiffness'].values
#             error_EI = np.abs(sim_stiffness_EI - field_stiffness_EI) / field_stiffness_EI if len(sim_stiffness_EI) != 0 else None
#             error_3EI = np.abs(sim_stiffness_3EI - field_stiffness_EI) / field_stiffness_EI if len(sim_stiffness_3EI) != 0 else None
#             error_2EI = np.abs(sim_stiffness_2EI - field_stiffness_EI[0]) / field_stiffness_EI[0] if len(sim_stiffness_2EI) != 0 and len(field_stiffness_EI) != 0 else None
#             if len(sim_stiffness_EI) != 0:
#                 print(f"Field stiffness: {field_stiffness_EI[0]:.3f}")
#                 print(f"Sim stiffness EI: {sim_stiffness_EI[0]:.3f} --> {sim_stiffness_EI[-1]:.3f} (error {error_EI[0]:.3f} --> {error_EI[-1]:.3f})")
#                 max_seg_EI_err = error_EI[-1]
#             else:
#                 print(f"Sim stiffness EI not found")
#             if len(sim_stiffness_3EI) != 0:
#                 print(f"Sim stiffness 3EI: {sim_stiffness_3EI[0]:.3f} --> {sim_stiffness_3EI[-1]:.3f} (error {error_3EI[0]:.3f} --> {error_3EI[-1]:.3f})")
#                 max_seg_3EI_err = error_3EI[-1]
#             else:
#                 print(f"Sim stiffness 3EI not found")
#             if len(sim_stiffness_2EI) != 0:
#                 print(f"Sim stiffness 2EI: {sim_stiffness_2EI[0]:.3f} --> {sim_stiffness_2EI[-1]:.3f} (error {error_2EI[0]:.3f} --> {error_2EI[-1]:.3f})")
#                 max_seg_2EI_err = error_2EI[-1]
#             else:
#                 print(f"Sim stiffness 2EI not found")

#             try:
#                 if min(max_seg_3EI_err, max_seg_2EI_err, max_seg_EI_err) == max_seg_3EI_err:
#                     tip_disp_better_ct += 1
#                     print(f"Tip displacement better for this trial")
#                 elif min(max_seg_3EI_err, max_seg_2EI_err, max_seg_EI_err) == max_seg_2EI_err:
#                     moment_disp_better_ct += 1
#                     print(f"Moment displacement better for this trial")
#                 elif min(max_seg_3EI_err, max_seg_2EI_err, max_seg_EI_err) == max_seg_EI_err:
#                     angle_better_ct += 1
#                     print(f"Angle better for this trial")
#             except NameError:
#                 print(f"can't make comparison for this trial")
# print(f"Tip displacement better count: {tip_disp_better_ct}, Angle better count: {angle_better_ct}, Moment better count: {moment_disp_better_ct}")

metadata = metadata_default
num_plots = len(metadata['sim_stiffness']) // plot_len
all_stiffs = np.reshape(metadata['sim_stiffness'][:num_plots*plot_len], (num_plots, plot_len))
all_errors = np.reshape(metadata['error'][:num_plots*plot_len], (num_plots, plot_len))


fig, ax = plt.subplots()
for i in range(num_plots):
    # normalize stiffness data between 0 and 1
    # stiffness_data = all_stiffs[i]
    # all_stiffs[i] = (stiffness_data - np.min(stiffness_data)) / (np.max(stiffness_data) - np.min(stiffness_data))
    ax.plot(segs_list, all_errors[i], label=f'Trial {i+1}')
# plot the average of all trials
avg_stiffness = np.mean(all_errors, axis=0)
ax.plot(segs_list, avg_stiffness, label='Average', color='black', linewidth=2, linestyle='dashed')
ax.set_xlabel("Number of Segments")
# ax.set_ylabel("Normalized Simulation Stiffness (N/mm)")
# plt.savefig(os.path.join(results_folder, f"B{BUSH_NUM}B{BRANCH_NUM}T{TRIAL_NUM}"))
ax.set_title("Errors with 3EI/L")
plt.show()