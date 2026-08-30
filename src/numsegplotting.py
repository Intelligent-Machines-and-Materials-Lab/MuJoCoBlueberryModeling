import pickle
import matplotlib.pyplot as plt
import numpy as np

# import pickled data from data/results/2026-08-28_20-35/metadata.pkl
with open('data/results/2026-08-28_20-35/metadata.pkl', 'rb') as f:
    metadata = pickle.load(f)

segs_list = [4, 6, 8, 10, 12, 14, 16, 18]
plot_len = len(segs_list)
num_plots = len(metadata['sim_stiffness']) // plot_len
all_stiffs = np.reshape(metadata['sim_stiffness'], (num_plots, plot_len))


fig, ax = plt.subplots()
for i in range(num_plots):
    # normalize stiffness data between 0 and 1
    stiffness_data = all_stiffs[i]
    all_stiffs[i] = (stiffness_data - np.min(stiffness_data)) / (np.max(stiffness_data) - np.min(stiffness_data))
    ax.plot(segs_list, stiffness_data, label=f'Trial {i+1}')
# plot the average of all trials
avg_stiffness = np.mean(all_stiffs, axis=0)
ax.plot(segs_list, avg_stiffness, label='Average', color='black', linewidth=2, linestyle='dashed')
ax.set_xlabel("Number of Segments")
ax.set_ylabel("Normalized Simulation Stiffness (N/mm)")
# plt.savefig(os.path.join(results_folder, f"B{BUSH_NUM}B{BRANCH_NUM}T{TRIAL_NUM}"))
plt.show()