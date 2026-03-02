# load a metadata file
import pandas as pd
import matplotlib.pyplot as plt
import cmcrameri.cm as cm
import numpy as np

def load_metadata(file_path):
    # Read CSV directly with pandas, automatically inferring numeric types
    return pd.read_csv(file_path)

def add_full_labels(metadata_df):
    # add a column that combines bush, branch, and trial into a single string label with 
    metadata_df['full_label'] = metadata_df.apply(lambda row: f"{int(row['bush_idx'])}/{int(row['branch'])}/{row['height_label']}", axis=1)
    return metadata_df

def add_height_str_labels(metadata_df):
    # add a column that designates whether the trial was "high", "middle", or "low" based on the trial number
    def height_label(trial):
        if trial == 3:
            return "L"
        elif trial == 2:
            return "M"
        else:
            return "H"
    metadata_df['height_label'] = metadata_df['trial'].apply(height_label)
    return metadata_df

def add_bush_idx(metadata_df):
    # Create a mapping of bush types to indices
    bush_types = metadata_df['bush'].unique()
    bush_mapping = {bush: idx+1 for idx, bush in enumerate(bush_types)}
    
    # Add a new column 'bush_idx' to the DataFrame
    metadata_df['bush_idx'] = metadata_df['bush'].map(bush_mapping)
    
    return metadata_df

def plot_slope_data_as_columns(metadata_df):
    fig, ax = plt.subplots(figsize=(18, 9))
    # ax.grid(axis='y', linestyle='--', color='gray', alpha=0.7)
    metadata_df.plot.bar(x='full_label', y=['field_stiffness', 'sim_stiffness'], cmap=cm.batlow, label=["Field Stiffness", "Simulation Stiffness"], ax=ax)
    ax.tick_params(axis='both', which='major', labelsize=14)
    # Add grid after plotting with proper zorder
    ax.grid(axis='y', linestyle='--', color='gray', alpha=0.7, zorder=0)
    ax.set_axisbelow(True)  # Ensure grid is behind the bars
    ax.legend(fontsize=16)  # Set legend font size
    ax.set_ylim(0, 1.5)
    ax.set_xlabel('Bush/Branch/Trial', fontsize=16)
    ax.set_ylabel('Stiffness (N/mm)', fontsize=16)
    plt.show()

def plot_slope_data_as_columns_with_splines(metadata_df, spline_df):
    # add new column to metadata_df called "spline_sim_stiffness" that contains spline_df['sim_stiffness'] that matches 'Bush', 'branch', and 'trial' from metadata_df
    metadata_df = metadata_df.merge(spline_df[['bush', 'branch', 'trial', 'sim_stiffness']], on=['bush', 'branch', 'trial'], how='left', suffixes=('', '_spline'))
    fig, ax = plt.subplots(figsize=(18, 9))
    # ax.grid(axis='y', linestyle='--', color='gray', alpha=0.7)
    metadata_df.plot.bar(x='full_label', y=['field_stiffness', 'sim_stiffness', 'sim_stiffness_spline'], label=["Field Stiffness", "Polyline Simulation Stiffness", "Spline Simulation Stiffness"], ax=ax, color=[cm.batlow(.1), cm.batlow(.85), cm.batlow(.5)], width=0.8)
    ax.tick_params(axis='both', which='major', labelsize=14)
    # # Add grid after plotting with proper zorder
    ax.grid(axis='y', linestyle='--', color='gray', alpha=0.7, zorder=0)
    ax.set_axisbelow(True)  # Ensure grid is behind the bars
    ax.legend(fontsize=16)  # Set legend font size
    ax.set_ylim(0, 1.7)
    ax.set_xlabel('Bush/Branch/Trial', fontsize=16)
    ax.set_ylabel('Stiffness (N/mm)', fontsize=16)
    plt.show()

def plot_slope_data_as_columns_with_both_splines(metadata_df, spline_df_og_radii, spline_df_new_radii):
    # add new column to metadata_df called "spline_sim_stiffness" that contains spline_df['sim_stiffness'] that matches 'Bush', 'branch', and 'trial' from metadata_df
    metadata_df = metadata_df.merge(spline_df_og_radii[['bush', 'branch', 'trial', 'sim_stiffness']], on=['bush', 'branch', 'trial'], how='left', suffixes=('', '_spline_og_radii'))
    metadata_df = metadata_df.merge(spline_df_new_radii[['bush', 'branch', 'trial', 'sim_stiffness']], on=['bush', 'branch', 'trial'], how='left', suffixes=('', '_spline_new_radii'))
    fig, ax = plt.subplots(figsize=(12, 6))
    # ax.grid(axis='y', linestyle='--', color='gray', alpha=0.7)
    # only plot where bush == 3
    metadata_df = metadata_df[metadata_df['bush'] == 3]
    metadata_df.plot.bar(x='full_label', y=['field_stiffness', 'sim_stiffness', 'sim_stiffness_spline_og_radii', 'sim_stiffness_spline_new_radii'], label=["Field Stiffness", "Polyline Simulation Stiffness", "Spline Simulation Stiffness (Measured Diam.)", "Spline Simulation Stiffness (Depth Diam.)"], ax=ax, color=[cm.batlow(0), cm.batlow(.99), cm.batlow(.5), cm.batlow(.3)], width=0.8)
    ax.tick_params(axis='both', which='major', labelsize=14)
    # # Add grid after plotting with proper zorder
    ax.grid(axis='y', linestyle='--', color='gray', alpha=0.7, zorder=0)
    ax.set_axisbelow(True)  # Ensure grid is behind the bars
    ax.legend(fontsize=16)  # Set legend font size
    ax.set_ylim(0, 1.7)
    ax.set_xlabel('Bush/Branch/Trial', fontsize=16)
    ax.set_ylabel('Stiffness (N/mm)', fontsize=16)
    plt.tight_layout()
    plt.show()


def plot_slope_data_as_scatterplot(metadata_df):
    # fig = plt.figure(figsize=(10, 6))
    fig, ax = plt.subplots(figsize=(18, 9))
    ax.grid(True, linestyle='--', color='gray', alpha=0.7, zorder=-1)
    ax.set_axisbelow(True)  # Ensure grid is behind the points
    plt.plot([0,1], [0,1], color='gray', linestyle='--', zorder=0)
    branch1df = metadata_df[metadata_df['branch'] == 1]
    branch2df = metadata_df[metadata_df['branch'] == 2]
    branch3df = metadata_df[metadata_df['branch'] == 3]

    # Set consistent color range for all scatter plots
    vmin = metadata_df['bush_idx'].min()
    vmax = metadata_df['bush_idx'].max()

    scatter = ax.scatter(branch1df['field_stiffness'], branch1df['sim_stiffness'], s=(10*branch1df['trial'])**2, cmap=cm.batlow, c=branch1df['bush_idx'], vmin=vmin, vmax=vmax, label='Branch 1', marker='o') # cmap=cm.batlowKS
    scatter2 = ax.scatter(branch2df['field_stiffness'], branch2df['sim_stiffness'], s=(10*branch2df['trial'])**2, cmap=cm.batlow, c=branch2df['bush_idx'], vmin=vmin, vmax=vmax, label='Branch 2', marker='s') # cmap=cm.batlowKS
    scatter3 = ax.scatter(branch3df['field_stiffness'], branch3df['sim_stiffness'], s=(10*branch3df['trial'])**2, cmap=cm.batlow, c=branch3df['bush_idx'], vmin=vmin, vmax=vmax, label='Branch 3', marker='^') # cmap=cm.batlowKS
    # scatter = ax.scatter(metadata_df['field_stiffness'], metadata_df['sim_stiffness'], s=(3*metadata_df['trial'])**3, cmap=cm.batlow, c=metadata_df['bush_idx']) # cmap=cm.batlowKS
    # for i, txt in enumerate(metadata_df['branch']):
    #     ax.annotate(txt, (metadata_df['field_stiffness'][i]+(.002*metadata_df['branch'][i]**2), metadata_df['sim_stiffness'][i]+(.002*metadata_df['branch'][i]**2)), fontsize=16, alpha=0.7)
    
    bushnumlabels = ["1", "2", "3", "4", "5", "6"]
    # Create custom legend handles for all 6 bush indices
    from matplotlib.patches import Patch
    legend_handles = [Patch(facecolor=cm.batlow((i-vmin)/(vmax-vmin)), alpha=1) for i in range(int(vmin), int(vmax)+1)]
    legend1 = ax.legend(legend_handles, bushnumlabels, loc="upper left", fontsize=16, title="Bush Num", title_fontsize=16)
    ax.add_artist(legend1)

    branchnumlabels = ["1", "2", "3"]
    from matplotlib.lines import Line2D
    branch_legend_handles = [Line2D([0], [0], marker='o', color='w', label=branchnumlabels[0], markerfacecolor='gray', markersize=10, alpha=0.6),
                             Line2D([0], [0], marker='s', color='w', label=branchnumlabels[1], markerfacecolor='gray', markersize=10, alpha=0.6),
                             Line2D([0], [0], marker='^', color='w', label=branchnumlabels[2], markerfacecolor='gray', markersize=10, alpha=0.6)]
    branch_legend = ax.legend(handles=branch_legend_handles, loc="upper left", bbox_to_anchor=(.1, 1), fontsize=16, title="Branch Num", title_fontsize=16)
    ax.add_artist(branch_legend)

    trialloclabels = ["Low", "Middle", "High"]
    handles, labels = scatter.legend_elements(prop="sizes", alpha=0.6)
    legend2 = ax.legend(handles[::-1], trialloclabels, loc="upper left", bbox_to_anchor=(.215, 1), title="Trial Height",labelspacing=1.5, fontsize=16, title_fontsize=16)
    ax.tick_params(axis='both', which='major', labelsize=14)
    plt.ylim(0, 1.5)
    plt.xlim(0, 1.02)
    plt.xlabel('Field Stiffness (N/mm)', fontsize=16)
    plt.ylabel('Simulation Stiffness (N/mm)', fontsize=16)
    # plt.title('Field vs Simulation Average Stiffness')
    plt.show()

    # smaller points = smaller branch num = thinner branches
    # smaller number label = smaller trial number = higher on the branch

def plot_slope_data_as_scatterplot_with_splines(metadata_df, spline_df):
    # fig = plt.figure(figsize=(10, 6))
    fig, ax = plt.subplots(figsize=(18, 9))
    ax.grid(True, linestyle='--', color='gray', alpha=0.7, zorder=-1)
    ax.set_axisbelow(True)  # Ensure grid is behind the points
    plt.plot([0,1], [0,1], color='gray', linestyle='--', zorder=0)
    branch1df = metadata_df[metadata_df['branch'] == 1]
    branch2df = metadata_df[metadata_df['branch'] == 2]
    branch3df = metadata_df[metadata_df['branch'] == 3]

    spline1df = spline_df[spline_df['branch'] == 1]
    spline2df = spline_df[spline_df['branch'] == 2]
    spline3df = spline_df[spline_df['branch'] == 3]

    # Set consistent color range for all scatter plots
    vmin = metadata_df['bush_idx'].min()
    vmax = metadata_df['bush_idx'].max()

    scatter = ax.scatter(branch1df['field_stiffness'], branch1df['sim_stiffness'], s=(10*branch1df['trial'])**2, cmap=cm.batlow, c=branch1df['bush_idx'], vmin=vmin, vmax=vmax, label='Branch 1', marker='o') # cmap=cm.batlowKS
    scatter2 = ax.scatter(branch2df['field_stiffness'], branch2df['sim_stiffness'], s=(10*branch2df['trial'])**2, cmap=cm.batlow, c=branch2df['bush_idx'], vmin=vmin, vmax=vmax, label='Branch 2', marker='s') # cmap=cm.batlowKS
    scatter3 = ax.scatter(branch3df['field_stiffness'], branch3df['sim_stiffness'], s=(10*branch3df['trial'])**2, cmap=cm.batlow, c=branch3df['bush_idx'], vmin=vmin, vmax=vmax, label='Branch 3', marker='^') # cmap=cm.batlowKS

    scatter4 = ax.scatter(spline1df['field_stiffness'], spline1df['sim_stiffness'], s=(10*spline1df['trial'])**2, color='gray', marker = 'o')
    scatter5 = ax.scatter(spline2df['field_stiffness'], spline2df['sim_stiffness'], s=(10*spline2df['trial'])**2, color='gray', marker = 's')
    scatter6 = ax.scatter(spline3df['field_stiffness'], spline3df['sim_stiffness'], s=(10*spline3df['trial'])**2, color='gray', marker = '^')
    # scatter = ax.scatter(metadata_df['field_stiffness'], metadata_df['sim_stiffness'], s=(3*metadata_df['trial'])**3, cmap=cm.batlow, c=metadata_df['bush_idx']) # cmap=cm.batlowKS
    # for i, txt in enumerate(metadata_df['branch']):
    #     ax.annotate(txt, (metadata_df['field_stiffness'][i]+(.002*metadata_df['branch'][i]**2), metadata_df['sim_stiffness'][i]+(.002*metadata_df['branch'][i]**2)), fontsize=16, alpha=0.7)
    
    bushnumlabels = ["1", "2", "3", "4", "5", "6", "2 (spline)"]
    # Create custom legend handles for all 6 bush indices
    from matplotlib.patches import Patch
    legend_handles = [Patch(facecolor=cm.batlow((i-vmin)/(vmax-vmin)), alpha=1) for i in range(int(vmin), int(vmax)+1)] + [Patch(facecolor='gray', alpha=1)]
    legend1 = ax.legend(legend_handles, bushnumlabels, loc="upper left", fontsize=16, title="Bush Num", title_fontsize=16)
    ax.add_artist(legend1)

    splinetextoffset = .035
    branchnumlabels = ["1", "2", "3"]
    from matplotlib.lines import Line2D
    branch_legend_handles = [Line2D([0], [0], marker='o', color='w', label=branchnumlabels[0], markerfacecolor='gray', markersize=10, alpha=0.6),
                             Line2D([0], [0], marker='s', color='w', label=branchnumlabels[1], markerfacecolor='gray', markersize=10, alpha=0.6),
                             Line2D([0], [0], marker='^', color='w', label=branchnumlabels[2], markerfacecolor='gray', markersize=10, alpha=0.6)]
    branch_legend = ax.legend(handles=branch_legend_handles, loc="upper left", bbox_to_anchor=(.1+splinetextoffset, 1), fontsize=16, title="Branch Num", title_fontsize=16)
    ax.add_artist(branch_legend)

    trialloclabels = ["Low", "Middle", "High"]
    handles, labels = scatter.legend_elements(prop="sizes", alpha=0.6)
    legend2 = ax.legend(handles[::-1], trialloclabels, loc="upper left", bbox_to_anchor=(.215+splinetextoffset, 1), title="Trial Height",labelspacing=1.5, fontsize=16, title_fontsize=16)
    ax.tick_params(axis='both', which='major', labelsize=14)
    plt.ylim(0, 1.5)
    plt.xlim(0, 1.02)
    plt.xlabel('Field Stiffness (N/mm)', fontsize=16)
    plt.ylabel('Simulation Stiffness (N/mm)', fontsize=16)
    # plt.title('Field vs Simulation Average Stiffness')
    plt.show()

def plot_error_data_as_boxplots(metadata_df):
    fig, ax = plt.subplots()
    # Filter out NaN values from each column before plotting
    error_data = [
        metadata_df['force_error_10mm'].dropna(), 
        metadata_df['force_error_15mm'].dropna(), 
        metadata_df['force_error_20mm'].dropna(), 
        metadata_df['force_error_25mm'].dropna(), 
        metadata_df['force_error_30mm'].dropna()
    ]
    labels = ['10', '15', '20', '25', '30']
    print("Data counts after removing NaNs:")
    for i, data in enumerate(error_data):
        print(f"{labels[i]}: {len(data)} values")
    newlabels = [f"{labels[i]} (n={len(data)})" for i, data in enumerate(error_data)]
    ax.set_ylabel('Force Error between Sim and Field (N)')
    ax.set_xlabel('Branch Displacement (mm)')
    # ax.set_title('Force Error between Simulation and Field at Different Displacements')
    medians = [data.median() for data in error_data]
    bplot = ax.boxplot(error_data, tick_labels=newlabels, medianprops=dict(color=cm.batlow(.65)))
    for i, median in enumerate(medians):
        ax.text(i + 1, median, f'{median:.2f}', ha='center', va='bottom', fontsize=10, color=cm.batlow(.3))
    plt.show()

def plot_push_data(bush_num, branch_num, trial_num, make_legend=True, ax=None):
    if ax is None:
        fig, ax = plt.subplots()
    # Load the corresponding push data file based on the bush, branch, and trial numbers
    file_path = f"data/cropped_push_data/bush_{bush_num}_branch_{branch_num}_trial_{trial_num}.csv"
    push_data_df = pd.read_csv(file_path)
    
    # plt.figure(figsize=(5, 3))
    # colors
    # 0.0 for far left column
    # 0.5 for middle left column
    ax.plot(push_data_df['Displacement (mm)'], push_data_df['Load (N)'], color=cm.batlow(0.0), label='Field Data')
    # ax.set_xlabel('Displacement (mm)')
    # ax.set_ylabel('Force (N)')
    # ax.set_title(f'Push Data for Bush {bush_num}, Branch {branch_num}, Trial {trial_num}')
    if make_legend:
        ax.legend(fontsize=12)
    ax.grid()
    ax.tick_params(axis='both', which='major', labelsize=12)

    return ax
    # plt.tight_layout()
    # plt.show()

    # save png to images/forcedisplacementplots/pushdata_bush_{bush_num}_branch_{branch_num}_trial_{trial_num}.png
    # plt.savefig(f"images/forcedisplacementplots/for_paper/pushdata_bush_{bush_num}_branch_{branch_num}_trial_{trial_num}.png", dpi=300)

def plot_push_data_with_regression_line(bush_num, branch_num, trial_num, make_legend=True, ax=None):
    if ax is None:
        fig, ax = plt.subplots()
    # Load the corresponding push data file based on the bush, branch, and trial numbers
    file_path = f"data/cropped_push_data/bush_{bush_num}_branch_{branch_num}_trial_{trial_num}.csv"
    push_data_df = pd.read_csv(file_path)

    # Perform linear regression
    fd_linearfit = np.polyfit(push_data_df['Displacement (mm)'], push_data_df['Load (N)'], 1)
    def get_force_at_disp(disp_in_mm):
        force = fd_linearfit[0]*disp_in_mm + fd_linearfit[1]
        return force
    
    # Plot the push data (e.g., force vs displacement)
    ax.plot(push_data_df['Displacement (mm)'], push_data_df['Load (N)'], color=cm.batlow(0.0), label='Field Data')
    ax.plot(push_data_df['Displacement (mm)'], get_force_at_disp(push_data_df['Displacement (mm)']), label='Field Data Linear Fit', linestyle='--', color=cm.batlow(.7))
    # ax.set_xlabel('Displacement (mm)')
    # ax.set_ylabel('Force (N)')
    # ax.set_title(f'Push Data for Bush {bush_num}, Branch {branch_num}, Trial {trial_num}')
    if make_legend:
        ax.legend(fontsize=12)
    ax.grid()
    ax.tick_params(axis='both', which='major', labelsize=12)
    # plt.show()
    return ax
    # save png to images/forcedisplacementplots/pushdata_bush_{bush_num}_branch_{branch_num}_trial_{trial_num}.png
    # plt.savefig(f"images/forcedisplacementplots/for_paper/pushdataWregression_bush_{bush_num}_branch_{branch_num}_trial_{trial_num}.png", dpi=300)

if __name__ == '__main__':
    metadata_df = load_metadata('data/results/metadata_2026-02-23_23-10.csv')
    spline_df_new = load_metadata('data/results/metadata_2026-02-28_15-54.csv')
    spline_df_og = load_metadata('data/results/metadata_2026-02-27_21-20.csv')
    # metadata_df = load_metadata('data/results/metadata_2026-02-23_11-39.csv')
    metadata_df = add_bush_idx(metadata_df)
    metadata_df = add_height_str_labels(metadata_df)
    metadata_df = add_full_labels(metadata_df)
    # plot_slope_data_as_scatterplot(metadata_df)
    # plot_error_data_as_boxplots(metadata_df)
    # plot_slope_data_as_columns(metadata_df)
    # plot_slope_data_as_columns_with_splines(metadata_df, spline_df)
    # plot_slope_data_as_scatterplot_with_splines(metadata_df, spline_df)

    plot_slope_data_as_columns_with_both_splines(metadata_df, spline_df_og, spline_df_new)

    # fig, axs = plt.subplots(2, 3, figsize=(11, 6))
    # # top left
    # plot_push_data(14, 2, 3, ax=axs[0, 0])
    # axs[0, 0].axvspan(36, 42.5, facecolor='r', alpha=0.3)
    # # bottom left
    # plot_push_data(5, 2, 2, make_legend=False,ax=axs[1, 0])
    # axs[1, 0].axvspan(14, 39, facecolor='r', alpha=0.3)
    # # top middle
    # plot_push_data(14, 1, 1, make_legend=False, ax=axs[0, 1])
    # axs[0, 1].axvspan(0, 18.5, facecolor='r', alpha=0.3)
    # # bottom middle
    # plot_push_data(23, 3, 2, make_legend=False, ax=axs[1, 1])
    # axs[1, 1].axvspan(0, 17, facecolor='r', alpha=0.3)
    # # top right
    # plot_push_data_with_regression_line(23, 1, 1, ax=axs[0, 2])
    # # bottom right
    # plot_push_data_with_regression_line(3, 2, 1, make_legend=False, ax=axs[1, 2])
    # fig.supxlabel('Displacement (mm)', fontsize=16)
    # fig.supylabel('Force (N)', fontsize=16)
    # plt.tight_layout()
    # plt.show()
    # plot_push_data_with_regression_line(14, 2, 1)
    # plot_push_data_with_regression_line(1, 1, 1)
    
