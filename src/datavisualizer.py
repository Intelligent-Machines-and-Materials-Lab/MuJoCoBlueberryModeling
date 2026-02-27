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
    metadata_df['full_label'] = metadata_df.apply(lambda row: f"{int(row['bush'])}/{int(row['branch'])}/{row['height_label']}", axis=1)
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
    bush_mapping = {bush: idx for idx, bush in enumerate(bush_types)}
    
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

def plot_slope_data_as_scatterplot(metadata_df):
    # fig = plt.figure(figsize=(10, 6))
    fig, ax = plt.subplots(figsize=(18, 9))
    ax.grid(True, linestyle='--', color='gray', alpha=0.7)
    plt.plot([0,1], [0,1], color='gray', linestyle='--', zorder=0)
    scatter = ax.scatter(metadata_df['field_stiffness'], metadata_df['sim_stiffness'], s=(3*metadata_df['branch'])**3, cmap=cm.batlow, c=metadata_df['bush_idx']) # cmap=cm.batlowKS
    for i, txt in enumerate(metadata_df['height_label']):
        ax.annotate(txt, (metadata_df['field_stiffness'][i]+(.002*metadata_df['branch'][i]**2), metadata_df['sim_stiffness'][i]+(.002*metadata_df['branch'][i]**2)), fontsize=16, alpha=0.7)
    
    mylabels = ["Bush 1", "Bush 3", "Bush 5", "Bush 9", "Bush 14", "Bush 23"]
    handles, labels = scatter.legend_elements(prop="colors", alpha=0.6)
    legend1 = ax.legend(handles, mylabels, loc="upper left", fontsize=16)
    ax.add_artist(legend1)

    handles, labels = scatter.legend_elements(prop="sizes", func=lambda s: (s ** (1/3))/3, alpha=0.6)
    legend2 = ax.legend(handles[::-1], labels[::-1], loc="upper right", title="Branch Num",labelspacing=1.5, fontsize=16, title_fontsize=16)
    ax.tick_params(axis='both', which='major', labelsize=14)
    plt.ylim(0, 1.5)
    plt.xlim(0, 1.05)
    plt.xlabel('Field Stiffness (N/mm)', fontsize=16)
    plt.ylabel('Simulation Stiffness (N/mm)', fontsize=16)
    # plt.title('Field vs Simulation Average Stiffness')
    plt.show()

    # smaller points = smaller branch num = thinner branches
    # smaller number label = smaller trial number = higher on the branch

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
    labels = ['10mm', '15mm', '20mm', '25mm', '30mm']
    print("Data counts after removing NaNs:")
    for i, data in enumerate(error_data):
        print(f"{labels[i]}: {len(data)} values")
    newlabels = [f"{labels[i]} (n={len(data)})" for i, data in enumerate(error_data)]
    ax.set_ylabel('Force Error between Sim and Field (N)')
    ax.set_xlabel('Branch Displacement')
    ax.set_title('Force Error between Simulation and Field at Different Displacements')
    medians = [data.median() for data in error_data]
    bplot = ax.boxplot(error_data, tick_labels=newlabels)
    for i, median in enumerate(medians):
        ax.text(i + 1, median, f'{median:.2f}', ha='center', va='bottom', fontsize=10, color='C0')
    plt.show()

def plot_push_data(bush_num, branch_num, trial_num):
    # Load the corresponding push data file based on the bush, branch, and trial numbers
    file_path = f"data/cropped_push_data/bush_{bush_num}_branch_{branch_num}_trial_{trial_num}.csv"
    push_data_df = pd.read_csv(file_path)
    
    # Plot the push data (e.g., force vs displacement)
    plt.figure(figsize=(5, 3))
    # colors
    # 0.0 for far left column
    # 0.5 for middle left column
    plt.plot(push_data_df['Displacement (mm)'], push_data_df['Load (N)'], color=cm.batlow(0.0), label='Field Data')
    plt.xlabel('Displacement (mm)')
    plt.ylabel('Force (N)')
    # plt.title(f'Push Data for Bush {bush_num}, Branch {branch_num}, Trial {trial_num}')
    plt.legend()
    plt.grid()
    plt.tight_layout()
    plt.show()

    # save png to images/forcedisplacementplots/pushdata_bush_{bush_num}_branch_{branch_num}_trial_{trial_num}.png
    # plt.savefig(f"images/forcedisplacementplots/for_paper/pushdata_bush_{bush_num}_branch_{branch_num}_trial_{trial_num}.png", dpi=300)

def plot_push_data_with_regression_line(bush_num, branch_num, trial_num):
    # Load the corresponding push data file based on the bush, branch, and trial numbers
    file_path = f"data/cropped_push_data/bush_{bush_num}_branch_{branch_num}_trial_{trial_num}.csv"
    push_data_df = pd.read_csv(file_path)

    # Perform linear regression
    fd_linearfit = np.polyfit(push_data_df['Displacement (mm)'], push_data_df['Load (N)'], 1)
    def get_force_at_disp(disp_in_mm):
        force = fd_linearfit[0]*disp_in_mm + fd_linearfit[1]
        return force
    
    # Plot the push data (e.g., force vs displacement)
    plt.figure(figsize=(5, 3))
    plt.plot(push_data_df['Displacement (mm)'], push_data_df['Load (N)'], color=cm.batlow(0.65), label='Field Data')
    plt.plot(push_data_df['Displacement (mm)'], get_force_at_disp(push_data_df['Displacement (mm)']), label='Field Data Linear Fit', linestyle='--', color=cm.batlow(.99))
    plt.xlabel('Displacement (mm)')
    # plt.ylabel('Force (N)')
    # plt.title(f'Push Data for Bush {bush_num}, Branch {branch_num}, Trial {trial_num}')
    plt.legend()
    plt.grid()
    plt.tight_layout()
    # plt.show()

    # save png to images/forcedisplacementplots/pushdata_bush_{bush_num}_branch_{branch_num}_trial_{trial_num}.png
    plt.savefig(f"images/forcedisplacementplots/for_paper/pushdataWregression_bush_{bush_num}_branch_{branch_num}_trial_{trial_num}.png", dpi=300)

if __name__ == '__main__':
    metadata_df = load_metadata('data/results/metadata_2026-02-23_23-10.csv')
    # metadata_df = load_metadata('data/results/metadata_2026-02-23_11-39.csv')
    metadata_df = add_bush_idx(metadata_df)
    metadata_df = add_height_str_labels(metadata_df)
    metadata_df = add_full_labels(metadata_df)
    # plot_slope_data_as_scatterplot(metadata_df)
    # plot_error_data_as_boxplots(metadata_df)
    # plot_slope_data_as_columns(metadata_df)

    # plot_push_data(14, 1, 1)
    # plot_push_data(23, 3, 2)
    # plot_push_data(14, 2, 3)
    # plot_push_data(5, 2, 2)
    # plot_push_data_with_regression_line(23, 1, 1)
    # plot_push_data_with_regression_line(3, 2, 1)
    plot_push_data_with_regression_line(14, 2, 1)
    plot_push_data_with_regression_line(1, 1, 1)
    
