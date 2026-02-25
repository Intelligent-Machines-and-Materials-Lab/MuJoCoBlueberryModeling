# load a metadata file
import pandas as pd
import matplotlib.pyplot as plt
import cmcrameri.cm as cm

def load_metadata(file_path):
    # Read CSV directly with pandas, automatically inferring numeric types
    return pd.read_csv(file_path)

def add_full_labels(metadata_df):
    # add a column that combines bush, branch, and trial into a single string label with 
    metadata_df['full_label'] = metadata_df.apply(lambda row: f"{int(row['bush'])}/{int(row['branch'])}/{int(row['trial'])}", axis=1)
    return metadata_df

def add_bush_idx(metadata_df):
    # Create a mapping of bush types to indices
    bush_types = metadata_df['bush'].unique()
    bush_mapping = {bush: idx for idx, bush in enumerate(bush_types)}
    
    # Add a new column 'bush_idx' to the DataFrame
    metadata_df['bush_idx'] = metadata_df['bush'].map(bush_mapping)
    
    return metadata_df

def plot_slope_data_as_columns(metadata_df):
    # fig, ax = plt.subplots() 
    # truncd_metadata_df = metadata_df.drop(index=8) # this one is just too dang high to plot with the others
    ax = metadata_df.plot.bar(x='full_label', y=['field_stiffness', 'sim_stiffness'], label=["Field Stiffness", "Simulation Stiffness"], figsize=(12, 6), color=['blue', 'orange'])
    ax.grid(axis='y', linestyle='--', color='gray', alpha=0.7)
    ax.set_ylim(0, 1.5)
    ax.set_xlabel('Bush/Branch/Trial')
    ax.set_ylabel('Stiffness (N/mm)')
    plt.show()

def plot_slope_data_as_scatterplot(metadata_df):
    # fig = plt.figure(figsize=(10, 6))
    plt.grid(True, linestyle='--', color='gray', alpha=0.7)
    plt.scatter(metadata_df['field_stiffness'], metadata_df['sim_stiffness'], s=(3*metadata_df['branch'])**3, cmap=cm.batlowKS, c=metadata_df['bush_idx'])
    for i, txt in enumerate(metadata_df['trial']):
        plt.annotate(txt, (metadata_df['field_stiffness'][i], metadata_df['sim_stiffness'][i]), fontsize=10, alpha=0.7)
    plt.plot([0,1], [0,1], color='red', linestyle='--')
    plt.ylim(0, 1.5)
    plt.xlim(0, 1.1)
    plt.xlabel('Field Stiffness (N/mm)')
    plt.ylabel('Simulation Stiffness (N/mm)')
    plt.title('Field vs Simulation Average Stiffness')
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
    

if __name__ == '__main__':
    metadata_df = load_metadata('data/results/metadata_2026-02-23_11-39.csv')
    metadata_df = add_bush_idx(metadata_df)
    metadata_df = add_full_labels(metadata_df)
    print(metadata_df)
    # plot_slope_data_as_scatterplot(metadata_df)
    # plot_error_data_as_boxplots(metadata_df)
    plot_slope_data_as_columns(metadata_df)