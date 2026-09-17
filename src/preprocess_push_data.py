#load a file from the data directory, preprocess it, and save it to a new file 
# Heavily modified on 7/9/26 to:
# 1) handle the format of data directly from the bag files 
# 2) crop the data to only include the time range specified in the trial_start_end_times.csv file
#    which is dictated by the IMU changepoint data
import csv
import numpy as np
import pandas as pd
import os

def crop_beg_of_data(data, start_time):

    try:
        # crop the data to only include rows where the Time (s) column is greater than or equal to the start time
        data = data[data['Time (s)'] >= start_time]

        # zero the Displacement (mm) column by subtracting the initial value from all values
        data.loc[:, 'actuator_displacement'] = data['actuator_displacement'] - data['actuator_displacement'].iloc[0]

        # zero the Load columns by subtracting the initial value from all values
        data.loc[:, 'load_cell_reading'] = data['load_cell_reading'] - data['load_cell_reading'].iloc[0]
        data.loc[:, 'Load (N)'] = data['Load (N)'] - data['Load (N)'].iloc[0]

        return data

    except IndexError:
        print("No positive load values found in this file. Skipping this file.")
        # bad_tests.append(input_file)

def convert_to_N(data):
    # create a new column for force called Load (N) and convert from gF to N
    data['Load (N)'] = data['load_cell_reading'] * 0.00980665
    return data

def crop_end_of_data(data, end_time):
    # crop the data to only include rows where the Time (s) column is less than or equal to the end time
    data = data[data['Time (s)'] <= end_time]
    return data

def load_csv_safely(file_path):
    with open(file_path, 'r') as f:
        reader = csv.reader(f)
        rows = list(reader)

    # Skip header, then split timestamp and sensor columns to avoid float32 overflow.
    body = rows[1:]
    timestamps_ns = np.array([int(r[0]) for r in body], dtype=np.int64)
    sensors = np.array([r[1:] for r in body], dtype=np.float32)

    # Convert to elapsed seconds for numerically stable FFT/sampling calculations.
    time_s = (timestamps_ns - timestamps_ns[0]) * 1e-9
    data = pd.DataFrame(sensors[:, -2:], columns=rows[0][-2:])
    data['Time (s)'] = time_s
    return data

def crop_filename(filename):
    # remove everything between the trial number and the .csv extension, including the underscore
    # e.g. bush_1_branch_1_trial_1_height_915_load_cell.csv --> bush_1_branch_1_trial_1.csv
    parts = filename.split('_')
    new_filename = '_'.join(parts[:6]) + '.csv'
    return new_filename


if __name__ == "__main__":

    # Get the start and end times for each trial from the CSV file
    start_and_end_times = pd.read_csv('../data/trial_start_end_times.csv')

    # Get the push files and sort them 
    push_files = [f for f in os.listdir(os.path.join('..', 'data', 'imu_data')) if f.endswith('_load_cell.csv')]
    # push_files = os.listdir('data/push_data')
    sorted_push_files = sorted(push_files, key=lambda x: int(''.join(filter(str.isdigit, x))))

    # iterate through the push files 
    for idx, push_file in enumerate(sorted_push_files):
        data = load_csv_safely(os.path.join('..', 'data', 'imu_data', push_file)) # get the data from the original push data file
        data = convert_to_N(data) # convert the load cell reading from gF to N
        data = crop_end_of_data(data, start_and_end_times.iloc[idx]["end_time"]) # crop the end of the readings first 
        data = crop_beg_of_data(data, start_and_end_times.iloc[idx]["start_time_imu"]) # crop the beginning of the readings second
        push_file_name = crop_filename(push_file) # crop the filename to remove the height and load cell info
        data.to_csv(os.path.join('..', 'data', 'imu_cropped_push_data', push_file_name), index=False)