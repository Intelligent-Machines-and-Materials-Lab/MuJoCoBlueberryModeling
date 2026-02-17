#load a file from the data directory, preprocess it, and save it to a new file 
import pandas as pd
import os

def preprocess_push_data(input_file, output_file):
    # Read the input CSV file
    data = pd.read_csv(input_file)

    # find the first load value that is positive and get the index of that row
    first_positive_load_index = data[data['Load Cell Reading (gf)'] > 0].index[0]
    print("First positive load index: ", first_positive_load_index)

    # drop all rows before the first positive load value
    data = data.iloc[first_positive_load_index:]
    print("Data after dropping rows before first positive load value: ", data.head())

    # create a new column for force called Load (N) and convert from gF to N
    data['Load (N)'] = data['Load Cell Reading (gf)'] * 0.00980665

    # zero the Displacement (mm) column by subtracting the initial value from all values
    data['Displacement (mm)'] = data['Displacement (mm)'] - data['Displacement (mm)'].iloc[0]    

    # Save the preprocessed data to a new CSV file
    data.to_csv(output_file, index=False)


if __name__ == "__main__":
    # iterate through all files in data directory
    print(os.listdir())
    # preprocess_push_data('data/push_data/bush_1_branch_1_trial_2.csv')
    for filename in os.listdir('data/push_data'):
        print(filename)
        input_file = os.path.join('data/push_data', filename)
        output_file = os.path.join('data/cropped_push_data', filename)

        preprocess_push_data(input_file, output_file)