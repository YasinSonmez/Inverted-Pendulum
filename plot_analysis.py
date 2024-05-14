import os
import re
import sys
import pickle
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

def find_execution_time(file_path, line_number):
    with open(file_path, 'r') as file:
        # Read the specified line
        for _ in range(line_number - 1):
            file.readline()
        # Extract the Cycles value
        target_line = file.readline()
        pattern = r'EXECUTION_TIME\s+(\d+)'
        match = re.search(pattern, target_line)
        if match:
            return int(match.group(1))
        else:
            return None

def traverse_directories(root_directory, line_number):
    timestep_cycles = {}
    for dirpath, _, filenames in os.walk(root_directory):
        for filename in filenames:
            if filename == 'core.stat.0.out':
                file_path = os.path.join(dirpath, filename)
                execution_time = find_execution_time(file_path, line_number)
                if execution_time is not None:
                    timestep_name = os.path.basename(dirpath)
                    timestep_index = int(timestep_name.split('_')[1])
                    computation_time_s = execution_time/1e15 # fs to s
                    timestep_cycles[timestep_index] = computation_time_s
    return timestep_cycles

def find_min_index_above_threshold(timestep_cycles, threshold):
    min_index = float('inf')
    for index, value in timestep_cycles.items():
        if value > threshold and index < min_index:
            min_index = index
    sorted_indices = np.array(sorted(timestep_cycles.keys()))
    return min_index if min_index != float('inf') else sorted_indices[-1] + 1

def load_or_create_cumulative_dict(parent_directory):
    grandparent_directory = os.path.abspath(os.path.join(parent_directory, '..'))
    cumulative_file_path = os.path.join(grandparent_directory, 'cumulative_timesteps.pkl')

    if os.path.exists(cumulative_file_path):
        with open(cumulative_file_path, 'rb') as file:
            cumulative_dict = pickle.load(file)
    else:
        cumulative_dict = []
    return cumulative_dict, cumulative_file_path

def save_cumulative_dict(cumulative_dict, file_path):
    with open(file_path, 'wb') as file:
        pickle.dump(cumulative_dict, file)

# Function to read CSV and plot
def plot_csv_data(filename, save_path, save_path_2, control_sampling_time):
    # Read the CSV file into a DataFrame
    df = pd.read_csv(filename, header=None)
    
    # Extract the timestep (first column) and other columns
    timestep = df[0]
    variables = df.iloc[:, 1:]
    
    # Number of variables (subtract 1 for the timestep column)
    num_vars = variables.shape[1]
    
    # Create subplots
    fig, axes = plt.subplots(num_vars, 1, figsize=(10, 2 * num_vars))
    
    # Plot each variable in a separate subplot
    for i in range(num_vars):
        ax = axes[i] if num_vars > 1 else axes
        ax.plot(timestep*control_sampling_time, variables.iloc[:, i])
        ax.scatter(timestep*control_sampling_time, variables.iloc[:, i], label=f'Variable {i+1}')
        ax.set_title(f'Variable {i+1}')
        ax.set_xlabel('Time(s)')
        ax.set_ylabel('Value')
        ax.grid()
    
    plt.tight_layout()
    # Save the figure as a PNG file
    plt.savefig(save_path)
    plt.close()

    # Plot variables against each other with timestep as color
    num_control_vars = 1
    num_state_vars = num_vars - num_control_vars

    fig, axes = plt.subplots(num_state_vars-1, num_state_vars-1, figsize=((num_state_vars-1) * 5, (num_state_vars-1) * 5))

    for i in range(num_state_vars-1):
        for j in range(i+1, num_state_vars):
            ax = axes[i, j-1]
            sc = ax.scatter(variables.iloc[:, i], variables.iloc[:, j], c=timestep, cmap='viridis')
            ax.set_title(f'Variable {i+1} vs Variable {j+1}')
            ax.set_xlabel(f'Variable {i+1}')
            ax.set_ylabel(f'Variable {j+1}')
            fig.colorbar(sc, ax=ax, label='Timestep')
            ax.grid()

    plt.tight_layout()
    plt.savefig(save_path_2)
    plt.close()

def plot_cumulative_dict(cumulative_dict, save_path, control_sampling_time):
    # Set a larger figure size
    plt.figure(figsize=(10, 6))
    for i, dictionary in enumerate(cumulative_dict):
        sorted_indices = np.array(sorted(dictionary.keys()))
        sorted_values = np.array([dictionary[index] for index in sorted_indices])
        # only print one above threshold
        if i < len(cumulative_dict)-1:
            min_index = find_min_index_above_threshold(cumulative_dict[i], control_sampling_time)
            filter = sorted_indices < (min_index+1)
            sorted_indices = sorted_indices[filter]
            sorted_values = sorted_values[filter]
        plt.plot(sorted_indices*control_sampling_time, sorted_values, marker='o', linestyle='-')
    
    # Highlight the minimum index where computation time exceeds the threshold
    last_dict = cumulative_dict[-1]
    min_index = find_min_index_above_threshold(last_dict, control_sampling_time)
    sorted_indices = np.array(sorted(last_dict.keys()))
    if min_index != sorted_indices[-1] + 1:
        min_value = last_dict[min_index]
        plt.scatter([min_index], [min_value], color='red', label=f'Min Index ({min_index})', zorder=5)
    # Plot horizontal line for control_sampling_time
    # plt.axhline(y=control_sampling_time, color='green', linestyle='--', label=f'Sampling Time ({control_sampling_time} s)', zorder=5)

    # Adjust text sizes
    plt.title('Computation Time', fontsize=16)
    plt.xlabel('Timestep', fontsize=14)
    plt.ylabel('Computation Time (s)', fontsize=14)
    
    # Adjust tick label sizes
    plt.xticks(fontsize=12)
    plt.yticks(fontsize=12)

    # Add legend
    plt.legend()

    plt.savefig(save_path)
    #plt.show()
    return min_index

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python script.py <root_directory> <control_sampling_time>")
        sys.exit(1)

    root_directory = sys.argv[1]
    control_sampling_time = float(sys.argv[2])
    line_number = 7

    cumulative_dict, cumulative_file_path = load_or_create_cumulative_dict(os.path.dirname(root_directory))
    
    timestep_cycles = traverse_directories(root_directory, line_number)

    # for index, value in timestep_cycles.items():
    #     if index in cumulative_dict:
    #         cumulative_dict[index] = value
    #     else:
    #         cumulative_dict[index] = value
    cumulative_dict.append(timestep_cycles)

    save_cumulative_dict(cumulative_dict, cumulative_file_path)
    min_index = plot_cumulative_dict(cumulative_dict, os.path.join(os.path.dirname(root_directory), 'cumulative_plot.png'), control_sampling_time)
    
    grandparent_directory = os.path.abspath(os.path.join(os.path.dirname(root_directory), '..'))
    state_and_input_matrix_file_path = os.path.join(grandparent_directory, 'state_and_input_matrix.csv')
    plot_csv_data(state_and_input_matrix_file_path, os.path.join(os.path.dirname(root_directory), 'cumulative_state_and_input_plot.png'), os.path.join(os.path.dirname(root_directory), 'cumulative_pairwise_plot.png'), control_sampling_time)
    print(min_index)