import os
import re
import sys
import pickle
import matplotlib.pyplot as plt
import numpy as np

def find_cycles(file_path, line_number):
    with open(file_path, 'r') as file:
        # Read the specified line
        for _ in range(line_number - 1):
            file.readline()
        # Extract the Cycles value
        target_line = file.readline()
        match = re.search(r'Cycles: (\d+)', target_line)
        if match:
            return int(match.group(1))
        else:
            return None

def traverse_directories(root_directory, line_number, clock_speed, nominal_timestep):
    timestep_cycles = {}
    for dirpath, _, filenames in os.walk(root_directory):
        for filename in filenames:
            if filename == 'core.stat.0.out':
                file_path = os.path.join(dirpath, filename)
                cycles = find_cycles(file_path, line_number)
                if cycles is not None:
                    timestep_name = os.path.basename(dirpath)
                    timestep_index = int(timestep_name.split('_')[1])
                    computation_time_s = cycles / clock_speed
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

def plot_cumulative_dict(cumulative_dict, save_path, nominal_timestep):
    # Set a larger figure size
    plt.figure(figsize=(10, 6))
    for i, dictionary in enumerate(cumulative_dict):
        sorted_indices = np.array(sorted(dictionary.keys()))
        sorted_values = np.array([dictionary[index] for index in sorted_indices])
        # only print one above threshold
        if i < len(cumulative_dict)-1:
            min_index = find_min_index_above_threshold(cumulative_dict[i], nominal_timestep)
            filter = sorted_indices < (min_index+1)
            sorted_indices = sorted_indices[filter]
            sorted_values = sorted_values[filter]
        plt.plot(sorted_indices, sorted_values, marker='o', linestyle='-')
    
    # Highlight the minimum index where computation time exceeds the threshold
    last_dict = cumulative_dict[-1]
    min_index = find_min_index_above_threshold(last_dict, nominal_timestep)
    sorted_indices = np.array(sorted(last_dict.keys()))
    if min_index != sorted_indices[-1] + 1:
        min_value = last_dict[min_index]
        plt.scatter([min_index], [min_value], color='red', label=f'Min Index ({min_index})', zorder=5)
    # Plot horizontal line for nominal_timestep
    # plt.axhline(y=nominal_timestep, color='green', linestyle='--', label=f'Nominal Timestep ({nominal_timestep} s)', zorder=5)

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
    if len(sys.argv) != 2:
        print("Usage: python script.py <root_directory>")
        sys.exit(1)

    root_directory = sys.argv[1]
    line_number = 5
    clock_speed = 2e9
    #nominal_timestep = 3.5e-6  # in seconds
    nominal_timestep = 0.1  # in seconds

    cumulative_dict, cumulative_file_path = load_or_create_cumulative_dict(os.path.dirname(root_directory))
    
    timestep_cycles = traverse_directories(root_directory, line_number, clock_speed, nominal_timestep)

    # for index, value in timestep_cycles.items():
    #     if index in cumulative_dict:
    #         cumulative_dict[index] = value
    #     else:
    #         cumulative_dict[index] = value
    cumulative_dict.append(timestep_cycles)

    save_cumulative_dict(cumulative_dict, cumulative_file_path)
    min_index = plot_cumulative_dict(cumulative_dict, os.path.join(os.path.dirname(root_directory), 'cumulative_plot.png'), nominal_timestep)

    print(min_index)