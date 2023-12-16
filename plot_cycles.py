import os
import re
import sys
import pickle
import matplotlib.pyplot as plt

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
                    computation_time_ms = cycles / clock_speed
                    timestep_cycles[timestep_index] = computation_time_ms

    return timestep_cycles

def find_min_index_above_threshold(timestep_cycles, threshold):
    min_index = float('inf')
    for index, value in timestep_cycles.items():
        if value > threshold and index < min_index:
            min_index = index
    return min_index if min_index != float('inf') else None

def load_or_create_cumulative_dict(parent_directory):
    grandparent_directory = os.path.abspath(os.path.join(parent_directory, '..'))
    cumulative_file_path = os.path.join(grandparent_directory, 'cumulative_timesteps.pkl')
    if os.path.exists(cumulative_file_path):
        with open(cumulative_file_path, 'rb') as file:
            cumulative_dict = pickle.load(file)
    else:
        cumulative_dict = {}
    return cumulative_dict, cumulative_file_path

def save_cumulative_dict(cumulative_dict, file_path):
    with open(file_path, 'wb') as file:
        pickle.dump(cumulative_dict, file)

def plot_cumulative_dict(cumulative_dict, save_path, nominal_timestep):
    sorted_indices = sorted(cumulative_dict.keys())
    sorted_values = [cumulative_dict[index] for index in sorted_indices]

    # Set a larger figure size
    plt.figure(figsize=(10, 6))

    plt.plot(sorted_indices, sorted_values, marker='o', linestyle='-', color='b')
    
    # Highlight the minimum index where computation time exceeds the threshold
    min_index = find_min_index_above_threshold(cumulative_dict, nominal_timestep)
    if min_index is not None:
        min_value = cumulative_dict[min_index]
        plt.scatter([min_index], [min_value], color='red', label=f'Min Index ({min_index})', zorder=5)
    else:
        min_index = sorted_indices[-1]
    # Plot horizontal line for nominal_timestep
    #plt.axhline(y=nominal_timestep, color='green', linestyle='--', label=f'Nominal Timestep ({nominal_timestep} ms)', zorder=5)

    # Adjust text sizes
    plt.title('Cumulative Computation Time in milliseconds', fontsize=16)
    plt.xlabel('Timestep Index', fontsize=14)
    plt.ylabel('Cumulative Computation Time (ms)', fontsize=14)
    
    # Adjust tick label sizes
    plt.xticks(fontsize=12)
    plt.yticks(fontsize=12)

    # Add legend
    #plt.legend()

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
    nominal_timestep = 0.01  # in milliseconds

    cumulative_dict, cumulative_file_path = load_or_create_cumulative_dict(os.path.dirname(root_directory))
    
    timestep_cycles = traverse_directories(root_directory, line_number, clock_speed, nominal_timestep)

    for index, value in timestep_cycles.items():
        if index in cumulative_dict:
            cumulative_dict[index] = value
        else:
            cumulative_dict[index] = value

    save_cumulative_dict(cumulative_dict, cumulative_file_path)
    min_index = plot_cumulative_dict(cumulative_dict, os.path.join(os.path.dirname(root_directory), 'cumulative_plot.png'), nominal_timestep)

    print(min_index - 1)