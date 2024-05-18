#! /usr/bin/bash

# Set the Scarab folder path
scarab_path=$RESOURCES_DIR/scarab

# Get the current directory of the script
main_path=$(cd "$(dirname "$0")" && pwd)
echo "Current path: $main_path"
echo ""

# Get the script arguments
start_idx=$1
time_horizon=$2
chip_params_path="$main_path/$3"
controller_params_path="$main_path/$4"
exp_name=$5
PARAM_INDEX=$6
simulation_executable=$7

# Calculate the total timesteps needed to reach the time_horizon
# Read and parse the JSON file to get the control_sampling_time
json=$(cat $controller_params_path)
control_sampling_time=$(echo "$json" | grep -o '"control_sampling_time": *[0-9.]*' | awk -F': *' '{print $2}')
total_timesteps=$(echo "($time_horizon / $control_sampling_time + 0.99999999999)/1" | bc)

echo "control_sampling_time: $control_sampling_time"
echo "total_timesteps: $total_timesteps"
echo ""

# Initialize variables
current_timestep=1
num_proc=$(nproc)
num_proc=3
current_time=$(date +'%Y-%m-%d_%H-%M-%S')
exp_path="$main_path/Scarab-Trace-and-Simulate-Script/results/${exp_name}_time_horizon${time_horizon}_control_sampling_time${control_sampling_time}_num_proc${num_proc}/${PARAM_INDEX}_$current_time"
mkdir -p "$exp_path"
restart_flag=false

# Main loop to run the simulation and trace scripts
while [ $current_timestep -lt $total_timesteps ]; do
    cd "$exp_path"
    echo "Current path: $(pwd)"
    echo "Current timestep: $current_timestep"
    echo "restart_flag: $restart_flag"
    echo ""

    # Run trace.sh with the restart_flag as an argument
    $main_path/Scarab-Trace-and-Simulate-Script/trace.sh $scarab_path $main_path/$7 $current_timestep $num_proc $restart_flag $chip_params_path $controller_params_path $control_sampling_time

    # Determine the starting simulation position based on the restart flag and current timestep
    if [ "$restart_flag" = false ] && [ "$current_timestep" -eq 1 ]; then
        start_simulation_from=0
    else
        start_simulation_from="$current_timestep"
    fi
    
    timesteps_path="$exp_path/Timesteps_${start_simulation_from}-$(($current_timestep + num_proc - 1))"

    # Run simulate.sh to simulate using scarab
    eval "$main_path/Scarab-Trace-and-Simulate-Script/simulate.sh" ${timesteps_path}

    # Get the new timestep from plot.sh
    new_timestep=$($main_path/Scarab-Trace-and-Simulate-Script/plot.sh ${timesteps_path} | tail -n 1)
    restart_flag=false
    
    # Check the condition for restarting
    if [ $new_timestep -lt $((current_timestep + num_proc)) ]; then
        restart_flag=true
    else
        restart_flag=false
    fi
    
    # Update the current timestep
    current_timestep=$new_timestep
    
    echo "Updated current timestep: $current_timestep"
    echo ""
done