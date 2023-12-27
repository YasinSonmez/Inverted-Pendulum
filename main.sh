#! /usr/bin/bash
# cd Scarab-Trace-and-Simulate-Script
# ./trace.sh ~/scarab ./../build/simulate_and_record ./../build/main 0 8
# ./simulate.sh
# cd ..
# index=$(python plot_cycles.py Scarab-Trace-and-Simulate-Script/2023-12-15_13-05-03/simulation)
# echo "index: $index"

main_path=$(pwd)
echo "Current path: " $main_path

#cd Scarab-Trace-and-Simulate-Script
start_idx=0
total_timesteps=10
current_timestep=0
num_proc=8
current_time=$(date +'%Y-%m-%d_%H-%M-%S')
main_path=$(pwd)
main_path=$main_path"/Scarab-Trace-and-Simulate-Script/$current_time"
mkdir -p "$main_path"
restart_flag=false
while [ $current_timestep -lt $total_timesteps ]; do
    cd "$main_path"
    echo "Current path: " $(pwd)
    echo "Current timestep: $current_timestep"
    echo "restart_flag: $restart_flag"
    
    # Run trace.sh with restart_flag as an argument
    ./../trace.sh ~/scarab ./../../build/simulate_and_record ./../../build/main $current_timestep $num_proc $restart_flag
    
    # Get the new timestep from simulate.sh
    new_timestep=$(./../simulate.sh | tail -n 1)
    restart_flag=false
    # Check the condition for restarting
    if [ $new_timestep -lt $((current_timestep + num_proc)) ]; then
        restart_flag=true
    else
        restart_flag=false
    fi
    
    # Update the current timestep
    current_timestep=$new_timestep
    
    echo "Current timestep: $current_timestep"
done