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
total_timesteps=32
current_timestep=0
num_proc=8
while [ $current_timestep -lt $total_timesteps ]; do
    cd "$main_path/Scarab-Trace-and-Simulate-Script"
    echo "Current path:  $pwd"
    echo "Current timestep: $current_timestep"
    ./trace.sh ~/scarab ./../build/simulate_and_record ./../build/main $current_timestep $num_proc
    current_timestep=$(./simulate.sh | tail -n 1)
done