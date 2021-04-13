#!/usr/bin/env bash

echo "# ============================================ #"
echo "#                                              #"
echo "#            Automatic script for              #"
echo "#      collecting the Center of Mass (COM)     #"
echo "#    of each Tensegrity rod adn measure the    #"
echo "#             module compression               #"
echo "#                                              #"
echo "# ============================================ #"
echo "#                                              #"
echo "#  parameters:                                 #"
echo "#     -  folder where results will be stored   #"
echo "#                                              #"
echo "# ============================================ #"

# data folder selection
main_output_folder=$1
if [[ "$1" == "" ]]; then main_output_folder=com_measures; fi
mkdir -p ${main_output_folder}

echo -e "\nResults will be stored in ${main_output_folder}\n"
# ================================== #

# compile the simulation for being executed in test mode (no actuation)
rm -r apps build
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DTEST_SINGLE_MODULE=OFF -DSTEP_INPUT=OFF -DNO_ACTIVE_CABLES=ON
cmake --build build -j

echo "Collecting rods COM when no cable is connected, so that no actuation is performed..."
for file in ./test_inputs/stiff_single/single_*.txt; do
    python ./measurements_scripts/measure_coms.py ./apps/robotCOM ${file}\
           ${main_output_folder}/no_cables --no_active_cables ;
done

# compile the simulation for being executed in test mode
rm -r apps build
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DTEST_SINGLE_MODULE=OFF -DSTEP_INPUT=ON -DNO_ACTIVE_CABLES=OFF
cmake --build build -j

echo "Collecting rods COM when the module is actuated..."
for file in ./test_inputs/stiff_single/single_*.txt; do
    python ./measurements_scripts/measure_coms.py ./apps/robotCOM ${file}\
           ${main_output_folder}/actuated ;
done

# compiles back the simulation apps for running EAs experiments
rm -r apps build
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DTEST_SINGLE_MODULE=OFF -DSTEP_INPUT=OFF -DNO_ACTIVE_CABLES=OFF
cmake --build build -j

# plot compression diagrams
python ./measurements_scripts/plot_cmd2com_comp.py ${main_output_folder}/actuated