#!/usr/bin/env bash

echo "# ============================================== #"
echo "#                                                #"
echo "#           Semi-automatic script for            #"
echo "#         measuring cables length in the         #"
echo "#     simulator and produce the physic model     #"
echo "#         associated to cable compression        #"
echo "#                                                #"
echo "# ============================================== #"
echo "#                                                #"
echo "#  first parameter: folder where data and charts #"
echo "#                   will be stored               #"
echo "# ============================================== #"
echo "#                                                #"
echo "#  second parameter: --cmd2comp                  #"
echo "#                    if set, create the graph    #"
echo "#                    of command vs compression   #"
echo "#                    physic model                #"
echo "#                                                #"
echo "# ============================================== #"
echo "#                                                #"
echo "#   Note: please verify that option STEP_INPUT   #"
echo "#   is added to robotSimulation before running   #"
echo "#   Phase 2 code (and later revert the change)   #"
echo "#                                                #"
echo "# ============================================== #"

# data folder and test phase selection
main_output_folder=$1
if [[ "$1" == "" ]]; then main_output_folder=after_cable_stretch_; fi
mkdir -p ${main_output_folder}

echo -e "\nResults will be stored in ${main_output_folder}\n"

plot_cmd_comp=$2
# ================================== #

# compile the simulation for being executed in test mode
rm -r apps build
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DTEST_SINGLE_MODULE=ON -DSTEP_INPUT=ON
cmake --build build -j

if [[ "$plot_cmd_comp" != "--cmd2comp" ]]; then
    echo -e "\n#============ Phase 1 ============ #\n"
    echo "Measuring cables length for each module rotation..."
    for conf_file in ./test_inputs/single_mods/*; do
        python ./measurements_scripts/plot_nn_commands.py ./apps/robotStatic ${conf_file}\
               ${main_output_folder}/len_measures ;
    done

    echo "Extracting stable measurements from collected data..."
    for data_file in ${main_output_folder}/len_measures/data_*_noise_*_face*.csv; do
        python ./measurements_scripts/cable_len_estimation.py ${data_file}\
               ${main_output_folder}/cable_len_est ;
    done

    echo "Computing cables len averages and plotting respective bar-charts to analyze the results..."
    # (different behaviour from above because the first parameters is now a folder rather than a file)
    python ./measurements_scripts/cable_len_estimation.py ${main_output_folder}/cable_len_est\
           ${main_output_folder}/analysis ;

    echo -e "\nInsert above AL values into robotController.h!\n"

else
    echo -e "\n#============ Phase 2 ============ #\n"
    # NOTE!
    # Before running the command to comparison branch,
    # replace mean cable len values produced from previous step (AL) in the controller code (array AL)
    echo "Collecting cables lengths behaviour over the different input commands..."
    for file in ./test_inputs/stiff_single/*; do
        python ./measurements_scripts/plot_nn_commands.py ./apps/robotSimulation ${file}\
               ${main_output_folder}/sources ;
    done
    for file in ./test_inputs/stiff_single/*; do
        python ./measurements_scripts/plot_nn_commands.py ./apps/robotSimulation ${file}\
               ${main_output_folder}/sources 1 0.035;
    done
    echo "Plotting the results..."
    for stiff in '0.1' '0.5' '1'; do
        for noise in '0.0' '0.035'; do
            python ./measurements_scripts/plot_cmd_comp.py\
                   ${main_output_folder}/sources/data_${stiff}_noise_${noise}_face*.csv\
                   ${main_output_folder}/cmd2comp ${stiff} ${noise};
        done
    done

    echo "Plotting the results (absolute)..."
    for stiff in '0.1' '0.5' '1'; do
        for noise in '0.0' '0.035'; do
            python ./measurements_scripts/plot_cmd_comp.py\
                   ${main_output_folder}/sources/data_${stiff}_noise_${noise}_face*.csv\
                   ${main_output_folder}/cmd2comp_abs ${stiff} ${noise} --absolute;
        done
    done
fi

# compiles back the simulation apps for running EAs experiments
rm -r apps build
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DTEST_SINGLE_MODULE=OFF -DSTEP_INPUT=OFF
cmake --build build -j