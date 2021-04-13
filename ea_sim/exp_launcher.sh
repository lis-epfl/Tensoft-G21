#! /bin/bash

# notify the start of the experiments run
python notifier.py "A new set of experiments has been launched!"

# execute all the experiments defined by provided configurations
experiments=(18_map-elites_selected_noisy.json
             05_vie_selected_noisy.json
             13_mu+l_selected_noisy.json)

# owner of results dir, optional parameter
owner=$1

for exp in ${experiments[*]};
do
    python main.py experiments/${exp}
    res=$?

    # notify the end of current experiment (with the exit code)
    python notifier.py ${res} `echo ${exp} | grep -o "[0-9][0-9]"`

    if [[ ! -z "$owner" ]]; then
        chown -R ${owner}:${owner} ../results
    fi
done

# plot experiments results
python visualization/data_plotter.py ../../results/13_mu+l_selected_noisy ../../results/05_vie_selected_noisy ../../results/18_map-elites_selected_noisy --out-file alg_comp.pdf --labels "mu+lambda" "ViE" "MAP-Elites"

# notify the end of all the experiments
python notifier.py "Experiments completed! Check the server that no error occurred."

if [[ ! -z "$owner" ]]; then
    chown -R ${owner}:${owner} ../results
fi