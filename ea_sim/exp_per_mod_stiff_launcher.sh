#!/usr/bin/env bash

# notify the start of the experiments run
python notifier.py "A new set of experiments has been launched!"

# execute all the experiments defined by provided configurations
experiments=(s6_map-elites_selected_noisy.json
             s2_vie_selected_noisy.json
             s4_mu+l_selected_noisy.json)

for exp in ${experiments[*]};
do
    python main.py exps_per_mod_stiff/${exp}
    res=$?

    # notify the end of current experiment (with the exit code)
    python notifier.py ${res} `echo ${exp} | grep -o "s[0-9][0-9]"`
done

# plot experiments results
python visualization/data_plotter.py ../../results/s4_mu+l_selected_noisy ../../results/s2_vie_selected_noisy ../../results/s6_map-elites_selected_noisy --out-file alg_comp.pdf --labels "mu+lambda" "ViE" "MAP-Elites"

# notify the end of all the experiments
python notifier.py "Experiments completed! Check the server that no error occurred."