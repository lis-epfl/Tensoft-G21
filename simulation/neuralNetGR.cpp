/**
 * @author Enrico Zardini
 */

#include <iostream>
#include <cstdlib>
#include <cstring>
#include <libgen.h>
#include <limits.h>
#include <unistd.h>
#include "math.h"
#include <string>
#include <fstream>
#include <sstream>

#include "neuralNetGR.h"


// constructor
neuralNetGR::neuralNetGR(char *nn_path) {
    std::ifstream nn_file(nn_path);
    std::string line;

    if (!nn_file.good()) {
        std::cout << "FAILED to find controller location." << std::endl;
        throw "FAILED to find controller location.";
    }

    // read input and output keys
    for(int i=0; i<2 && std::getline(nn_file, line); i++){
        std::istringstream iss(line);

        int num_values, val;
        iss >> num_values;

        for(int j=0; j < num_values; j++) {
            iss >> val;
            if (i == 0) {
                input_keys.push_back(val);
            } else {
                output_keys.push_back(val);
            }
        }
    }

    while (std::getline(nn_file, line) && line.rfind("fitness", 0) != 0) {
        std::istringstream iss(line);

        Neuron* neuron = new Neuron;
        int num_inputs;

        iss >> neuron->node_id >> neuron->bias;

        iss >> num_inputs;
        for (int i=0; i < num_inputs; i++) {
            std::pair<int,double> input;
            iss >> input.first >> input.second;
            neuron->inputs.push_back(input);
        }

        proc_order.push_back(neuron->node_id);
        network[neuron->node_id] = neuron;
    }

    nn_file.close();

    double frequencies_values[] = {0.25, 0.5};
    frequencies.insert(frequencies.begin(), frequencies_values, frequencies_values+2);

    double phases_values [] = {0, 1.57, 3.14, 4.71};
    phases.insert(phases.begin(), phases_values, phases_values+4);
}

// destructor
neuralNetGR::~neuralNetGR(void) {}

void neuralNetGR::compute_module_params(double input_values[], double module_params[], int num_outputs){
    for (int node_id: proc_order) {
        Neuron* neuron = network[node_id];

        double aggregation = 0.0;
        for(auto input: neuron->inputs) {
            if (input.first < 0) {
                aggregation += input_values[-input.first-1] * input.second;
            } else {
                aggregation += network[input.first]->output * input.second;
            }
        }
        aggregation += neuron->bias;

        neuron->output = 1.0/(1.0 + exp(-aggregation));
    }


    for(int i=0; i < num_outputs; i++) {
        std::map<int,Neuron*>::iterator it = network.find(output_keys[i]);

        double out;
        if (it != network.end()) {
            out = it->second->output;
        } else {
            out = 0.0;
        }

        if(i%2 == 0) { // frequency
            module_params[i] = frequencies[std::min(int(floor(out*frequencies.size())), int(frequencies.size())-1)];
        } else { // phase
            module_params[i] = phases[std::min(int(floor(out*phases.size())), int(phases.size())-1)];
        }
    }
}

void neuralNetGR::printNN() {
    std::cout << "-----------------------------------------------" << std::endl;

    std::cout << input_keys.size() << " ";
    for (auto i_k: input_keys){
        std::cout << i_k << " ";
    }
    std::cout << std::endl;

    std::cout << output_keys.size() << " ";
    for (auto o_k: output_keys){
        std::cout << o_k << " ";
    }
    std::cout << std::endl;

    for (auto p: proc_order){
        std::cout << p << " ";
    }
    std::cout << std::endl;

    for (auto neuron: network) {
        std::cout << neuron.first << " " << neuron.second->bias << " ";
        for (auto pair: neuron.second->inputs) {
            std::cout << pair.first << " " << pair.second << " ";
        }
        std::cout << std::endl;
    }

    std::cout << "-----------------------------------------------" << std::endl;
}
