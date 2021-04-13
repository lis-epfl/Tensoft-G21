/**
 * @author Enrico Zardini
 */

#ifndef ROBOT_SIMULATION_NEURALNETGR_H
#define ROBOT_SIMULATION_NEURALNETGR_H

#include <map>
#include <vector>

struct Neuron
{
    int node_id;
    double bias;
    std::vector<std::pair<int,double>> inputs;
    double output;
};

class neuralNetGR
{
public:
    //constructor
    neuralNetGR(char *nn_path);

    //destructor
    ~neuralNetGR();

    //command provider
    void compute_module_params(double input_values[], double module_params[], int num_outputs);

    void printNN();

private:
    std::vector<int> input_keys;
    std::vector<int> output_keys;

    std::vector<int> proc_order;

    std::map<int,Neuron*> network;

    std::vector<double> frequencies;
    std::vector<double> phases;
};

#endif //ROBOT_SIMULATION_NEURALNETGR_H
