#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cstring>
#include <libgen.h>
#include <limits.h>
#include <unistd.h>
#include "neuralNet.h"


// constructor
neuralNet::neuralNet() {
    size_state = 1;

    scaler_X =  0.00555741;
    X_min    = -0.00033344;

    scaler_Y.resize(LAYER_SIZE_2);
    Y_min.resize(LAYER_SIZE_2);

    W_fc1.resize(LAYER_SIZE_1);
    W_fc2.resize(LAYER_SIZE_1, LAYER_SIZE_1);
    W_fc3.resize(LAYER_SIZE_1, LAYER_SIZE_2);

    b_fc1.resize(LAYER_SIZE_1);
    b_fc2.resize(LAYER_SIZE_1);
    b_fc3.resize(LAYER_SIZE_2);

    scaler_Y << 60.99636407, 96.67252525, 41.69332778, 46.12917713, 57.37630381;
    Y_min << -2.0624877, -3.37442644, -0.86497624, -1.31366324, -1.87704198;

    char *exec_path = new char[PATH_MAX];
    char weights_sim_path[] = "nn_weights/weights_keras_model5";

    ssize_t count = readlink("/proc/self/exe", exec_path, PATH_MAX);
    if (count == -1) {
        std::cout << "FAILED to find executable location." << std::endl;
        throw "FAILED to find executable location.";
    }

    char *weights_file = dirname(exec_path);
    strcpy(strrchr(weights_file,'/')+1,weights_sim_path);

    std::ifstream weights;

    weights.open(weights_file);
    if (weights.is_open()) {
        std::cout << "Weights file (5) opened." << std::endl;
    }
    else {
          std::cout << "FAILED to open weights file." << std::endl;
          throw "FAILED to open weights file.";
    }

    // load first fully-connected layer weights
    for (int i = 0; i < LAYER_SIZE_1; i++) {
        weights >> W_fc1(i);
    }
    // load first layer biases
    for (int i = 0; i < LAYER_SIZE_1; i++) {
      weights >> b_fc1(i);
    }

      // load second fully-connected layer weights
    for (int i = 0; i < LAYER_SIZE_1; i++) {
      for(int j = 0; j < LAYER_SIZE_1; j++) {
        weights >> W_fc2(i, j);
      }
    }
      // load second layer biases
    for (int i = 0; i < LAYER_SIZE_1; i++) {
      weights >> b_fc2(i);
    }

      // load third fully-connected layer weights
    for (int i = 0; i < LAYER_SIZE_1; i++) {
      for (int j = 0; j < LAYER_SIZE_2; j++) {
        weights >> W_fc3(i, j);
      }
    }
      // load third layer biases
    for (int i = 0; i < LAYER_SIZE_2; i++) {
      weights >> b_fc3(i);
    }

    weights.close();
    std::cout << "Weights file closed." << std::endl;
}

// destructor
neuralNet::~neuralNet(void) {}

void neuralNet::neuralNet_commands(double input, double commands[]){

    Eigen::VectorXd out_fc1(LAYER_SIZE_1);
    Eigen::VectorXd out_fc2(LAYER_SIZE_1);
    Eigen::VectorXd out_fc3(LAYER_SIZE_2);
    Eigen::ArrayXd output(LAYER_SIZE_2);

    double scaled_input = input * 0.00555741 - 0.00033344;

    // TODO: combine weights and biases in a single matrix?
    /* ============= NN ============= */
    out_fc1 = W_fc1 * scaled_input;
    out_fc1 += b_fc1;

    // apply ReLU function to each element
    for (int i = 0; i < LAYER_SIZE_1; i++) {
      if (out_fc1[i] < 0.0) out_fc1[i] = 0.0;
    }

    out_fc2 = W_fc2.transpose() * out_fc1;
    out_fc2 += b_fc2;

    // apply ReLU function to each element
    for (int i = 0; i < LAYER_SIZE_1; i++) {
      if (out_fc2[i] < 0.0) out_fc2[i] = 0.0;
    }

    out_fc3 = W_fc3.transpose() * out_fc2;
    out_fc3 = out_fc3 + b_fc3 - Y_min;
    output << out_fc3.array() / scaler_Y.array();

    for (int i = 0; i < LAYER_SIZE_2; i++) {
      commands[i] = output[i];
    }
}

