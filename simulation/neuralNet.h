#ifndef NN_CONT_LIB
#define NN_CONT_LIB

#include <Eigen/Core>

const int LAYER_SIZE_1 = 256;
const int LAYER_SIZE_2 = 5;

class neuralNet
{
public:
	//constructor
	neuralNet();

	//destructor
	~neuralNet();

	//command provider
	void neuralNet_commands(double input, double commands[]);

private:
	int size_state;

  double scaler_X;
  double X_min;

  Eigen::ArrayXd scaler_Y;
  Eigen::VectorXd Y_min;

  Eigen::VectorXd W_fc1;
  Eigen::MatrixXd W_fc2;
  Eigen::MatrixXd W_fc3;

  Eigen::VectorXd b_fc1;
  Eigen::VectorXd b_fc2;
  Eigen::VectorXd b_fc3;
};

#endif
