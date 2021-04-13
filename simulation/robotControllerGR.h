/*
 * Copyright © 2012, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * All rights reserved.
 * 
 * The NASA Tensegrity Robotics Toolkit (NTRT) v1 platform is licensed
 * under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0.
 * 
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific language
 * governing permissions and limitations under the License.
*/

#ifndef ROBOT_CONTROLLER_GR_H
#define ROBOT_CONTROLLER_GR_H

/**
 * @author Enrico Zardini
 * @version 1.0.0
 * $Id$
 */

// The C++ Standard Library
#include <vector>
#include <Eigen/Dense>

#include "neuralNetGR.h"
#include "targetModel.h"
#include "robotController.h"

class robotControllerGR : public robotController
{
public:

    /**
     * Construct the controller. Typically occurs in the main function.
     * The controller will need to be attached to a subject (model)
     * Parameters are currently set in the initalizer lists.
     */
    robotControllerGR(char* controller_nn_path, targetModel* tModel, long init_time, int noiseType = 0, double noiseLevel = 0.0, int seed = 42, char* log_file_path = NULL);

    /* Special constructor needed for evolving K constants that modify cables compression */
    robotControllerGR(char* controller_nn_path, targetModel* tModel, long init_time, int noiseType, double noiseLevel, int seed, std::vector<double>& Ks, char* log_file_path = NULL);

    /**
     * Apply the sineWave controller. Called my notifyStep(dt) of its
     * subject. Calls the applyImpedanceControl functions of this class
     * @param[in] subject - the MovingBallModel that is being
     * Subject must have a MuscleMap populated
     * @param[in] dt, current timestep must be positive
     */
    virtual void onStep(robotModel& subject, double dt);

    /**
     * Reset class members
     * @param subject
     */
    virtual void onTeardown(robotModel& subject);

protected:
    long iteration;

    long init_time;

    Eigen::Matrix2f rotation;

    Eigen::Vector2f translation;

    /* Neural network for goal reaching */
    neuralNetGR *controller_nn;

    /* target model */
    targetModel* tModel;

    char* data_file_path;

    void init_data_file_path(char* log_file_path);

    void init_tranformation_variables(btVector3 init_pos, btVector3 init_dir);

    void write_null_io(int inputs_size, int outputs_size);

    Eigen::Vector2f transform_coordinates(btVector3 robotPos);

    void store_data(double inputs[], int inputs_size, double outputs[], int outputs_size, Eigen::Vector2f transfRobotPos);

    char const *inputs_names [5] = {"t_distance", "t_bearing", "obstacle", "w_distance", "w_bearing"};
};

#endif // ROBOT_CONTROLLER_GR_H
