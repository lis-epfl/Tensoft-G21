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

/**
 * @author Enrico Zardini
 * @copyright Copyright (C) 2019 LIS
 * $Id$
 */

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/uniform_real_distribution.hpp>

#include <fstream>
#include <libgen.h>
#include <math.h>

// NTRT library
#include "ntrt/core/tgBasicActuator.h"
#include "ntrt/controllers/tgImpedanceController.h"
#include "ntrt/controllers/tgBasicController.h"
#include "ntrt/tgcreator/tgUtil.h"

#include "neuralNetGR.h"
#include "robotModel.h"
#include "robotControllerGR.h"
#include "targetModel.h"

robotControllerGR::robotControllerGR(char* controller_nn_path, targetModel* tModel, long init_time, int noiseType, double noiseLevel, int seed, char* log_file_path)
    : robotController(noiseType, noiseLevel, seed),
      tModel(tModel),
      init_time(init_time)
{
    controller_nn = new neuralNetGR(controller_nn_path);

    #ifdef CONTROLLER_INFO
        init_data_file_path(log_file_path);
    #endif
}

robotControllerGR::robotControllerGR(char* controller_nn_path, targetModel* tModel, long init_time, int noiseType, double noiseLevel, int seed, std::vector<double>& Ks, char* log_file_path)
    : robotController(noiseType, noiseLevel, seed, Ks),
      tModel(tModel),
      init_time(init_time)
{
    controller_nn = new neuralNetGR(controller_nn_path);

    #ifdef CONTROLLER_INFO
        init_data_file_path(log_file_path);
    #endif
}

void robotControllerGR::onStep(robotModel& subject, double dt)
{
    simTime += dt;
    iteration++;
    segments = subject.m_moduleData.size();

#ifdef CONTROLLER_INFO
    if (iteration == init_time) {
        btVector3 init_ff(0.0, 0.0, 0.0);
        btVector3 init_pf(0.0, 0.0, 0.0);
        subject.getFirstModulePosition(init_ff, init_pf);

        init_tranformation_variables(init_ff, init_ff-init_pf);
        write_null_io(2, 2*segments);
    }
#endif

    if (iteration > init_time) {
        // get current position of the target
        btVector3 targetPos(0.0, 0.0, 0.0);
        tModel->getPosition(targetPos);

        // compute distance and bearing wrt target position
        double distance, bearing;
        subject.getDistAndBearingToGoal(targetPos, distance, bearing);

        // compute modules frequencies and phases
        double input_values[] = {distance, bearing};
        double modules_params[2 * segments];
        controller_nn->compute_module_params(input_values, modules_params, 2 * segments);

        // get movement direction of the robot
        bool direct_association = false;
        int mov_dir = subject.getMovDir();

#ifdef CONTROLLER_INFO
        btVector3 robotPos(0.0, 0.0, 0.0);
        subject.getCurrentPosition(robotPos);
        store_data(input_values, 2, modules_params, 2*segments, transform_coordinates(robotPos));
#endif

#ifdef DIRECT_ASSOC
        direct_association = true;
#endif

        const std::vector<tgBasicActuator *> &allActuators = subject.getActuators("motor");
        // cycle over all the modules of the robot
        for (std::size_t i = 0; i < segments; i++) {
            std::vector<tgBasicActuator *>::const_iterator first = allActuators.begin() + numActuatedCables * i;
            std::vector<tgBasicActuator *>::const_iterator last =
                    allActuators.begin() + numActuatedCables * i + numActuatedCables;
            std::vector < tgBasicActuator * > segmentActuators(first, last);

            int f_index, p_index;
            f_index = p_index = -1;
            if (mov_dir == 1 or direct_association) {
                f_index = 2 * i;
                p_index = 2 * i + 1;
            } else {
                f_index = (2 * segments - 1) - (2 * i + 1);
                p_index = (2 * segments - 1) - (2 * i);
            }

            applyImpedanceControlInside(segmentActuators, dt, i,
                                        modules_params[f_index],
                                        subject.m_moduleData[i].amplitude,
                                        modules_params[p_index]);
        }
    }
}

void robotControllerGR::onTeardown(robotModel& subject) {
    robotController::onTeardown(subject);
    iteration = 0;
}

bool ends_with(const char* str, const char* suffix) {
    int len = strlen(str);
    int suffix_len = strlen(suffix);
    if(suffix_len > len) {
        return false;
    }

    str += (len - suffix_len);
    return strcmp(str, suffix) == 0;
}

void robotControllerGR::init_data_file_path(char* log_file_path) {
    data_file_path = new char[PATH_MAX];
    char extension[] = ".csv";

    if (strncmp(log_file_path, "/", 1) == 0) {
        strcpy(data_file_path, log_file_path);
    } else {
        char *exec_path = new char[PATH_MAX];
        char data_file_rel_path[] = "nn_data/";

        ssize_t count = readlink("/proc/self/exe", exec_path, PATH_MAX);
        if (count == -1) {
            std::cout << "FAILED to find executable location." << std::endl;
            throw "FAILED to find executable location.";
        }

        strcpy(data_file_path, dirname(exec_path));
        strcpy(strrchr(data_file_path,'/')+1,data_file_rel_path);
        if(strrchr(log_file_path,'/') != NULL){
            strcat(data_file_path, strrchr(log_file_path,'/')+1);
        } else {
            strcat(data_file_path, log_file_path);
        }
    }

    if (!ends_with(data_file_path, extension)) {
        strcat(data_file_path, extension);
    }

    std::cout << "Log file: " << data_file_path << std::endl;
}

void robotControllerGR::init_tranformation_variables(btVector3 init_pos, btVector3 init_dir) {
    translation << -init_pos.getX(), -init_pos.getZ();

    double angle = atan2(1.0, 0.0) - atan2(init_dir.getZ(), init_dir.getX());
    if (angle < -M_PI) {
        angle += 2 * M_PI;
    } else if (angle > M_PI) {
        angle -= 2 * M_PI;
    }

    rotation << cos(angle), -sin(angle),
                sin(angle),  cos(angle);
}

void robotControllerGR::write_null_io(int inputs_size, int outputs_size) {
    std::ofstream data_file;
    data_file.open(data_file_path, std::ios_base::out);

    double null_inputs[inputs_size];
    double null_outputs[outputs_size];

    data_file << "simTime";
    for(int i=0; i < inputs_size; i++) {
        data_file << "," << inputs_names[i];
        null_inputs[i] = 0.0;
    }
    for(int j=0; j < outputs_size; j++) {
        data_file << "," << (j%2 == 0 ? "frequency_m" : "phase_m") << (j/2 +1);
    }
    data_file << ",transf_x,transf_z" << std::endl;

    controller_nn->compute_module_params(null_inputs, null_outputs, outputs_size);

    data_file << "0";
    for(int i=0; i < inputs_size; i++) {
        data_file << "," << null_inputs[i];
    }
    for(int j=0; j < outputs_size; j++) {
        data_file << "," << null_outputs[j];
    }
    data_file << ",0,0" << std::endl;

    data_file.close();
}

Eigen::Vector2f robotControllerGR::transform_coordinates(btVector3 robotPos){
    Eigen::Vector2f eigen_pos(robotPos.getX(), robotPos.getZ());

    return rotation*(eigen_pos+translation);
}

void robotControllerGR::store_data(double inputs[], int inputs_size, double outputs[], int outputs_size, Eigen::Vector2f transfRobotPos) {
    std::ofstream data_file;
    data_file.open(data_file_path, std::ios_base::app);

    data_file << simTime;
    for(int i=0; i < inputs_size; i++) {
        data_file << "," << inputs[i];
    }
    for(int j=0; j < outputs_size; j++) {
        data_file << "," << outputs[j];
    }
    data_file << "," << transfRobotPos.x() << "," << transfRobotPos.y() << std::endl;

    data_file.close();
}