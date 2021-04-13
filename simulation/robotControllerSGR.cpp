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
#include <math.h>

// NTRT library
#include "ntrt/core/tgBasicActuator.h"
#include "ntrt/controllers/tgImpedanceController.h"
#include "ntrt/controllers/tgBasicController.h"
#include "ntrt/tgcreator/tgUtil.h"

#include "neuralNetGR.h"
#include "obstacleModel.h"
#include "robotModel.h"
#include "robotControllerSGR.h"
#include "targetModel.h"

robotControllerSGR::robotControllerSGR(char* controller_nn_path, targetModel* tModel, long init_time, obstacleModel* walls, int noiseType, double noiseLevel, int seed, char* log_file_path)
    : robotControllerGR(controller_nn_path, tModel, init_time, noiseType, noiseLevel, seed, log_file_path),
      walls(walls) {}

robotControllerSGR::robotControllerSGR(char* controller_nn_path, targetModel* tModel, long init_time, obstacleModel* walls, int noiseType, double noiseLevel, int seed, std::vector<double>& Ks, char* log_file_path)
    : robotControllerGR(controller_nn_path, tModel, init_time, noiseType, noiseLevel, seed, Ks, log_file_path),
      walls(walls) {}

void robotControllerSGR::onStep(robotModel& subject, double dt)
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
        write_null_io(5, 2*segments);
    }
#endif

    if (iteration > init_time) {
        // get current position of the target
        btVector3 targetPos(0.0, 0.0, 0.0);
        tModel->getPosition(targetPos);

        // compute distance and bearing wrt target position
        double t_distance, t_bearing;
        subject.getDistAndBearingToGoal(targetPos, t_distance, t_bearing);

        // get front and parallel faces positions
        btVector3 frontFaceCoM(0.0, 0.0, 0.0);
        btVector3 frontFacePCoM(0.0, 0.0, 0.0);
        subject.getFirstModulePosition(frontFaceCoM, frontFacePCoM);

        // ignore Y coordinates and determine front module's normalized direction
        frontFaceCoM.setY(0);
        frontFacePCoM.setY(0);
        btVector3 frontFaceDir = (frontFaceCoM - frontFacePCoM).normalized();

        // determine if an obstacle is on path
        bool obstacle_on_path = walls->isOnPath(frontFaceCoM, frontFaceDir, obst_detect_range);
        double obstacle = obstacle_on_path ? 1.0 : 0.0;

        // compute distance and bearing wrt the walls opening entrance (distance with sign)
        btVector3 walls_opening_entrance(0.0, 0.0, 0.0);
        walls->get_opening_pos(walls_opening_entrance);

        double woe_distance, woe_bearing;
        subject.getDistAndBearingToGoal(walls_opening_entrance, woe_distance, woe_bearing);
        woe_distance *= walls->get_distance_sign(frontFaceCoM);

        // compute modules frequencies and phases
        double input_values[] = {t_distance, t_bearing, obstacle, woe_distance, woe_bearing};
        double modules_params[2 * segments];
        controller_nn->compute_module_params(input_values, modules_params, 2 * segments);

        // get movement direction of the robot
        int mov_dir = subject.getMovDir();

#ifdef CONTROLLER_INFO
        btVector3 robotPos(0.0, 0.0, 0.0);
        subject.getCurrentPosition(robotPos);
        store_data(input_values, 5, modules_params, 2*segments, transform_coordinates(robotPos));
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
            if (mov_dir == 1) {
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
