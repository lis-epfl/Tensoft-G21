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
 * @copyright Copyright (C) 2018 LIS
 * $Id$
*/

// The C++ Standard Library
#include <iostream>
#include <fstream>

#include <cstdlib>
#include <vector>
#include <iomanip>

// NTRT core libraries for this app
#include "ntrt/core/terrain/tgBoxGround.h"
#include "ntrt/core/terrain/tgHillyGround.h"
#include "ntrt/core/tgModel.h"
#include "ntrt/core/tgSimViewGraphics.h"
#include "ntrt/core/tgSimulation.h"
#include "ntrt/core/tgWorld.h"

// data logging libraries
#include "ntrt/sensors/tgDataLogger2.h"
#include "ntrt/sensors/tgRodSensorInfo.h"
#include "ntrt/sensors/tgSpringCableActuatorSensorInfo.h"

#include "robotModel.h"

/**
 * The entry point.
 * @param[in] argc the number of command-line arguments
 * @param[in] argv argv[0] is the executable name
 * @return 0
 */
int main(int argc, char** argv)
{
    const double yaw = 0.0;
    const double pitch = 0.0;
    const double roll = 0.0;

    const tgBoxGround::Config groundConfig(btVector3(yaw, pitch, roll));

    tgBoxGround* ground = new tgBoxGround(groundConfig);

    const tgWorld::Config config(98.1);

    tgWorld world(config, ground);

    // Second create the view
    const double timestep_physics = 0.001; // Seconds
    const double timestep_graphics = 1.f/60.f; // Seconds

    // tgSimView runs the simulation headless
    tgSimView view(world, timestep_physics, timestep_graphics);

    // Third create the simulation
    tgSimulation simulation(view);

    // construct robot model from given inline configuration
    robotModel* rbModel = NULL;

    // init time
    long init_time = 3000;

    const int offset = 2;
    if (argc > offset) {
        // create robot from command line arguments
        // Note: the first two arguments regards noise config,
        // and are not needed to build the robot
        rbModel = new robotModel(argc - offset, argv + offset);

        // movement direction
        rbModel->setMovDir(atoi(argv[1]));
    } else {
        std::cerr << "Wrong program usage. Review which parameters can be used." << std::endl;
        exit(2);
    }

    simulation.addModel(rbModel);

    // determine the best rotation angle
    double best_angle = 0.0;
    double best_residual = 3.15;
    double rot_angle = 0.0;

    std::cout << "Finding best rotation angle..." << std::endl;
    std::cout.setstate(std::ios_base::failbit);

    int num_trials = 10;
    for (int i=0; i < num_trials; i++) {
        // run the simulation for three seconds to initialize the robot
        // and reduce construction/rotation/falling effects on the fitness
        simulation.run(init_time);

        // get robot initial position (front and parallel face)
        btVector3 rbInitPos(0.0, 0.0, 0.0);
        btVector3 rbInitPosP(0.0, 0.0, 0.0);
        rbModel->getFirstModulePosition(rbInitPos, rbInitPosP);

        // compute robot front face direction
        btVector3 frontFaceDir = rbInitPos - rbInitPosP;

        // determine angle w.r.t. z-axis and normalize in interval [-pi, +pi]
        double angle = atan2(frontFaceDir.getZ(), frontFaceDir.getX()) - atan2(1.0, 0.0);
        if (angle < -M_PI) {
            angle += 2 * M_PI;
        } else if (angle > M_PI) {
            angle -= 2 * M_PI;
        }

        if (std::abs(angle) < best_residual){
            best_angle = rot_angle;
            best_residual = std::abs(angle);
        }

        if(i != (num_trials-1)) {
            rot_angle += (0.75 * angle);
            rbModel->setRotationAngle(rot_angle);
            simulation.reset();
        }
    }
    std::cout.clear();

    std::cout << "residual: " << std::setprecision(20) << best_residual << std::endl;
    std::cout << "rotation angle: " << best_angle << std::endl;

    // exit here is necessary to avoid other
    // components to write on the standard output
    exit(0);
}


