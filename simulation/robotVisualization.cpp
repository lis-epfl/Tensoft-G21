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
 * @file robotVisualization.cpp
 * @brief Application to visualize the behaviour of evolved robots
 * @author Daniele Bissoli
 * @copyright Copyright (C) 2019 LIS
 * $Id$
*/

// The C++ Standard Library
#include <iostream>
#include <fstream>

#include <cstdlib>

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
#include "robotController.h"

/**
 * The entry point.
 * @param[in] argc the number of command-line arguments
 * @param[in] argv argv[0] is the executable name
 * @return 0
 */
int main(int argc, char** argv) {
    const double yaw = 0.0;
    const double pitch = 0.0;
    const double roll = 0.0;

    const tgBoxGround::Config groundConfig(btVector3(yaw, pitch, roll));
    const tgHillyGround::Config hillyConfig(
        btVector3(yaw, pitch, roll),
        1,
        0,
        btVector3(500.0, 1.5, 500.0),
        btVector3(0,0,0),
        100,
        100,
        0.05,
        10,
        5,
        0
    );

    tgBoxGround* ground = new tgBoxGround(groundConfig);
    tgHillyGround* hillyground = new tgHillyGround(hillyConfig);

    const tgWorld::Config config(98.1);
    tgWorld world(config, ground);
//    tgWorld world(config, hillyground);

    // set up simulation viewer
    const double timestep_physics = 0.001; // Seconds
    const double timestep_graphics = 1.f/60.f; // Seconds
    tgSimViewGraphics view(world, timestep_physics, timestep_graphics);
    tgSimulation simulation(view);

    // create the robot model either from a conf file or from inline values
    robotModel* myModel = NULL;
    if (argc >= 3) {
        // from conf files
        if (std::string(argv[1]) == "file") {
            // value 1 -> inform the model to read conf from file
            // advance argv of 2 to point to the conf filepath
            myModel = new robotModel(1, argv + 2);

            // in case also noise values are given (after filepath)
            if (argc == 6) {
                int noiseType = atoi(argv[3]);
                double noiseLevel = atof(argv[4]);
                int seed = atoi(argv[5]);
                robotController* const pMuscleControl =
                        new robotController(noiseType, noiseLevel, seed);
                myModel->attach(pMuscleControl);
            }
            // otherwise use default noiseless behaviour
            else {
                robotController* const pMuscleControl = new robotController();
                myModel->attach(pMuscleControl);
            }

        }
        // from command line arguments
        else {
            if (std::string(argv[1]) == "in") {
                // Note: remember that this method requires also the position
                // where the robot will start moving as first parameter before its configuration
                myModel = new robotModel(argc - 5, argv + 5);

                // noise related parameters
                int noiseType = atoi(argv[2]);
                double noiseLevel = atof(argv[3]);
                int seed = atoi(argv[4]);

                robotController* const pMuscleControl =
                        new robotController(noiseType, noiseLevel, seed);
                myModel->attach(pMuscleControl);
            }
            else {
                std::cerr << "Wrong program usage."
                          << "Review which parameters can be used."
                          << std::endl;
                exit(2);
            }
        }
    }
    else {
        std::cerr << "Wrong program usage."
                  << "Review which parameters can be used."
                  << std::endl;
        exit(2);
    }

#ifdef SIN_CONTROLLER
    std::cout << "Sinusoidal controller exploited." << std::endl;
#else
    std::cout << "NN controller exploited." << std::endl;
#endif

    simulation.addModel(myModel);

    // display the simulation and run it until exit key (q) is pressed
    simulation.run();

    return 0;
}
