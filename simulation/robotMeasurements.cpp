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
 * @file robotStatic.cpp
 * @brief Visualization app with the purpose of simplifying
 *        the understanding how to connect robot modules
 * @author Daniele Bissoli
 * @copyright Copyright (C) 2019 LIS
 * $Id$
*/

// The C++ Standard Library
#include <iostream>
#include <fstream>
#include <vector>
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
    const int numActiveCables = 5;

    const tgBoxGround::Config groundConfig(btVector3(yaw, pitch, roll));
    tgBoxGround* ground = new tgBoxGround(groundConfig);

    const tgWorld::Config config(98.1);
    tgWorld world(config, ground);

    // set up simulation viewer
    const double timestep_physics = 0.001; // Seconds
    const double timestep_graphics = 1.f/60.f; // Seconds

    tgSimView view(world, timestep_physics, timestep_graphics);
    tgSimulation simulation(view);

    // create the robot model either from a conf file or from inline values
    robotModel* myModel = NULL;
    if (argc == 7 or argc == 9) {
        int noiseType = 0;
        double noiseLevel = 0.0;
        int seed = 42;

        // load noise parameters if provided
        if (argc == 9) {
            noiseType = atoi(argv[7]);
            noiseLevel = atof(argv[8]);
        }
        if (argc == 10) seed = atoi(argv[9]);

        std::vector<double> constantsModifiers;
        for (int i = 0; i < numActiveCables; i++) {
            constantsModifiers.push_back(std::stod(argv[2 + i]));
        }
        myModel = new robotModel(1, argv + 1);

        robotController* const pMuscleControl =
                new robotController(noiseType, noiseLevel, seed, constantsModifiers);
        myModel->attach(pMuscleControl);
    }
    else {
        std::cerr << "Wrong program usage."
                  << "Review which parameters can be used."
                  << std::endl;
        exit(2);
    }

    simulation.addModel(myModel);
    // for measuring cable length
    simulation.run(150000);

#ifdef TEST_SINGLE_MODULE
    const std::vector<tgBasicActuator*>& allActuators = myModel->getActuators("motor");

    std::vector<tgBasicActuator*>::const_iterator first = allActuators.begin();
    std::vector<tgBasicActuator*>::const_iterator last = allActuators.begin() + numActiveCables;
    std::vector<tgBasicActuator*> segmentActuators(first, last);

    for (int i = 0; i < numActiveCables; i++) {
        std::cout << segmentActuators[i]->getCurrentLength() << ",";
    }
    std::cout << std::endl;
#endif

    exit(0);
}