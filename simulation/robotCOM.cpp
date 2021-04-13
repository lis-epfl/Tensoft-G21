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
 * @file robotCOM.cpp
 * @brief App for measuring rods center of mass throughout the simulation
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
#include "ntrt/core/tgModel.h"
#include "ntrt/core/tgSimViewGraphics.h"
#include "ntrt/core/tgSimulation.h"
#include "ntrt/core/tgWorld.h"

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
    if (argc >= 3) {
        // from conf files
        if (std::string(argv[1]) == "file") {
            myModel = new robotModel(1, argv + 2);
        }
        // from command line arguments
        else {
            myModel = new robotModel(argc - 3 , argv + 3);
        }

        robotController* const pMuscleControl = new robotController();
        myModel->attach(pMuscleControl);
    }
    else {
        std::cerr << "Wrong program usage."
                  << "Review which parameters can be used."
                  << std::endl;
        exit(2);
    }

    simulation.addModel(myModel);

    #ifdef COM_COLLECTION
        // for measuring cable length
        #ifdef STEP_INPUT
            simulation.run(800000);
        #else
            simulation.run(200000);
        #endif
        // to delimit debug data
        std::cout << "è,";
    #else
        std::cout << "Warning: program has not been compiled with COM_COLLECTION flag!\n"
                  << "Please add the flag and compile the program again." << std::endl;
    #endif

    exit(0);
}