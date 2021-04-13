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

    tgBoxGround* ground = new tgBoxGround(groundConfig);
    const tgWorld::Config config(98.1);
    tgWorld world(config, ground);

    // set up simulation viewer
    const double timestep_physics = 0.001; // Seconds
    const double timestep_graphics = 1.f/60.f; // Seconds
#ifdef TEST_SINGLE_MODULE
    tgSimView view(world, timestep_physics, timestep_graphics);
#else
    tgSimViewGraphics view(world, timestep_physics, timestep_graphics);
#endif
    tgSimulation simulation(view);

    // create the robot model either from a conf file or from inline values
    robotModel* myModel = NULL;
    if (argc >= 3) {
        // from conf files
        if (std::string(argv[1]) == "file") {
            // value 1 -> inform the model to read conf from file
            // advance argv of 2 to point to the conf filepath
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

#ifdef MEASURE_CABLES
    std::cout << "\nNodes of the tensegrity in the world position\n" << std::endl;
    const std::vector<tgStructure*> modules = myModel->finalStruct.getChildren();
    for (auto& m : modules) {
            auto nodes = m->getNodes().getNodes();
            for (auto& n : nodes) {
                std::cout << n << std::endl;
            }
    }

    std::cout << "\nCenter of mass BEFORE stable conf\n" << std::endl;
    std::vector<tgBaseRigid*> allRods1 = myModel->find<tgBaseRigid>("rod");
    for (auto& r : allRods1) {
        std::cout << r->getTags() << " - " << r->centerOfMass() << std::endl;

    }
    std::vector<tgSphere*> spheres1 = myModel->find<tgSphere>("sphere");
    for (auto& r : spheres1) {
        std::cout << r->getTags() << " - " << r->centerOfMass() << std::endl;

    }

    simulation.run(120000);

    std::cout << "\nCenter of mass AFTER stable conf\n" << std::endl;
    std::vector<tgBaseRigid*> allRods2 = myModel->find<tgBaseRigid>("rod");
    for (auto& r : allRods2) {
        std::cout << r->getTags() << " - " << r->centerOfMass() << std::endl;
    }
#else
    #ifdef TEST_SINGLE_MODULE
        // for measuring cable length
        simulation.run(200000);
        // to delimit debug data
        std::cout << "è,";
    #else
        // display the simulation and run it until exit key (q) is pressed
        simulation.run();
    #endif
#endif

    exit(0);
}