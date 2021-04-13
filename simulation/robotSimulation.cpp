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
 * @file AppMovingBall_sim.cpp
 * @brief Contains the definition function main() for the MovingBall_sim
 * @author Jean Marc Bejjani
 * @copyright Copyright (C) 2018 LIS
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
int main(int argc, char** argv)
{
    const double yaw = 0.0;
    const double pitch = 0.0;
    const double roll = 0.0;

    const tgBoxGround::Config groundConfig(btVector3(yaw, pitch, roll));
//    const tgHillyGround::Config hillyConfig(
//        btVector3(yaw, pitch, roll),
//        1,
//        0,
//        btVector3(500.0, 1.5, 500.0),
//        btVector3(0,0,0),
//        100,
//        100,
//        0.05,
//        10,
//        5,
//        0
//    );

    // the world will delete this
    tgBoxGround* ground = new tgBoxGround(groundConfig);
//    tgHillyGround* hillyground = new tgHillyGround(hillyConfig);

    const tgWorld::Config config(98.1);
    //const tgWorld::Config config(0.09810);
    //const tgWorld::Config config(981);

    tgWorld world(config, ground);
    // tgWorld world(config, hillyground);

    // Second create the view
    const double timestep_physics = 0.001; // Seconds
    const double timestep_graphics = 1.f/60.f; // Seconds

    // tgSimView runs the simulation headless
    tgSimView view(world, timestep_physics, timestep_graphics);

    // Third create the simulation
    tgSimulation simulation(view);

    // construct robot model from given inline configuration
    robotModel* myModel = NULL;
    const int offset = 4;
    if (argc > offset)
    {
        // create robot from command line arguments
        // Note: the first two arguments regards noise config,
        // and are not needed to build the robot
        myModel = new robotModel(argc - offset, argv + offset);

        // noise related parameters
        int noiseType = atoi(argv[offset - 3]);
        double noiseLevel = atof(argv[offset - 2]);
        int seed = atoi(argv[offset - 1]);
        std::cout << noiseType << " - " << noiseLevel << " - " << seed << std::endl;

        robotController* const pMuscleControl =
                new robotController(noiseType, noiseLevel, seed);
        myModel->attach(pMuscleControl);
    }
    else
    {
        std::cerr << "Wrong program usage. Review which parameters can be used." << std::endl;
        exit(2);
    }

    simulation.addModel(myModel);

    // run the simulation for three seconds to initialize the robot
    // and reduce construction/rotation/falling effects on the fitness
    simulation.run(3000);

    std::vector<tgBaseRigid*> allRods = myModel->find<tgBaseRigid>("rod");

    // initialize center of mass of the robot
    btVector3 totalCenterOfMassInit(0.0, 0.0, 0.0);

    for (auto& rod : allRods) totalCenterOfMassInit += rod->centerOfMass();

    totalCenterOfMassInit = totalCenterOfMassInit/allRods.size();
    totalCenterOfMassInit.setY(0);
    std::cout << "initial position: " << totalCenterOfMassInit  << std::endl;

#ifdef TEST_SINGLE_MODULE
    simulation.run(800000);
    // to delimit debug data
    std::cout << "è,";
#else
    // run the simulation for 20s in background
    simulation.run(20000);
#endif

    allRods = myModel->find<tgBaseRigid>("rod");
    btVector3 totalCenterOfMassFinal(0.0, 0.0, 0.0);

    for (auto& rod : allRods) totalCenterOfMassFinal += rod->centerOfMass();

    totalCenterOfMassFinal = totalCenterOfMassFinal/allRods.size();
    totalCenterOfMassFinal.setY(0);

    // compute the total distance travelled by the robot during the simulation
    double distance = (totalCenterOfMassFinal-totalCenterOfMassInit).length();

    // depending on the requirements the fitness can assume different values
    // for current purpose we consider it equal to travelled distance
    double fitness = distance;

    std::cout << "final position: " << totalCenterOfMassFinal  << std::endl;
    std::cout << "distance: " << distance  << std::endl;
    std::cout << "fitness: " << fitness  << std::endl;

    // exit here is necessary to avoid other
    // components to write on the standard output
    exit(0);
}
