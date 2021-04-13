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
#include <iomanip>
#include <fstream>
#include <cstdlib>
#include <string.h>

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
#include "robotControllerGR.h"
#include "targetModel.h"

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

    // the world will delete this
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

    // create the robot model from given inline config and the target model
    robotModel* rbModel = NULL;
    targetModel* tModel = NULL;

    // init time in milliseconds
    long init_time = 3000;

    // simulation time in seconds
    long sim_time;

    // distance and bearing
    double distance, bearing;

#ifndef CONTROLLER_INFO
    const int offset = 9;
#else
    const int offset = 10;
#endif

    if (argc > offset) {
        // create robot from command line arguments
        // Note: the first two arguments regards noise config,
        // and are not needed to build the robot
        rbModel = new robotModel(argc - offset, argv + offset);

        // simulation time in seconds
        sim_time = std::stol(argv[1]);

        // noise related parameters
        int noiseType = atoi(argv[2]);
        double noiseLevel = atof(argv[3]);
        int seed = atoi(argv[4]);
        std::cout << noiseType << " - " << noiseLevel << " - " << seed << std::endl;

        // controller path
        char* controller_path = argv[5];
        std::cout << controller_path << std::endl;

        // movement direction
        rbModel->setMovDir(atoi(argv[6]));

        // target distance and bearing
        distance = atof(argv[7]);
        bearing = atof(argv[8]);

        tModel = new targetModel();

#ifndef CONTROLLER_INFO
        robotControllerGR* const pMuscleControl =
                new robotControllerGR(controller_path, tModel, init_time, noiseType, noiseLevel, seed);
#else
        char* log_file_path = argv[9];

        robotControllerGR* const pMuscleControl =
                new robotControllerGR(controller_path, tModel, init_time, noiseType, noiseLevel, seed, log_file_path);
#endif
        rbModel->attach(pMuscleControl);
    } else {
        std::cerr << "Wrong program usage. Review which parameters can be used." << std::endl;
        exit(2);
    }

    // add robot model to the simulation
    simulation.addModel(rbModel);

    // run the simulation for three seconds to initialize the robot
    // and reduce construction/rotation/falling effects on the fitness
    simulation.run(init_time);

    // get robot initial position
    btVector3 rbInitPos(0.0, 0.0, 0.0);
    rbModel->getCurrentPosition(rbInitPos);

    // set target position
    btVector3 targetPos(0.0, 0.0, 0.0);
    rbModel->getCoordinatesFromDistAndBearing(distance, bearing, targetPos);
    tModel->setPosition(targetPos);

    // add target model to the simulation
    simulation.addModel(tModel);

    // compute the initial distance wrt the target
    double initial_dist, init_bearing;
    rbModel->getDistAndBearingToGoal(targetPos, initial_dist, init_bearing);

    // run the simulation in background for sim_time seconds
    simulation.run(sim_time*1000);

    // compute the final distance wrt the target
    double final_dist, final_bearing;
    rbModel->getDistAndBearingToGoal(targetPos, final_dist, final_bearing);

    // depending on the requirements the fitness can assume different values
    // for current purpose we consider it equal to the final distance
    double fitness = final_dist;

    // original precision
    std::streamsize precision = std::cout.precision();

    std::cout << "initial distance: " << initial_dist  << std::endl;
    std::cout << "initial bearing: " << init_bearing  << std::endl;
    std::cout << "robot initial position: " << std::setprecision(20) << rbInitPos.getX() << " " << rbInitPos.getZ() << std::endl;
    std::cout << "target position: " << targetPos.getX() << " " << targetPos.getZ() << std::endl;
    std::cout << "final distance: " << std::setprecision(precision) << final_dist  << std::endl;
    std::cout << "final bearing: " << final_bearing  << std::endl;
    std::cout << "fitness: " << fitness  << std::endl;

    // exit here is necessary to avoid other
    // components to write on the standard output
    exit(0);
}
