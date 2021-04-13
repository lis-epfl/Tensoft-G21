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
 * @file robotTrajectory.cpp
 * @brief Simulation application to record on which trajectory the robot moves
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
int main(int argc, char** argv)
{
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

    // the world will delete this
    tgBoxGround* ground = new tgBoxGround(groundConfig);
    tgHillyGround* hillyground = new tgHillyGround(hillyConfig);

    const tgWorld::Config config(98.1);

    tgWorld world(config, ground);

    // Second create the view
    const double timestep_physics = 0.001; // Seconds
    const double timestep_graphics = 1.f/60.f; // Seconds

    // tgSimView runs the simulation headless
    tgSimView view(world, timestep_physics, timestep_graphics);

    // Third create the simulation
    tgSimulation simulation(view);

    // create the robot model either from a conf file or from inline values
    robotModel* myModel = NULL;
    std::string trajectory_file = "";
    if (argc >= 5) {
        // from conf files
        if (std::string(argv[1]) == "file") {
            // value 1 -> inform the model to read conf from file
            // advance argv of 2 to point to the conf filepath
            myModel = new robotModel(1, argv + 2);

            // in case also noise values are given (after filepath)
            trajectory_file = std::string(argv[3]);
            if (argc == 7) {
                int noiseType = atoi(argv[4]);
                double noiseLevel = atof(argv[5]);
                int seed = atoi(argv[6]);
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
                trajectory_file = std::string(argv[2]);
                int noiseType = atoi(argv[3]);
                double noiseLevel = atof(argv[4]);
                int seed = atoi(argv[5]);

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

    simulation.addModel(myModel);

    // run the simulation for three seconds to initialize the robot
    simulation.run(3000);

    // gather the initial trajectory positions
    std::vector<tgBaseRigid*> allRods = myModel->find<tgBaseRigid>("rod");
    std::vector<tgBaseRigid*> actuatorRods = myModel->find<tgBaseRigid>("r0");

    btVector3 first_com = actuatorRods[0]->centerOfMass();
    btVector3 last_com = actuatorRods[actuatorRods.size()-1]->centerOfMass();

    // initialize center of mass of the robot
    btVector3 totalCenterOfMassInit(0.0, 0.0, 0.0);

    for (auto& rod : allRods) totalCenterOfMassInit += rod->centerOfMass();

    totalCenterOfMassInit = totalCenterOfMassInit/allRods.size();
    totalCenterOfMassInit.setY(0);
    std::cout << "initial position: " << totalCenterOfMassInit  << std::endl;

    if (trajectory_file == "") {
        trajectory_file = "trajectories/trajectory.csv";
    }

    std::ofstream ofs(trajectory_file, std::ofstream::out);
    ofs << "X,Z,x0,z0,xM,zM\n";
    ofs << totalCenterOfMassInit.getX() << "," << totalCenterOfMassInit.getZ() << ","
        << first_com.getX() << "," << first_com.getZ() << ","
        << last_com.getX() << "," << last_com.getZ() << "\n";

    // advance the simulation of a second for 20 seconds in total
    for (int i = 0; i < 20; i++) {
        simulation.run(1000);

        // record current robot position
        btVector3 robotCenterOfMass(0.0, 0.0, 0.0);
        allRods = myModel->find<tgBaseRigid>("rod");
        actuatorRods = myModel->find<tgBaseRigid>("r0");

        for (auto& rod : allRods) robotCenterOfMass += rod->centerOfMass();
        robotCenterOfMass /= allRods.size();

        // center of mass of first and last modules' actuator
        first_com = actuatorRods[0]->centerOfMass();
        last_com = actuatorRods[actuatorRods.size()-1]->centerOfMass();

        ofs << robotCenterOfMass.getX() << "," << robotCenterOfMass.getZ() << ","
            << first_com.getX() << "," << first_com.getZ() << ","
            << last_com.getX() << "," << last_com.getZ() << "\n";
    }
    ofs << std::endl;
    ofs.close();

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
