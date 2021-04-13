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
    robotController* pMuscleControl = NULL;

    const int offset = 4;
    if (argc > offset)
    {
        // create robot from command line arguments
        // Note: the first two arguments regards noise config,
        // and are not needed to build the robot
        rbModel = new robotModel(argc - offset, argv + offset);

        // noise related parameters
        int noiseType = atoi(argv[offset - 3]);
        double noiseLevel = atof(argv[offset - 2]);
        int seed = atoi(argv[offset - 1]);
        std::cout << noiseType << " - " << noiseLevel << " - " << seed << std::endl;
        
        pMuscleControl = new robotController(noiseType, noiseLevel, seed);
    }
    else
    {
        std::cerr << "Wrong program usage. Review which parameters can be used." << std::endl;
        exit(2);
    }

    simulation.addModel(rbModel);

    // run the simulation for three seconds to initialize the robot
    // and reduce construction/rotation/falling effects on the fitness
    simulation.run(3000);

    // get initial positions of first module's faces of interest
    btVector3 f_face_init_pos(0.0,0.0,0.0);
    btVector3 p_face_init_pos(0.0,0.0,0.0);
    rbModel->getFirstModulePosition(f_face_init_pos, p_face_init_pos);

    // attach robot controller
    rbModel->attach(pMuscleControl);

    // run the simulation for 10s in background
    simulation.run(10000);

    // get final position of first module's faces of interest
    btVector3 f_face_final_pos(0.0,0.0,0.0);
    btVector3 p_face_final_pos(0.0,0.0,0.0);
    rbModel->getFirstModulePosition(f_face_final_pos, p_face_final_pos);

    //determine if the robot moves forward or backward
    btVector3 first_module_dir = f_face_init_pos-p_face_init_pos;

    btVector3 line_dir(0.0,0.0,0.0);
    line_dir.setX(first_module_dir.getZ());
    line_dir.setZ(-first_module_dir.getX());

    double q = p_face_init_pos.getZ() - (line_dir.getZ()/line_dir.getX())*p_face_init_pos.getX();

    bool init_plane = f_face_init_pos.getZ() >= (line_dir.getZ()/line_dir.getX())*f_face_init_pos.getX()+q;
    bool final_plane = p_face_final_pos.getZ() >= (line_dir.getZ()/line_dir.getX())*p_face_final_pos.getX()+q;

    int direction = init_plane == final_plane ? 1 : -1;

    std::cout << "direction: " << direction << std::endl;

    // exit here is necessary to avoid other
    // components to write on the standard output
    exit(0);
}
