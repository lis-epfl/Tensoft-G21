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
 * @file AppNestedTetrahedrons.cpp
 * @brief Contains the definition function main() for the Nested Tetrahedrons
 * application.
 * @author Jean Marc Bejjani
 * @copyright Copyright (C) 2014 NASA Ames Research Center
 * $Id$
 */

// This application
#include "MovingBallModel_active.h"
#include "MovingBallSineWaves_active.h"
// This library
#include "core/terrain/tgBoxGround.h"
#include "core/tgModel.h"
#include "core/tgSimViewGraphics.h"
#include "core/tgSimulation.h"
#include "core/tgWorld.h"
// The C++ Standard Library
#include <iostream>
#include <fstream>
//The data logging library
#include "sensors/tgDataLogger2.h"
#include "sensors/tgRodSensorInfo.h"
#include "sensors/tgSpringCableActuatorSensorInfo.h"


/**
 * The entry point.
 * @param[in] argc the number of command-line arguments
 * @param[in] argv argv[0] is the executable name
 * @return 0
 */
int main(int argc, char** argv)
{

    std::cout << "AppNestedStructureTest" << std::endl;

    const double yaw = 0.0;
	//const double pitch = M_PI/15.0;
	const double pitch = 0.0;
	const double roll = 0.0;
	const tgBoxGround::Config groundConfig(btVector3(yaw, pitch, roll));
	// the world will delete this
	tgBoxGround* ground = new tgBoxGround(groundConfig);

	//log configuration
	std::string log_filename = "/home/bejjani/Desktop/NTRTsim-master/src/dev/activeLog";
	double samplingTimeInterval = 0.01;


       //const tgWorld::Config config(98.1);
     const tgWorld::Config config(98.10);
	 //const tgWorld::Config config(0.0981);
    // Second create the view
        tgWorld world(config, ground);

	// Second create the view
	const double timestep_physics = 0.001; // Seconds
	const double timestep_graphics = 1.f/60.f; // Seconds
	tgSimViewGraphics view(world, timestep_physics, timestep_graphics);

    // Third create the simulation


    // Fourth create the models with their controllers and add the models to the
    // simulation
    //MovingBallModel* myModel = new MovingBallModel();
    /*MovingBallSineWaves* const pMuscleControl =
      new MovingBallSineWaves();
    myModel->attach(pMuscleControl);*/
    //simulation.addModel(myModel);

    //attach logging



    int loop=0;
    for(loop=100;loop<1000;loop++)
    {
    	tgSimulation simulation(view);

    	tgDataLogger2* myDataLogger = new tgDataLogger2(log_filename, samplingTimeInterval);
    	tgRodSensorInfo* myRodSensorInfo = new tgRodSensorInfo();
    	tgSpringCableActuatorSensorInfo* mySCASensorInfo = new tgSpringCableActuatorSensorInfo();

    	MovingBallModel* myModel = new MovingBallModel(loop);

    	MovingBallSineWaves* const pMuscleControl = new MovingBallSineWaves();
    	myModel->attach(pMuscleControl);


    	simulation.addModel(myModel);


    	myDataLogger->addSenseable(myModel);
    	myDataLogger->addSensorInfo(myRodSensorInfo);
		myDataLogger->addSensorInfo(mySCASensorInfo);
		simulation.addDataManager(myDataLogger);


		simulation.run();
/*
    	simulation.limitRun(5000, 200);

    	simulation.teardown();*/
    	//delete myModel;
    	//myDataLogger->teardown();
    	//delete myDataLogger;
    	//delete myRodSensorInfo;
    	//delete mySCASensorInfo;



    }
	// Run until the user stops

    //simulation.limitRun(10000, 200);



    //Teardown is handled by delete, so that should be automatic
    return 0;
}
