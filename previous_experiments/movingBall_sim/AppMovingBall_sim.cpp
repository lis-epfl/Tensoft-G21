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

// This application
#include "core/terrain/tgBoxGround.h"
#include "core/terrain/tgHillyGround.h"
#include "core/tgModel.h"
#include "core/tgSimViewGraphics.h"
#include "core/tgSimulation.h"
#include "core/tgWorld.h"
// The C++ Standard Library
#include <iostream>
#include <fstream>

#include "MovingBallModel_sim.h"
#include "Obstacles/ObstacleModel.h"
#include "Obstacles/ObstacleModel3.h"
#include "Obstacles/tgCraterDeep.h"

#include "MovingBallSineWaves_sim.h"
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



    const double yaw = 0.0;
	//const double pitch = M_PI/15.0;
	const double pitch = 0.0;
	const double roll = 0.0;

	const tgBoxGround::Config groundConfig(btVector3(yaw, pitch, roll));

	const tgHillyGround::Config hillyConfig(btVector3(yaw, pitch, roll),1,0,
			btVector3(500.0, 1.5, 500.0), btVector3(0,0,0), 100, 100, 0.05, 10, 5, 0);

	// the world will delete this
	tgBoxGround* ground = new tgBoxGround(groundConfig);

	tgHillyGround* hillyground = new tgHillyGround(hillyConfig);


    //log configuration
	std::string log_filename = "./NTRTsim-master/src/dev/log";
	double samplingTimeInterval = 0.1;
	tgDataLogger2* myDataLogger = new tgDataLogger2(log_filename, samplingTimeInterval);

       const tgWorld::Config config(98.1);
     //const tgWorld::Config config(0.09810);
	 //const tgWorld::Config config(981);
    // Second create the view



       tgWorld world(config, ground);
       //tgWorld world(config, hillyground);


	// Second create the view
	const double timestep_physics = 0.001; // Seconds
	const double timestep_graphics = 1.f/60.f; // Seconds
	tgSimViewGraphics view(world, timestep_physics, timestep_graphics);
    MovingBallModel* myModel = NULL;
    // Third create the simulation
    tgSimulation simulation(view);

    if(argc>1 && *argv[1]!='s')
    {
	
	myModel = new MovingBallModel(argc, argv);
	//std::cout << inputStr << std::endl;	
    }
    else
	{
		myModel = new MovingBallModel();
	}
    // Fourth create the models with their controllers and add the models to the
    // simulation
    
    ObstacleModel3* Obstacle = new ObstacleModel3();

	

    MovingBallSineWaves* const pMuscleControl =
      new MovingBallSineWaves();
    myModel->attach(pMuscleControl);
    simulation.addModel(myModel);
    //simulation.addObstacle(Obstacle);
    
    //attach logging
    /* myDataLogger->addSenseable(myModel);
    tgRodSensorInfo* myRodSensorInfo = new tgRodSensorInfo();
    tgSpringCableActuatorSensorInfo* mySCASensorInfo = new tgSpringCableActuatorSensorInfo();
    myDataLogger->addSensorInfo(myRodSensorInfo);
    myDataLogger->addSensorInfo(mySCASensorInfo);
    simulation.addDataManager(myDataLogger);
 */
	// Run until the user stops


    std::vector<tgBaseRigid*> allRods = myModel->find<tgBaseRigid>("rod");

	btVector3 totalCenterOfMassInit(0,0,0);
	for (int j=0; j<allRods.size();j++)
	{
		totalCenterOfMassInit+=allRods[j]->centerOfMass();
	}

	totalCenterOfMassInit = totalCenterOfMassInit/allRods.size();

	totalCenterOfMassInit.setY(0);


	std::cout << "initial position: " << totalCenterOfMassInit  << std::endl;


	if(argc>1)
	{
		if(*argv[1]=='s')
		{
			simulation.run();
		}
			
	}

	simulation.limitRun(20000);
	
    	

    allRods = myModel->find<tgBaseRigid>("rod");
    btVector3 minPositionX=allRods[0]->centerOfMass();
    btVector3 maxPositionX=allRods[0]->centerOfMass();

	btVector3 totalCenterOfMassFinal(0,0,0);
	for (int j=0; j<allRods.size();j++)
	{
		if(allRods[j]->centerOfMass().getX()<minPositionX.getX())
		{
			minPositionX=allRods[j]->centerOfMass();
		}
		if(allRods[j]->centerOfMass().getX()>maxPositionX.getX())
		{
			maxPositionX=allRods[j]->centerOfMass();
		}
		//std::cout << "rod position: " << allRods[j]->centerOfMass()  << std::endl;
		totalCenterOfMassFinal+=allRods[j]->centerOfMass();
	}

	totalCenterOfMassFinal = totalCenterOfMassFinal/allRods.size();

	totalCenterOfMassFinal.setY(0);

	double distance = (totalCenterOfMassFinal-totalCenterOfMassInit).length();

	double X = totalCenterOfMassFinal.getX();

	double Y = totalCenterOfMassFinal.getZ();

	double fitness = 0;

	/*if(X<0)
	{
		fitness = 1/(X*X+1);
	}
	else
	{
		fitness = sqrt(X*X);
	}*/

	fitness=distance;

	std::cout << "final position: " << totalCenterOfMassFinal  << std::endl;

	/*std::cout << "min position: " << minPositionX  << std::endl;

	std::cout << "max position: " << maxPositionX  << std::endl;*/

	std::cout << "distance: " << distance  << std::endl;

	std::cout << "fitness: " << fitness  << std::endl;


	std::ofstream outFile;

    outFile.open("./NTRTsim-master/src/dev/MovingBall_sim/output_sim.txt");

	outFile << fitness;

	outFile.close();

	exit(1);


}
