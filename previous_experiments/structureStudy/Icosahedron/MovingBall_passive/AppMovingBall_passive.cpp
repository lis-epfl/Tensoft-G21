
/**
 * @file AppMovingBall_passive.cpp
 * @brief Contains the definition function main() for the passive behavior
 *
 * @author Jean Marc Bejjani
 *
 *
 */

// This application
#include "MovingBallModel_passive.h"
#include "MovingBallSineWaves_passive.h"
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

#include <unistd.h>



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
	// the world will delete this
	tgBoxGround* ground = new tgBoxGround(groundConfig);

	//log configuration

	double samplingTimeInterval = 0.01;


       //const tgWorld::Config config(98.1);
     const tgWorld::Config config(98.10);
	 //const tgWorld::Config config(0.0981);
    // Second create the view
        tgWorld world(config, ground);

	// Second create the view
	const double timestep_physics = 0.0001; // Seconds
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


	std::ofstream outFile;

	outFile.open("./Tensoft/src/dev/StructureStudy/Icosahedron/data0-5.csv");

	double initH=0;

    int loop=0;
    for(loop=150;loop<160;loop++)
    {
    	std::cout << loop << std::endl;
    	tgSimulation simulation(view);

    	std::ostringstream convert;   // stream used for the conversion

    	convert << loop;

    	std::string log_filename = "./Tensoft/src/dev/StructureStudy/Icosahedron/passive3/"+ convert.str();

    	tgDataLogger2* myDataLogger = new tgDataLogger2(log_filename, samplingTimeInterval);
    	tgRodSensorInfo* myRodSensorInfo = new tgRodSensorInfo();
    	tgSpringCableActuatorSensorInfo* mySCASensorInfo = new tgSpringCableActuatorSensorInfo();

    	MovingBallModel* myModel = new MovingBallModel(0.5*loop);

    	simulation.addModel(myModel);



    	myDataLogger->addSenseable(myModel);
    	myDataLogger->addSensorInfo(myRodSensorInfo);
		myDataLogger->addSensorInfo(mySCASensorInfo);
		simulation.addDataManager(myDataLogger);

    	MovingBallSineWaves* const pMuscleControl =
    			new MovingBallSineWaves();
		//myModel->attach(pMuscleControl);

    	simulation.run();
    	simulation.limitRun(15000);

	std::vector<tgSphere*> upSensorRods = myModel->find<tgSphere>("up sensor");
    	std::vector<tgSphere*> downSensorRods = myModel->find<tgSphere>("down sensor");
    	std::vector<tgBasicActuator*> TensionCables = myModel->find<tgBasicActuator>("custom muscle");



    	btVector3 upPosition=btVector3(0,0,0);
    	btVector3 downPosition=btVector3(0,0,0);
    	double totalTension=0;


    	for(int i=0;i<upSensorRods.size();i++)
    	{
    		upPosition+=upSensorRods[i]->centerOfMass();
    		downPosition+=downSensorRods[i]->centerOfMass();
    		totalTension+=TensionCables[i]->getTension();
    	}
    	upPosition=upPosition/upSensorRods.size();
    	downPosition=downPosition/downSensorRods.size();

	std::cout<<"the top sensor is:" << upPosition<<std::endl;
	std::cout<<"the down sensor is:" << downPosition<<std::endl;

    	if(loop==0)
    	{
    		initH=upPosition.getY() - downPosition.getY();
    	}

    	double strain = (1-((upPosition.getY() - downPosition.getY())/initH))*100;
    	std::cout<<strain<<std::endl;

    	std::cout<<totalTension<<std::endl;

    	outFile << strain << " " << totalTension/1000 << "\n";

    	//usleep(1000 * 1000);

    	myModel->notifyTeardown();

    	myModel->teardown();
    	myDataLogger->teardown();

    	simulation.teardown();

    	//delete myModel;
    	//myDataLogger->teardown();
    	//delete myDataLogger;
    	//delete myRodSensorInfo;
    	//delete mySCASensorInfo;



    }
	// Run until the user stops

    //simulation.limitRun(10000, 200);



    //Teardown is handled by delete, so that should be automatic
    outFile.close();
    exit(1);
}
