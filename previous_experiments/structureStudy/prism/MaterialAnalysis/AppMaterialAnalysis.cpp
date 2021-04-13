
/**
 * @file AppMovingBall_passive.cpp
 * @brief Contains the definition function main() for the passive behavior
 *
 * @author Jean Marc Bejjani
 *
 *
 */

// This application
#include "MaterialAnalysisModel.h"

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
	const double timestep_physics = 0.001; // Seconds
	const double timestep_graphics = 1.f/60.f; // Seconds
	tgSimViewGraphics view(world, timestep_physics, timestep_graphics);

    // Third create the simulation


    // Fourth create the models with their controllers and add the models to the
    // simulation
    //MaterialAnalysisModel* myModel = new MaterialAnalysisModel();
    /*MovingBallSineWaves* const pMuscleControl =
      new MovingBallSineWaves();
    myModel->attach(pMuscleControl);*/
    //simulation.addModel(myModel);

    //attach logging



    int loop=0;
    for(loop=0;loop<1500;loop++)
    {
    	std::cout << loop << std::endl;
    	tgSimulation simulation(view);

    	std::ostringstream convert;   // stream used for the conversion

    	convert << loop;

    	std::string log_filename = "./NTRTsim-master/src/dev/StructureStudy/prism/MaterialAnalysis3/"+ convert.str();

    	tgDataLogger2* myDataLogger = new tgDataLogger2(log_filename, samplingTimeInterval);
    	tgRodSensorInfo* myRodSensorInfo = new tgRodSensorInfo();
    	tgSpringCableActuatorSensorInfo* mySCASensorInfo = new tgSpringCableActuatorSensorInfo();

    	MaterialAnalysisModel* myModel = new MaterialAnalysisModel(1*loop);

    	simulation.addModel(myModel);



    	myDataLogger->addSenseable(myModel);
    	myDataLogger->addSensorInfo(myRodSensorInfo);
		myDataLogger->addSensorInfo(mySCASensorInfo);
		simulation.addDataManager(myDataLogger);

    	//MovingBallSineWaves* const pMuscleControl =
    		//	new MovingBallSineWaves();
		//myModel->attach(pMuscleControl);

    	//simulation.run();
    	simulation.limitRun(10);

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
    return 0;
}
