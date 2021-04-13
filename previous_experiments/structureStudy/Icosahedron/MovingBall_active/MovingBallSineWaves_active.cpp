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
 * @file MovingBallSineWaves.cpp
 * @brief Contains the implementation of class MovingBallSineWaves
 * @author Jean Marc Bejjani
 * @version 1.0.0
 * $Id$
 */

// This module
#include "MovingBallSineWaves_active.h"

// Its subject
#include "MovingBallModel_active.h"

// NTRTSim
#include "core/tgBasicActuator.h"
#include "controllers/tgImpedanceController.h"
#include "tgcreator/tgUtil.h"

MovingBallSineWaves::MovingBallSineWaves() :
    in_controller(new tgImpedanceController(500, 6130, 200)),
    segments(1.0),
    //insideLength(16.5),
    //outsideLength(19.5),
    insideLength(sqrt(2)*7.5),
    simTime(0.0),
    cycle(0.0),
    target(0.0)
{

}

MovingBallSineWaves::~MovingBallSineWaves()
{
	delete in_controller;
	//delete out_controller;
}

void MovingBallSineWaves::applyImpedanceControlInside(const std::vector<tgBasicActuator*> stringList, double dt ,
		std::size_t unit, double frequency, double amplitude, double phase)
{
    for(std::size_t i = 0; i < stringList.size(); i++)
    {
    	//function for sinus wave
    	cycle = sin(simTime * frequency * 2 * M_PI  + phase) +0.5;
    	//target = 0.001*cycle*amplitude + insideLength;

    	//function for castle rim wave

    	//double var=0;
    	//var = (simTime+phase)*frequency;
    	//cycle = (var - floor(var));



    	target = insideLength*(1-cycle*amplitude);

        double setTension = in_controller->control(*(stringList[i]),
                                            dt,
											target , 0
                                            );
        //std::cout << target << std::endl;
        #if (0) // Conditional compile for verbose control
        std::cout << "Inside String " << i << " tension " << setTension
        << " act tension " << stringList[i]->getMuscle()->getTension()
        << " length " << stringList[i]->getMuscle()->getActualLength() << std::endl;
        #endif
    }    
}
/*
void MovingBallSineWaves::applyImpedanceControlOutside(const std::vector<tgBasicActuator*> stringList,
                                                            double dt,
                                                            std::size_t phase)
{
    for(std::size_t i = 0; i < stringList.size(); i++)
    {
        cycle = sin(simTime * cpgFrequency + 2 * bodyWaves * M_PI * i / (segments) + phaseOffsets[phase]);
        target = offsetSpeed + cycle*cpgAmplitude;
        
        double setTension = out_controller->control(*(stringList[i]),
                                            dt,
                                            outsideLength,
                                            target
                                            );
        #if(0) // Conditional compile for verbose control
        std::cout << "Outside String " << i << " com tension " << setTension
        << " act tension " << stringList[i]->getMuscle()->getTension()
        << " length " << stringList[i]->getMuscle()->getActualLength() << std::endl;
        #endif
    }    
}
*/
void MovingBallSineWaves::onStep(MovingBallModel& subject, double dt)
{
    simTime += dt;
    
    segments = subject.m_moduleData.size();
    const std::vector<tgBasicActuator*>& allActuators = subject.getActuators("motor");

    for(std::size_t i=0 ; i<segments ; i++)
    {
    	std::vector<tgBasicActuator*>::const_iterator first = allActuators.begin() + 6*i;
    	std::vector<tgBasicActuator*>::const_iterator last = allActuators.begin() + 6*i+5;
    	std::vector<tgBasicActuator*> segmentActuators(first, last);
    	applyImpedanceControlInside( segmentActuators , dt, i, subject.m_moduleData[i].frequency,
    			subject.m_moduleData[i].amplitude, subject.m_moduleData[i].phase);
    }



    
}

