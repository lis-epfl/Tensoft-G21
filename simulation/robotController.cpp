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
 * @author Jean Marc Bejjani, Daniele Bissoli
 * @copyright Copyright (C) 2019 LIS
 * $Id$
 */

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/uniform_real_distribution.hpp>

// NTRT library
#include "ntrt/core/tgBasicActuator.h"
#include "ntrt/controllers/tgImpedanceController.h"
#include "ntrt/controllers/tgBasicController.h"
#include "ntrt/tgcreator/tgUtil.h"

#include "robotModel.h"
#include "robotController.h"


robotController::robotController(int noiseType, double noiseLevel, int seed)
    : in_controller(new tgImpedanceController(1000, 6130, 0)),
      segments(1.0),
      insideLength(4.5),
      simTime(0.0),
      cycle(0.0),
      target(0.0),
      noiseType(noiseType),
      noiseLevel(noiseLevel),
      seed(seed)
{
    rng.seed(seed);
//    Ks = { 1.0, 1.0, 1.0, 1.0, 1.0 };
// TARGET: 0.4
    Ks = {
        1.5350773654170244,
        1.1682581291061436,
        1.3630111209665405,
        1.3603623969250653,
        1.142116164534608
    }; // best
}
robotController::robotController(int noiseType, double noiseLevel, int seed, std::vector<double>& Ks)
    : in_controller(new tgImpedanceController(1000, 6130, 0)),
      segments(1.0),
      insideLength(4.5),
      simTime(0.0),
      cycle(0.0),
      target(0.0),
      noiseType(noiseType),
      noiseLevel(noiseLevel),
      seed(seed),
      Ks(Ks)
{
    rng.seed(seed);
}

robotController::~robotController()
{
    delete in_controller;
}

void robotController::applyImpedanceControlInside(const std::vector<tgBasicActuator*> stringList, double dt ,
                                                  std::size_t unit, double frequency, double amplitude, double phase)
{

#ifndef SIN_CONTROLLER
    // cycle [0-180]
    cycle = 90 * (sin(simTime * frequency * 2 * M_PI  + phase) + 1.0);

    #ifdef STEP_INPUT
        // this inference is needed only when tests on single module are performed
        // keep the same command for STEP_SIZE (10000 ms that is 10 seconds)
        nn.neuralNet_commands(inputs[int(simTime * 1000) / STEP_SIZE], commands);
        std::cout << "è," << inputs[int(simTime * 1000) / STEP_SIZE];
    #else
        #ifdef DEBUG_INFO
            std::cout << "è," << cycle;
        #endif

        // regular commands (desired cable length) computation
        nn.neuralNet_commands(cycle, commands);
    #endif
#endif

    for(std::size_t i = 0; i < numActuatedCables; i++)
    {
        double noise;
        switch (noiseType)
        {
            case 0: {
                // no noise added (explicitly added for noiseless experiments)
                noise = 0.0;
                break;
            }
            case 1: {
                // gaussian noise added
                boost::random::normal_distribution<> normal_dist(0.0, 1.0);
                noise = normal_dist(rng) * noiseLevel;
                break;
            }
            case 2: {
                // uniform noise added
                boost::random::uniform_real_distribution<> uni_dist(-1.0, 1.0);
                noise = uni_dist(rng) * noiseLevel;
                break;
            }
            default:
                // wrong type values are ignored and no noise is added
                noise = 0.0;
        }
    #ifdef SIN_CONTROLLER
        cycle = sin(simTime * frequency * 2 * M_PI  + phase) + 0.5;
        target = insideLength * (1 - amplitude * (cycle + noise));
    #else
    //    target = AL[i] - (maxCableFraction[i] - commands[i]) * 100 + noise;
        target = AL[i] - (maxCableFraction[i] - commands[i]) * 100 * Ks[i] + noise;
    #endif
    #ifndef STATIC
        double setTension = in_controller->control(*(stringList[i]), dt, target, 0);
    #endif

    #ifdef DEBUG_INFO
        #ifndef COM_COLLECTION
            std::cout << "," << stringList[i]->getCurrentLength();
        #endif
    #endif
    }
}

void robotController::onStep(robotModel& subject, double dt)
{
    simTime += dt;
    segments = subject.m_moduleData.size();

    #ifndef NO_ACTIVE_CABLES
        const std::vector<tgBasicActuator*>& allActuators = subject.getActuators("motor");

        // cycle over all the modules of the robot
        for (std::size_t i = 0; i < segments; i++)
        {
            std::vector<tgBasicActuator*>::const_iterator first = allActuators.begin() + numActuatedCables*i;
            std::vector<tgBasicActuator*>::const_iterator last = allActuators.begin() + numActuatedCables*i+numActuatedCables;
            std::vector<tgBasicActuator*> segmentActuators(first, last);

            applyImpedanceControlInside(segmentActuators, dt, i,
                                        subject.m_moduleData[i].frequency,
                                        subject.m_moduleData[i].amplitude,
                                        subject.m_moduleData[i].phase);
        }
    #else
        // print always 0 as time to notify that no compression is performed
        std::cout << "è,0";
    #endif

    #ifdef COM_COLLECTION
        std::vector<tgBaseRigid*> allRods = subject.find<tgBaseRigid>("rod");

        for (auto& rod : allRods) {
            auto com = rod->centerOfMass();
//            auto com = rod->getPRigidBody()->getCenterOfMassPosition();
            std::cout << "," << com[0] << ";" << com[1] << ";" << com[2];
        }
    #endif
}

void robotController::onTeardown(robotModel& subject) {
    segments = 1.0;
    simTime = cycle = target = 0.0;
    rng.seed(seed);
}

