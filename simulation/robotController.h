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

#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

/**
 * @file MovingBallSineWaves.h
 * @brief Contains the definition of class MovingBallSineWaves
 * @author Brian Tietz
 * @version 1.0.0
 * $Id$
 */

// The C++ Standard Library
#include <vector>

#include <boost/random.hpp>
#include <boost/cstdint.hpp>
#include <boost/nondet_random.hpp>

// NTRT library
#include "ntrt/core/tgObserver.h"

#include "neuralNet.h"

// Forward Declarations
class tgImpedanceController;
class tgBasicActuator;
class robotModel;

/**
 * Control the MovingBallModel with a series of sine waves
 * and local impedance controllers
 */
class robotController : public tgObserver<robotModel>
{
public:

    /**
     * Construct the controller. Typically occurs in the main function.
     * The controller will need to be attached to a subject (model)
     * Parameters are currently set in the initalizer lists.
     */
    robotController(int noiseType = 0, double noiseLevel = 0.0, int seed = 42);

    /* Special constructor needed for evolving K constants that modify cables compression */
    robotController(int noiseType, double noiseLevel, int seed, std::vector<double>& Ks);

    /**
     * Destructor. Frees the tgImpedanceController pointers
     */
    ~robotController();

    /**
     * Applies the impedance controllers using a velocity setpoint determined
     * by the frequency, amplitude and phase parameters.
     * Called during this classes onStep function.
     * @param[in] stringList a std::vector of strings taken from the
     * subject's MuscleMap
     * @param[in] dt - a timestep. Must be positive.
     * @param[in] frequency
     * @param[in] amplitude
     * @param[in] phase - reads the index out of the phaseOffsets vector
     */
    void applyImpedanceControlInside(const std::vector<tgBasicActuator*> stringList,
                                    double dt, std::size_t unit, double frequency, double amplitude, double phase);

    /**
     * Apply the sineWave controller. Called my notifyStep(dt) of its
     * subject. Calls the applyImpedanceControl functions of this class
     * @param[in] subject - the MovingBallModel that is being
     * Subject must have a MuscleMap populated
     * @param[in] dt, current timestep must be positive
     */
    virtual void onStep(robotModel& subject, double dt);

    /**
     * Reset class members
     * @param subject
     */
    virtual void onTeardown(robotModel& subject);

protected:
    /**
     * Pointers to impedance controllers
     */
    tgImpedanceController* in_controller;

    std::size_t segments;

    /**
     * Muscle Length Parameters
     */
    const double insideLength = 4.5;

    /**
     * Parameters that consolidate the sine wave computations within
     * the update code. Cycle deals with sin(theta), and target handles
     * offset + amplitude * cycle
     */
    double simTime;
    double cycle;
    double target;

    /**
     * Parameters related to the noise introduced in the simulation
     */
    const int noiseType;
    const double noiseLevel;

    /**
    * Simulation seed
    */
    const int seed;

    /**
     * Array employed to store neural network output. Each value represents
     * the compression needed to achieve by each associated active cable.
     */
    double commands[5] = {0.0, 0.0, 0.0, 0.0, 0.0};


#ifdef STEP_INPUT
    const int STEP_SIZE = 10000;
    /**
     * Values of input for the neural network required to simulate the
     * Tensegrity module state transition from relaxed to compressed situation and backward
     */
    #ifdef EVOLVE_MODULE
        /* Note: inputs values are placed in the following manner to reduce
           module motion that could affect the measurements:
            - 0 <= input <= 4 -> module stabilization
            - 5 <= input <= 7 -> collection of COMs at initial position
            - 8 <= input <= 10 -> transitional values from initial to final position
            - 11 <= input <= 13 -> collection of COMs at final position
            - input == 14 -> border values not considered
        */
        double inputs[15] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 45.0,
                             135.0, 180.0, 180.0, 180.0, 180.0, 180.0};
    #else
        double inputs[80] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0,
                         40.0, 45.0, 50.0, 55.0, 60.0, 65.0, 70.0, 75.0, 80.0, 85.0, 90.0, 95.0,
                         100.0, 105.0, 110.0, 115.0, 120.0, 125.0, 130.0, 135.0, 140.0, 145.0, 150.0,
                         155.0, 160.0, 165.0, 170.0, 175.0, 180.0, 180.0, 175.0, 170.0, 165.0, 160.0,
                         155.0, 150.0, 145.0, 140.0, 135.0, 130.0, 125.0, 120.0, 115.0, 110.0, 105.0,
                         100.0, 95.0, 90.0, 85.0, 80.0, 75.0, 70.0, 65.0, 60.0, 55.0, 50.0, 45.0,
                         40.0, 35.0, 30.0, 25.0, 20.0, 15.0, 10.0, 5.0, 0.0, 0.0};
    #endif
#endif

    const int numActuatedCables = 5;

    /**
     * Small multiplicative constants that serve to increment max module compression
     */
    std::vector<double> Ks;

    const double maxCableFraction[5] = { 0.0492627, 0.0445782, 0.0441734, 0.0490044, 0.0493764 };

    /**
    * Length [cm] of the internal active cable (connected to the actuator) once the module is stable
    * according to neural network output
        // double initCableLengths[5] = {
        //     4.3605380185858555,
        //     4.925254734596728,
        //     4.725783970853703,
        //     4.5755071487974535,
        //     4.5373592568218015
        // };
    */

    /*
     * After stretch Length: initial internal cables length once robots modules have stabilized
     *
     *  collected cables length at rest with no active cable pre-stretching
     *  and passive cable pre-stretching set at 65% of maximum stiffness
     */
    double AL[5] = {
        3.7924985751515052,
        4.5968806782932745,
        4.165038304316172,
        4.294122628208502,
        4.432296027591343
    };


    std::time_t now = std::time(0);
    boost::random::mt19937 rng{(boost::uint32_t)(now)};

    /* Neural Network */
    neuralNet nn;
};

#endif // ROBOT_CONTROLLER_H
