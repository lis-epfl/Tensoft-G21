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

#ifndef MOVING_BALL_MODEL_H
#define MOVING_BALL_MODEL_H

/**
 * @file MovingBallModel.h
 * @brief Contains the definition of class MovingBallModel
 * @author Brian Tietz
 * $Id$
 */

// This library
#include "core/tgModel.h" 
#include "core/tgSubject.h"
// This library
#include "core/tgCast.h"
#include "core/tgBasicActuator.h"
#include "core/tgRod.h"
#include "core/tgString.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgBasicActuatorInfo.h"
#include "tgcreator/tgRigidAutoCompound.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgSphereInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
#include "tgcreator/tgUtil.h"
// The Bullet Physics library
#include "btBulletDynamicsCommon.h"
// The C++ Standard Library
#include <map>
#include <set>
#include <string>
#include <vector>

// Forward declarations
class tgBasicActuator;

struct Module
{
	int order;
	std::vector<int> connected_modules;
	std::vector<int> connected_faces;
	int parent_face;

	btVector3 position;

	double frequency;
	double amplitude;
	double phase;
	int rotation;

} ;

/**
 * The MovingBallModel implements a tensegrity with rigid
 * tetrahedrons and six strings between segments, also known as
 * Tetraspine. 
 */
class MovingBallModel: public tgSubject<MovingBallModel>, public tgModel
{
public: 
	
	/**
	 * Used within this function to map segments to string keys
	 */
    typedef std::map<std::string, std::vector<tgBasicActuator*> > ActuatorMap;
	
	/**
	 * The only constructor. The model details are instantiated once
	 * setup is called typically when the model is added to a simulation.
	 * @param[in] segments, a positive integer dictating how many
	 * rigid segments will be constructed. 
	 */
    MovingBallModel();

    MovingBallModel(int loop);
	
	/**
	 * Nothing to do. Most functions already handled by tgModel::teardown
	 */
    virtual ~MovingBallModel()
    {}
    
    /**
     * Create the model. Place the rods and strings into the world
     * that is passed into the simulation. This is triggered
     * automatically when the model is added to the simulation, when
     * tgModel::setup(world) is called (if this model is a child),
     * and when reset is called. Also notifies controllers of setup.
     * @param[in] world - the world we're building into
     */
    virtual void setup(tgWorld& world);
	
	/**
     * Step the model, its children. Notifies controllers of step.
     * @param[in] dt, the timestep. Must be positive.
     */
    virtual void step(const double dt);
	
	/**
	 * Get a group of actuators according to the provided key. Groups are
	 * populated during setup.
	 * @param[in] key - a std:string* used as a key to a std::map from
	 * a string to a vector of actuators
	 * @return a std::vector of pointers to the actuators found by the key
	 */
    const std::vector<tgBasicActuator*>& getActuators (const std::string& key) const;
    
    /**
     * Return a std::size_t indicating the number of segments in the 
     * tetraSpine.
     * @return a std::size_t with the value of m_segments
     */
    std::vector<Module> m_moduleData;

private:
	
	/**
	 * A std::vector containing all of the tgBasicActuators amongst
	 * the children of this model. Populated during setup
	 */
    std::vector<tgBasicActuator*> allActuators;
	
	/**
	 * A typdef of std::map from std::string to tgBasicActuator*. Contains
	 * mapped actuators, populated during setup.
	 */
    ActuatorMap actuatorMap;
    
    double force;

    /**
     * The number of segments in the spine
     */
};

#endif
