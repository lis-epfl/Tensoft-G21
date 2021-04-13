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

#ifndef TARGET_MODEL_H
#define TARGET_MODEL_H

/**
 * @author Enrico Zardini
 * $Id$
 */

#ifndef SIM_MODEL_H
#define SIM_MODEL_H

// The C++ Standard Library
#include <map>
#include <set>
#include <string>
#include <vector>

#include <Eigen/Dense>

// The Bullet Physics library
#include "btBulletDynamicsCommon.h"

// NTRT libraries
#include "ntrt/core/tgModel.h"
#include "ntrt/core/tgSubject.h"
#include "ntrt/core/tgCast.h"
#include "ntrt/core/abstractMarker.h"
#include "ntrt/core/tgBasicActuator.h"
#include "ntrt/tgcreator/tgBasicContactCableInfo.h"
#include "ntrt/core/tgSpringCableActuator.h"
#include "ntrt/core/tgRod.h"
#include "ntrt/core/tgBox.h"
#include "ntrt/core/tgString.h"
#include "ntrt/tgcreator/tgBuildSpec.h"
#include "ntrt/tgcreator/tgBasicActuatorInfo.h"
#include "ntrt/tgcreator/tgRigidAutoCompound.h"
#include "ntrt/tgcreator/tgRodInfo.h"
#include "ntrt/tgcreator/tgSphereInfo.h"
#include "ntrt/tgcreator/tgBoxInfo.h"
#include "ntrt/tgcreator/tgStructure.h"
#include "ntrt/tgcreator/tgStructureInfo.h"
#include "ntrt/tgcreator/tgUtil.h"

#endif


class targetModel: public tgSubject<targetModel>, public tgModel
{
public:

    /**
     * Constructor
     */
     targetModel();

	/**
	 * Constructor
	 * @param position target position in the simulation
	 * @param init_iters number of initialization iterations
	 */
    targetModel(btVector3 position);

	/**
	 * Nothing to do. Most functions already handled by tgModel::teardown
	 */
    virtual ~targetModel() {}
    
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
     * Receives a tgModelVisitor and dispatches itself into the
     * visitor's "render" function. This model will go to the default
     * tgModel function, which does nothing.
     * @param[in] r - a tgModelVisitor which will pass this model back
     * to itself 
     */
    virtual void onVisit(tgModelVisitor& r);

    /**
     * Provides the position of the target
     * @param position btVector3 instance which will be set to the target position
     */
    void getPosition(btVector3& position);

    /**
     * Sets the target position (it does not affect the visual position of a target already added to the simulation)
     * @param position btVector3 instance containing the target position
     */
    void setPosition(btVector3& position);

    /**
     * Undoes setup
     */
    virtual void teardown();

private:
    /**
     *  btVector3 instance representing the target position
     */
    btVector3 target_position;

    /**
     * Vector of rods composing the target
     */
    std::vector<tgRod*> rods;
};

#endif
