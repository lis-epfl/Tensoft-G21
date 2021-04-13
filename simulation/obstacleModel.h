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

#ifndef OBSTACLE_MODEL_H
#define OBSTACLE_MODEL_H

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

class obstacleModel: public tgSubject<obstacleModel>, public tgModel
{
public:

    /**
     * Constructor
     */
    obstacleModel();

    /**
     * Constructor
     * @param frontFaceCoM center of mass of the robot's front face
     * @param robot_len length of the robot
     * @param target_pos target position (required to determine the opening's position)
     * @param t_bearing target bearing
     */
	obstacleModel(btVector3 frontFaceCoM, double rb_len, btVector3 target_pos, double rb_target_bearing);

	/**
	 * Nothing to do. Most functions already handled by tgModel::teardown
	 */
    virtual ~obstacleModel() {}

    /**
    * Initializes the properties required for setup
    * @param frontFaceCoM center of mass of the robot's front face
    * @param rb_len length of the robot
    * @param target_pos target position (required to determine the opening's position)
    * @param rb_target_bearing target bearing
    */
    void initialize(btVector3 frontFaceCoM, double rb_len, btVector3 target_pos, double rb_target_bearing);
    
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
     * Determine if the robot is going to hit a wall wrt to a sensitivity value
     * @param frontFaceCoM position of the robot front face
     * @param frontFaceDir direction of the robot
     * @param sensitivity proximity sensor's sensitivity
     * @return true if it is going to hit a wall, false otherwise
     */
    bool isOnPath(btVector3 frontFaceCoM, btVector3 frontFaceDir, double sensitivity);

    /**
     * Provides the position of the opening
     * @param opening_pos btVector3 instance that will contain the opening position
     */
    void get_opening_pos(btVector3& opening_pos);

    /**
     * Returns the sign of the distance w.r.t. the opening entrance
     * @param robot_pos position of the robot's front face
     * @return 1 if the robot is inside the walls, -1 otherwise
     */
    int get_distance_sign(btVector3 robot_pos);

	/**
     * Receives a tgModelVisitor and dispatches itself into the
     * visitor's "render" function. This model will go to the default
     * tgModel function, which does nothing.
     * @param[in] r - a tgModelVisitor which will pass this model back
     * to itself 
     */
    virtual void onVisit(tgModelVisitor& r);

    /**
     * Undoes setup
     */
    virtual void teardown();

private:
    std::vector<tgPair> h_walls;
    std::vector<tgPair> v_walls;

    std::vector<btVector3> opening_entrance;

    btVector3 rbInitFrontFace;

    double robot_len;

    btVector3 target_dir;

    double t_bearing;

    double t_angle;
};

#endif
