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

#ifndef ROBOT_MODEL_H
#define ROBOT_MODEL_H

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
	double stiffness;

	btVector3 e1;
	btVector3 e2;
	btVector3 e3;

	std::vector<btVector3> rotation_directions;
};


class robotModel: public tgSubject<robotModel>, public tgModel
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
    robotModel();
    robotModel(int argc, char *argv[]);
    robotModel(int argc, char *argv[], std::vector<double>& activePreStretches);

	/**
	 * Nothing to do. Most functions already handled by tgModel::teardown
	 */
    virtual ~robotModel() {}
    
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
    const std::vector<tgBasicActuator*>& getActuators(const std::string& key) const;

    /**
     * Returns the module's size.
     * @return module's size.
     */
     double getModuleSize();

    /**
     * Returns the length of the robot
     * @param num_modules number of robot's modules (used only in case the model has not been added to the simulation yet)
     * @return robot's length
     */
    double getLength(int num_modules);

    /**
     * Returns the movement direction of the robot
     * @return movement direction (-1 = backward, 1 = forward)
     */
    int getMovDir();

    /**
     * Sets the movement direction parameter (used to determine the front face)
     * @param moving_direction movement direction (-1 = backward, 1 = forward)
     */
    void setMovDir(int moving_direction);

    /**
     * Sets the rotation angle w.r.t. the z axis
     * @param rot_angle rotation angle in radiants
     */
    void setRotationAngle(double rot_angle);

    /**
     * Computes the coordinates of the robot front face
     * @param position vector that will contain the result
     */
    void getCurrentPosition(btVector3& position);

    /**
     * Computes the coordinates of the robot front face and the one parallel to it
     */
    void getFirstModulePosition(btVector3& f_position, btVector3& p_position);

    /**
     * Computes target coordinates from distance and bearing
     * @param distance distance from the frontFace CoM
     * @param bearing bearing wrt the axis of the first module
     * @param target btVector3 that will contain the result
     */
    void getCoordinatesFromDistAndBearing(double distance, double bearing, btVector3& target);

    /**
     * Compute the distance and the bearing to a given target
     * @param target btVector3 instance containing the coordinates of the target to reach
     * @param distance the computed distance value
     * @param bearing the computed bearing value
     */
    void getDistAndBearingToGoal(btVector3 target, double& distance, double& bearing);

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
    
    /**
     * Return a std::size_t indicating the number of segments in the 
     * tetraSpine.
     * @return a std::size_t with the value of m_segments
     */
    std::vector<Module> m_moduleData;

#ifdef MEASURE_CABLES
    tgStructure finalStruct;
#endif

private:
	
	/**
	 * A std::vector containing all of the tgBasicActuators amongst
	 * the children of this model. Populated during setup
	 */
    std::vector<tgBasicActuator*> allActuators;
	
	/**
	 * A typedef of std::map from std::string to tgBasicActuator*. Contains
	 * mapped actuators, populated during setup.
	 */
    ActuatorMap actuatorMap;

    /**
	 * A std::vector containing the rods related to the robot's front
	 * face. Populated during setup
	 */
    std::vector<tgRod*> frontFace;

    /**
	 * A std::vector containing the rods related to the robot's face
	 * parallel to the front one. Populated during setup
	 */
    std::vector<tgRod*> frontFaceParallel;

    int m_argc;
    char **m_argv;
	int m_initPos;

	/**
	 * Active cables pre-stretch modifiers
	 */
	std::vector<double> activePreStretches;

    /**
     * Movement direction: -1 = backward, 1 = forward
     */
    int mov_dir;

    double rotation_angle;

    double module_size = 11.11260546;
};

#endif
