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
 * @file MaterialAnalysisModel.cpp
 * @brief Contains the implementation of class MaterialAnalysisModel
 * @author Jean Marc Bejjani
 * $Id$
 */

// This module
#include "MaterialAnalysisModel.h"

#include <fstream>

#include <iostream>
#include <stdexcept>
#include <Eigen/Dense>



namespace
{
    // see tgBasicActuator and tgRod for a descripton of these rod parameters
    // (specifically, those related to the motor moving the strings.)
    // NOTE that any parameter that depends on units of length will scale
    // with the current gravity scaling. E.g., with gravity as 98.1,
    // the length units below are in decimeters.

    // Note: This current model of the SUPERball rod is 1.5m long by 3 cm radius,
    // which is 0.00424 m^3.
    // For SUPERball v1.5, mass = 3.5kg per strut, which comes out to
    // 0.825 kg / (decimeter^3).

    // similarly, frictional parameters are for the tgRod objects.


	const struct Config
	{
		double density;
		double radius;
		double low_stiffness;
		double high_stiffness;
		double damping;
		double low_pretension;
		double high_pretension;
		double edge;
		double height;

		double friction;
        double rollFriction;
        double restitution;
        bool   hist;
        double muscle_maxTens;
        double muscle_targetVelocity;

        double sphere_radius;

        double connector_stiffness;
        double connector_damping;
        double connector_pretension;
        double connector_maxTens;
        double connector_targetVelocity;

	}
	config =
    {
		1.73,     				// density (g / cm^3)
		0.15,     				// radius (cm)
		533333.0,				    // low stiffness (mN / cm)
		4500.0,   				// high stiffness (mN / cm)
		70.0,     				// damping (mN / (cm*sec))
		1*1000*5*0.150,     	// low pretension (mass * length / sec^2)
		1*1000*5*0.150,   		// high pretension (mass * length / sec^2)
		2,     				// triangle_edge (length)
		5.5,     				// prism_height (length)

		0.6,      				// friction (unitless)
		0.005,     				// rollFriction (unitless)
		0.0,      				// restitution (?)
		0,						// History logging (boolean)
		100000,   				// muscle maxTens
		25,    					// muscle targetVelocity

		0.4,      				//sphere radius

		600000.0,				//connector stiffness
		2.0,					//connector damping
		0,						//connector pretension
		100000,					//connector max tension
		10,						//connector target Velocity
	};
} // namespace

MaterialAnalysisModel::MaterialAnalysisModel() :
    tgModel()
{
}
MaterialAnalysisModel::MaterialAnalysisModel(int loop) :
		force(0.0001*loop)
{}
/**
 * Anonomous namespace for helper functions
 */
namespace
{

	void addNodes(tgStructure& oneUnit)
    {


		oneUnit.addNode(-config.edge, 0, 0, "weight"); // 0

		oneUnit.addNode( config.edge, 0, 0, "weight"); // 1

		oneUnit.addNode(-config.edge, config.edge, 0, "weight"); // 2

		oneUnit.addNode(config.edge, config.edge, 0, "weight"); // 3

    }

	void addPairs(tgStructure& oneUnit)
	{
		oneUnit.addPair( 0,  1, "low muscle");
		oneUnit.addPair( 2,  3, "high muscle");

	}

	btVector3 convertVectors(Eigen::Vector3d Vector)
	{
		return btVector3(Vector.x(),Vector.y(),Vector.z());
	}

	Eigen::Vector3d convertVectors(btVector3 Vector)
	{
		return Eigen::Vector3d(Vector.getX(),Vector.getY(),Vector.getZ());
	}



    void trace(const tgStructureInfo& structureInfo, tgModel& model)
    {
        std::cout << "StructureInfo:" << std::endl
          << structureInfo    << std::endl
          << "Model: "        << std::endl
          << model            << std::endl;
    }


    btVector3 rotateAxis(btVector3 Axis, double angle, btVector3 Vector )
    {
    	Eigen::Matrix3d rotationMatrix;
		rotationMatrix <<
				cos(angle)+pow(Axis.getX(),2)*(1-cos(angle)) ,
				Axis.getX()*Axis.getY()*(1-cos(angle))-Axis.getZ()*sin(angle) ,
				Axis.getX()*Axis.getZ()*(1-cos(angle))+Axis.getY()*sin(angle) ,

				Axis.getY()*Axis.getX()*(1-cos(angle))+Axis.getZ()*sin(angle) ,
				cos(angle)+pow(Axis.getY(),2)*(1-cos(angle)) ,
				Axis.getY()*Axis.getZ()*(1-cos(angle))-Axis.getX()*sin(angle) ,

				Axis.getX()*Axis.getZ()*(1-cos(angle))-Axis.getY()*sin(angle) ,
				Axis.getY()*Axis.getZ()*(1-cos(angle))+Axis.getX()*sin(angle) ,
				cos(angle)+pow(Axis.getZ(),2)*(1-cos(angle));
		std::cout << rotationMatrix << std::endl;
		return convertVectors(rotationMatrix*convertVectors(Vector));
    }



} // namespace

void MaterialAnalysisModel::setup(tgWorld& world)
{

	// weight config
	const tgSphere::Config weightConfig(config.sphere_radius/2, 0*config.density);

	//low muscle config
	const tgBasicActuator::Config lmuscleConfig(config.low_stiffness, config.damping, force*config.low_stiffness*10, config.hist,
						config.muscle_maxTens, config.muscle_targetVelocity);

	//high muscle config
	const tgBasicActuator::Config hmuscleConfig(config.high_stiffness, config.damping, force*config.high_stiffness*10, config.hist,
						config.muscle_maxTens, config.muscle_targetVelocity);


	tgStructure finalStruct;
	addNodes(finalStruct);
	addPairs(finalStruct);


	finalStruct.move(btVector3(0, 40, 0));


	// Create the build spec that uses tags to turn the structure into a real model
	tgBuildSpec spec;




	spec.addBuilder("weight", new tgSphereInfo(weightConfig));

	spec.addBuilder("low muscle", new tgBasicActuatorInfo(lmuscleConfig));
	spec.addBuilder("high muscle", new tgBasicActuatorInfo(hmuscleConfig));

	// Create your structureInfo
	tgStructureInfo structureInfo(finalStruct, spec);

	// Use the structureInfo to build ourselves
	structureInfo.buildInto(*this, world);

    // We could now use tgCast::filter or similar to pull out the models (e.g. actuators)
    // that we want to control.    

    //trace(structureInfo, *this);

	//print data

	std::cout << "number of modules: " << m_moduleData.size() << std::endl;
	for(int h=0 ; h < m_moduleData.size() ; h++)
	{
		std::cout << "order: " << m_moduleData[h].order << std::endl;

		std::cout << "connected modules: ";
		for(int f=0; f<m_moduleData[h].connected_modules.size();f++)
		{
			std::cout << m_moduleData[h].connected_modules[f] << " ";
		}
		std::cout << std::endl;
		std::cout << "connected faces: ";
		for(int f=0; f<m_moduleData[h].connected_faces.size();f++)
		{
			std::cout << m_moduleData[h].connected_faces[f] << " ";
		}
		std::cout << std::endl;
		std::cout << "freq: " << m_moduleData[h].frequency << std::endl;
		std::cout << "amp: " << m_moduleData[h].amplitude << std::endl;
		std::cout << "phase: " << m_moduleData[h].phase << std::endl;

		std::cout << "position: " << m_moduleData[h].position << std::endl;

		std::cout << "parent: " << m_moduleData[h].parent_face << std::endl;
	}

    // Actually setup the children
    tgModel::setup(world);
}

void MaterialAnalysisModel::step(double dt)
{
    if (dt < 0.0)
    {
        throw std::invalid_argument("dt is not positive");
    }
    else
    {
        // Notify observers (controllers) of the step so that they can take action
        notifyStep(dt);
        // Step any children
        tgModel::step(dt);
    }
}
    
const std::vector<tgBasicActuator*>&
MaterialAnalysisModel::getActuators (const std::string& key) const
{
    const ActuatorMap::const_iterator it = actuatorMap.find(key);
    if (it == actuatorMap.end())
    {
        throw std::invalid_argument("Key '" + key + "' not found in actuator map");
    }
    else
    {
        return it->second;
    }
}

