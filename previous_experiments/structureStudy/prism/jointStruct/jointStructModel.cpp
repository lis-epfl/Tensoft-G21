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
 * @file jointStructModel.cpp
 * @brief Contains the implementation of class jointStructModel
 * @author Jean Marc Bejjani
 * $Id$
 */

// This module
#include "jointStructModel.h"

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
		1*70342, //4500.0,				    // low stiffness (mN / cm)
		70342,   				// high stiffness (mN / cm)
		70.0,     				// damping (mN / (cm*sec))
		0.0*70342*5*0.150, //1*4500*5*0.150,     	// low pretension (mass * length / sec^2)
		1*70342*5*0.0,   		// high pretension (mass * length / sec^2)
		6.5,     				// triangle_edge (length)
		5.5,     				// prism_height (length)

		0.6,      				// friction (unitless)
		0.005,     				// rollFriction (unitless)
		0.0,      				// restitution (?)
		0,						// History logging (boolean)
		100000,   				// muscle maxTens
		25,    					// muscle targetVelocity

		0.4,      				//sphere radius

		6000000.0,				//connector stiffness
		2.0,					//connector damping
		0,						//connector pretension
		100000,					//connector max tension
		10,						//connector target Velocity
	};
} // namespace

jointStructModel::jointStructModel() :
    tgModel()
{
}
jointStructModel::jointStructModel(int loop) :
		force(0.001*loop)
{}
/**
 * Anonomous namespace for helper functions
 */
namespace
{

	void addNodes(tgStructure& oneUnit,int nbModules)
    {
		double width=config.edge*cos(M_PI/6);

		for(int i=0; i<nbModules;i++)
		{
			// bottom right
			oneUnit.addNode(-config.edge / 2.0, i*(config.height+0.2), 0); // 6i
			// bottom left
			oneUnit.addNode( config.edge / 2.0, i*(config.height+0.2), 0); // 6i+1
			// bottom front
			oneUnit.addNode(0, i*(config.height+0.2), width); // 6i+2
			// top right
			oneUnit.addNode(-config.edge / 2.0, i*(config.height+0.2)+config.height, 0); // 6i+3
			// top left
			oneUnit.addNode( config.edge / 2.0, i*(config.height+0.2)+config.height, 0); // 6i+4
			// top front
			oneUnit.addNode(0, i*(config.height+0.2)+config.height, width); // 6i+5
		}

		// base + down sensors

    			// bottom right
    			oneUnit.addNode((-config.edge / 2.0), 0-0.2, 0, "down sensor"); 					// 6*nb attached to 0
    			// bottom left
    			oneUnit.addNode( (config.edge / 2.0), 0-0.2, 0, "down sensor"); 					// 6*nb+1 attached to 1
    			// bottom front
    			oneUnit.addNode(0, 0-0.2, width, "down sensor"); 									// 6*nb+2 attached to 2

    			// bottom right
				oneUnit.addNode((-config.edge / 2.0), 0-0.65, 0, "base weight"); 					// 6*nb+3 attached to 0
				// bottom left
				oneUnit.addNode( (config.edge / 2.0), 0-0.65, 0, "base weight"); 					// *nb+4 attached to 1
				// bottom front
				oneUnit.addNode(0, 0-0.65, width, "base weight"); 									// 3*nb+5 attached to 2


		//custom weight + up sensors

				// top right
				oneUnit.addNode((-config.edge / 2.0), (config.height-20), 0, "custom weight"); 		// 6*nb+6 attached to 6*nb-3
				// top left
				oneUnit.addNode( (config.edge / 2.0), (config.height-20), 0, "custom weight"); 		// 6*nb+7 attached to 6*nb-2
				// top front
				oneUnit.addNode(0, (config.height-20), width, "custom weight"); 						// 6*nb+8 attached to 6*nb-1
				// top right
				oneUnit.addNode((-config.edge / 2.0), nbModules*config.height+0.3, 0, "up sensor"); 			// 6*nb+9 attached to 6*nb-3
				// top left
				oneUnit.addNode( (config.edge / 2.0), nbModules*config.height+0.3, 0, "up sensor"); 			// 6*nb+10 attached to 6*nb-2
				// top front
				oneUnit.addNode(0, nbModules*config.height+0.3, width, "up sensor"); 							// 6*nb+11 attached to 6*nb-1



    }

	void addPairs(tgStructure& oneUnit, int nbModules, int* stif)
	{

		for(int i=0;i<nbModules;i++)
		{
			//rods
			oneUnit.addPair( 6*i,    6*i+4, "rod");
			oneUnit.addPair( 6*i+1,  6*i+5, "rod");
			oneUnit.addPair( 6*i+2,  6*i+3, "rod");

			// Top
			if(!stif[3*i])
				oneUnit.addPair(6*i+3, 6*i+4, "low muscle");
			else
				oneUnit.addPair(6*i+3, 6*i+4, "high muscle");

			if(!stif[3*i+1])
				oneUnit.addPair(6*i+4, 6*i+5, "low muscle");
			else
				oneUnit.addPair(6*i+4, 6*i+5, "high muscle");

			if(!stif[3*i+2])
				oneUnit.addPair(6*i+5, 6*i+3, "low muscle");
			else
				oneUnit.addPair(6*i+5, 6*i+3, "high muscle");

			//Edges
			oneUnit.addPair(6*i,   6*i+3, "high muscle");
			oneUnit.addPair(6*i+1, 6*i+4, "high muscle");
			oneUnit.addPair(6*i+2, 6*i+5, "high muscle");


			if(i!=0)
			{
				//connection between modules
				oneUnit.addPair( 6*i,    6*i-3, "connector");
				oneUnit.addPair( 6*i+1,  6*i-2, "connector");
				oneUnit.addPair( 6*i+2,  6*i-1, "connector");
			}
		}

		// Bottom Triangle
		oneUnit.addPair(0, 1,  "high muscle");
		oneUnit.addPair(1, 2,  "high muscle");
		oneUnit.addPair(2, 0,  "high muscle");

		//down sensors
		oneUnit.addPair( 6*nbModules,    0, "connector");
		oneUnit.addPair( 6*nbModules+1,  1, "connector");
		oneUnit.addPair( 6*nbModules+2,  2, "connector");

		// base attach
		oneUnit.addPair( 6*nbModules+3,  0, "connector");
		oneUnit.addPair( 6*nbModules+4,  1, "connector");
		oneUnit.addPair( 6*nbModules+5,  2, "connector");

		//custom muscle
		oneUnit.addPair( 6*nbModules+6, 6*nbModules-3, "custom muscle");
		oneUnit.addPair( 6*nbModules+7, 6*nbModules-2, "custom muscle");
		oneUnit.addPair( 6*nbModules+8, 6*nbModules-1, "custom muscle");

		//up sensors
		oneUnit.addPair( 6*nbModules+9, 6*nbModules-3, "connector");
		oneUnit.addPair( 6*nbModules+10,6*nbModules-2, "connector");
		oneUnit.addPair( 6*nbModules+11,6*nbModules-1, "connector");
	}

	btVector3 convertVectors(Eigen::Vector3d Vector)
	{
		return btVector3(Vector.x(),Vector.y(),Vector.z());
	}

	Eigen::Vector3d convertVectors(btVector3 Vector)
	{
		return Eigen::Vector3d(Vector.getX(),Vector.getY(),Vector.getZ());
	}

    void mapActuators(jointStructModel::ActuatorMap& actuatorMap,
            tgModel& model)
    {
        // Note that tags don't need to match exactly, we could create
        // supersets if we wanted to
        actuatorMap["motor"]  = model.find<tgBasicActuator>("active muscle");
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

void jointStructModel::setup(tgWorld& world)
{

	//rigid structure config
	const tgRod::Config rodConfig(config.radius, config.density, config.friction,
				config.rollFriction, config.restitution);
	//weightless structure config
	const tgRod::Config weightlessRodConfig(config.radius, 0, config.friction,
							config.rollFriction, config.restitution);

	// weight config
	const tgSphere::Config weightConfig(config.sphere_radius/2, 0*config.density);
	const tgSphere::Config sensorConfig(config.sphere_radius/2, config.density);

	//low muscle config
	const tgBasicActuator::Config lmuscleConfig(config.low_stiffness, config.damping, config.low_pretension, config.hist,
						config.muscle_maxTens, config.muscle_targetVelocity);

	//high muscle config
	const tgBasicActuator::Config hmuscleConfig(config.high_stiffness, config.damping, config.high_pretension, config.hist,
						config.muscle_maxTens, config.muscle_targetVelocity);

	//connector config
	const tgBasicActuator::Config connectorConfig(config.connector_stiffness, config.connector_damping,
			config.connector_pretension, config.hist, config.connector_maxTens, config.connector_targetVelocity);

	//custom muscle config
	const tgBasicActuator::Config customMuscleConfig(config.high_stiffness, config.damping, force*config.high_stiffness*10,
			config.hist, config.muscle_maxTens, 10*config.muscle_targetVelocity);

	tgStructure finalStruct;

	int nbModules=4;

	int stif[3*nbModules]={0,0,0,1,1,1,0,0,0,1,1,1};

	addNodes(finalStruct,nbModules);
	addPairs(finalStruct,nbModules,stif);


	finalStruct.move(btVector3(0, 2.5, 0));


	// Create the build spec that uses tags to turn the structure into a real model
	tgBuildSpec spec;
	spec.addBuilder("rod", new tgRodInfo(rodConfig));
	spec.addBuilder("sensor down rod", new tgRodInfo(rodConfig));
	spec.addBuilder("sensor up rod", new tgRodInfo(rodConfig));



	spec.addBuilder("base weight", new tgSphereInfo(weightConfig));
	spec.addBuilder("custom weight", new tgSphereInfo(weightConfig));
	spec.addBuilder("up sensor", new tgSphereInfo(sensorConfig));
	spec.addBuilder("down sensor", new tgSphereInfo(sensorConfig));


	spec.addBuilder("low muscle", new tgBasicActuatorInfo(lmuscleConfig));
	spec.addBuilder("high muscle", new tgBasicActuatorInfo(hmuscleConfig));
	spec.addBuilder("connector", new tgBasicActuatorInfo(connectorConfig));
	spec.addBuilder("custom muscle", new tgBasicActuatorInfo(customMuscleConfig));

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

void jointStructModel::step(double dt)
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
jointStructModel::getActuators (const std::string& key) const
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

