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
 * @file MovingBallModel.cpp
 * @brief Contains the implementation of class MovingBallModel
 * @author Jean Marc Bejjani
 * $Id$
 */

// This module
#include "MovingBallModel_passive.h"

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
        double muscle_stiffness;
        double muscle_damping;
        double rod_length;
        double rod_space;
        double friction;
        double rollFriction;
        double restitution;
        double muscle_pretension;
        bool   hist;
        double muscle_maxTens;
        double muscle_targetVelocity;
        double sphere_radius;
        double sphere_density;
        double connector_stiffness;
        double connector_damping;
        double connector_pretension;
        double connector_maxTens;
        double connector_targetVelocity;

    } config =
   {
	    1.73,    // density (g/cm3)
		0.15,     // radius (cm)
		0.5*11520.0,   //  muscle stiffness (mN / cm) so must be A(cm2) * E(mN/cm2) / L(cm)
		70.0,    // muscle damping (mN /(cm*sec))
		8.2,     // rod_length (cm)
		3.75,      // rod_space (cm)
		//0.99,      // friction (unitless)
		0.61,      // friction (unitless)
		0.005,    // rollFriction (unitless)
		0.0,      // restitution (?)
		0.5*11520*5*0.150,        // muscle pretension -> set to 4 * 613, the previous value of the rest length controller
		0,			// History logging (boolean)
		100000,   // muscle maxTens
		25,    // muscle targetVelocity

		0.4,      //sphere radius
		20.7, // with motor 20.7,		//sphere density

		60000.0,	//connector stiffness
		2000.0,	//connector damping
		0,			//connector pretension
		100000,	//connector max tension
		1,		//connector target Velocity
		 // Use the below values for earlier versions of simulation.
		 // 1.006,
		 // 0.31,
		 // 300000.0,
		 // 3000.0,
		 // 15.0,
		 // 7.5,
  };
} // namespace

MovingBallModel::MovingBallModel() :
    tgModel()
{
}
MovingBallModel::MovingBallModel(int loop) :
		force(0.01*loop)
{}
/**
 * Anonomous namespace for helper functions
 */
namespace
{

	void addNodes(tgStructure& oneUnit)
    {
    	const double half_length = config.rod_length / 2;

		oneUnit.addNode(-config.rod_space,  -half_length, 0);            // 0	real:7		connected to sphere triangle 2
		oneUnit.addNode(-config.rod_space,   half_length, 0);            // 1  real:1
		oneUnit.addNode( config.rod_space,  -half_length, 0);            // 2	real:12
		oneUnit.addNode( config.rod_space,   half_length, 0);            // 3	real:5		connected to sphere triangle 1
		oneUnit.addNode(0,           -config.rod_space,   -half_length); // 4	real:8		connected to sphere triangle 2
		oneUnit.addNode(0,           -config.rod_space,    half_length); // 5	real:11
		oneUnit.addNode(0,            config.rod_space,   -half_length); // 6	real:4
		oneUnit.addNode(0,            config.rod_space,    half_length); // 7	real:6		connected to sphere triangle 1
		oneUnit.addNode(-half_length, 0,            config.rod_space);   // 8	real:2
		oneUnit.addNode( half_length, 0,            config.rod_space);   // 9	real:10		connected to sphere triangle 1
		oneUnit.addNode(-half_length, 0,           -config.rod_space);   // 10	real:3		connected to sphere triangle 2
		oneUnit.addNode( half_length, 0,           -config.rod_space);   // 11	real:9
		oneUnit.addNode(           -config.rod_space+4.4705/2, 0,             0,"sphere");	     // 12	sphere
		oneUnit.addNode(-config.rod_space,  0, 0);						 // 13  rigid connection to sphere

		//weightless base
		oneUnit.addNode( config.rod_space+0.2,   half_length+0.2, 0+0.2,"down sensor");      	  	// 14	connected to 3 real:5
		oneUnit.addNode(0+0.2,            config.rod_space+0.2,    half_length+0.2, "down sensor"); 	// 15	connected to 7 real:6
		oneUnit.addNode( half_length+0.2, 0+0.2,            config.rod_space+0.2, "down sensor");	// 16 connected to 9 real:10

		oneUnit.addNode( config.rod_space+0.4,   half_length+0.4, 0+0.4, "weight");      	   		  // 17	connected to 3 real:5
		oneUnit.addNode(0+0.4,            config.rod_space+0.4,    half_length+0.4, "weight"); 	  // 18	connected to 7 real:6
		oneUnit.addNode( half_length+0.4, 0+0.4,            config.rod_space+0.4,"weight");		  // 19 connected to 9 real:10


		//custom weight
		oneUnit.addNode(-config.rod_space+20,  -half_length+20, 0+20,"weight");            // 20 connected to 0	real:7
		oneUnit.addNode(0+20,           -config.rod_space+20,   -half_length+20,"weight"); // 21 connected to 4	real:8
		oneUnit.addNode(-half_length+20, 0+20,           -config.rod_space+20,"weight");   // 22 connected to 10	real:3

		oneUnit.addNode(-config.rod_space-0.2,  -half_length-0.2, 0-0.2,"up sensor");            // 23 connected to 0	real:7
		oneUnit.addNode(0-0.2,           -config.rod_space-0.2,   -half_length-0.2,"up sensor"); // 24 connected to 4	real:8
		oneUnit.addNode(-half_length-0.2, 0-0.2,           -config.rod_space-0.2,"up sensor");   // 25 connected to 10	real:3



    }

	void addPairs(tgStructure& oneUnit)
	{
		//rigid structure
		oneUnit.addPair( 0,  13, "motor rod");
		oneUnit.addPair( 13,  1, "motor rod");
		oneUnit.addPair( 2,  3, "rod");
		oneUnit.addPair( 4,  5, "rod");
		oneUnit.addPair( 6,  7, "rod");
		oneUnit.addPair( 8,  9, "rod");
		oneUnit.addPair(10, 11, "rod");
		//light structure
		oneUnit.addPair(12, 13, "motor rod");
		//sensors structure
		oneUnit.addPair(14, 3, "connector");
		oneUnit.addPair(15, 7, "connector");
		oneUnit.addPair(16, 9, "connector");

		oneUnit.addPair(23, 0, "connector");
		oneUnit.addPair(4, 24, "connector");
		oneUnit.addPair(25, 10, "connector");


		//flexible structure
		oneUnit.addPair(0, 4,  "passive muscle");
		oneUnit.addPair(0, 5,  "passive muscle");
		oneUnit.addPair(0, 8,  "passive muscle");
		oneUnit.addPair(0, 10, "passive muscle");

		oneUnit.addPair(1, 6,  "passive muscle");
		oneUnit.addPair(1, 7,  "passive muscle");
		oneUnit.addPair(1, 8,  "passive muscle");
		oneUnit.addPair(1, 10, "passive muscle");

		oneUnit.addPair(2, 4,  "passive muscle");
		oneUnit.addPair(2, 5,  "passive muscle");
		oneUnit.addPair(2, 9,  "passive muscle");
		oneUnit.addPair(2, 11, "passive muscle");

		oneUnit.addPair(3, 7,  "passive muscle");
		oneUnit.addPair(3, 6,  "passive muscle");
		oneUnit.addPair(3, 9,  "passive muscle");
		oneUnit.addPair(3, 11, "passive muscle");

		oneUnit.addPair(4, 2,  "passive muscle");
		oneUnit.addPair(4, 10, "passive muscle");
		oneUnit.addPair(4, 11, "passive muscle");

		oneUnit.addPair(5, 8,  "passive muscle");
		oneUnit.addPair(5, 9,  "passive muscle");

		oneUnit.addPair(6, 10, "passive muscle");
		oneUnit.addPair(6, 11, "passive muscle");

		oneUnit.addPair(7, 8,  "passive muscle");
		oneUnit.addPair(7, 9,  "passive muscle");

		// new center node

		oneUnit.addPair(12,0, "active muscle");
		oneUnit.addPair(12,3, "active muscle");
		oneUnit.addPair(12,4, "active muscle");
		oneUnit.addPair(12,7, "active muscle");
		oneUnit.addPair(12,9, "active muscle");
		oneUnit.addPair(12,10, "active muscle");

		// added weights
		oneUnit.addPair(20,0, "custom muscle");
		oneUnit.addPair(21,4, "custom muscle");
		oneUnit.addPair(22,10, "custom muscle");

		oneUnit.addPair(17,3, "connector");
		oneUnit.addPair(18,7, "connector");
		oneUnit.addPair(19,9, "connector");

	}
   /* void addSegments(tgStructure& finalStruct, const tgStructure& oneUnit, double edge,
             size_t unitCount)
    {

    	const btVector3 offset(edge, edge, edge);
		const btVector3 center(0, 0, 0);
		for (size_t i = 0; i < unitCount; ++i)
		{
		  tgStructure* const t = new tgStructure(oneUnit);
		  t->addTags(tgString("Unit nb", i )); // Use num0, num1, num2 as a tag!!

		  t->addRotation(center,offset,(i + 1) * M_PI);
		  t->move((i + 1) * offset);


		  // Add a child to the structure
		  finalStruct.addChild(t);

		}
    }*/
	void addSegments(tgStructure& finalStruct, const tgStructure& oneUnit, std::vector<Module> moduleData)
	{
		double dist=1;
		btVector3 position[8]={btVector3(dist,dist,dist) , btVector3(-dist,dist,dist) ,
		    	btVector3(-dist,dist,-dist) , btVector3(dist,dist,-dist) , btVector3(-dist,-dist,-dist) ,
				btVector3(dist,-dist,-dist) , btVector3(dist,-dist,dist) , btVector3(-dist,-dist,dist)};
		const btVector3 center(0, 0, 0);
		for (size_t i = 0; i < moduleData.size(); ++i)
		{
		  tgStructure* const t = new tgStructure(oneUnit);
		  t->addTags(tgString("Unit nb", i )); // Use num0, num1, num2 as a tag!!
		  //std::cout << " parent face orientation: "<< moduleData[i].parent_face-1;
		  if(i!=0)
		  {
			  t->move(moduleData[i].position);
			  t->addRotation(moduleData[i].position,moduleData[i].position-moduleData[i-1].position, i*M_PI);
		  }
		  // Add a child to the structure
		  finalStruct.addChild(t);
		  //btVector3 j=btVector3(2,3,4);
		}
	}
    // Add actuators that connect the segments
    /*void connectUnits(tgStructure& finalStruct)
    {
    	const std::vector<tgStructure*> children = finalStruct.getChildren();
		for (size_t i = 1; i < children.size(); ++i)
		{
			tgNodes n0 = children[i-1]->getNodes();
			tgNodes n1 = children[i  ]->getNodes();
			finalStruct.addPair(n0[7], n1[4], "connector");
			finalStruct.addPair(n0[3], n1[0], "connector");
			finalStruct.addPair(n0[9], n1[10], "connector");
		}
    }*/
	void connectUnits(tgStructure& finalStruct, std::vector<Module> moduleData)
	{
		const std::vector<tgStructure*> children = finalStruct.getChildren();
		for (size_t i = 0; i < children.size(); ++i)
		{
			tgNodes n0 = children[i]->getNodes();
			for(size_t j=0;j<moduleData[i].connected_modules.size();j++)
			{
				if(moduleData[i].connected_modules[0]!=0)
				{
					tgNodes n1 = children[moduleData[i].connected_modules[j]-1]->getNodes();
					if(moduleData[i].connected_faces[j]==1)
					{
						finalStruct.addPair(n0[7], n1[4], "connector");
						finalStruct.addPair(n0[3], n1[0], "connector");
						finalStruct.addPair(n0[9], n1[10], "connector");
					}
					if(moduleData[i].connected_faces[j]==2)
					{
						finalStruct.addPair(n0[1], n1[2], "connector");
						finalStruct.addPair(n0[7], n1[4], "connector");
						finalStruct.addPair(n0[8], n1[11], "connector");
					}
					if(moduleData[i].connected_faces[j]==3)
					{
						finalStruct.addPair(n0[6], n1[5], "connector");
						finalStruct.addPair(n0[1], n1[2], "connector");
						finalStruct.addPair(n0[10], n1[9], "connector");
					}
					if(moduleData[i].connected_faces[j]==4)
					{
						finalStruct.addPair(n0[3], n1[0], "connector");
						finalStruct.addPair(n0[6], n1[5], "connector");
						finalStruct.addPair(n0[11], n1[8], "connector");
					}
					if(moduleData[i].connected_faces[j]==5)
					{
						finalStruct.addPair(n1[7], n0[4], "connector");
						finalStruct.addPair(n1[3], n0[0], "connector");
						finalStruct.addPair(n1[9], n0[10], "connector");
					}
					if(moduleData[i].connected_faces[j]==6)
					{
						finalStruct.addPair(n1[1], n0[2], "connector");
						finalStruct.addPair(n1[7], n0[4], "connector");
						finalStruct.addPair(n1[8], n0[11], "connector");
					}
					if(moduleData[i].connected_faces[j]==7)
					{
						finalStruct.addPair(n1[6], n0[5], "connector");
						finalStruct.addPair(n1[1], n0[2], "connector");
						finalStruct.addPair(n1[10], n0[9], "connector");
					}
					if(moduleData[i].connected_faces[j]==8)
					{
						finalStruct.addPair(n1[3], n0[0], "connector");
						finalStruct.addPair(n1[6], n0[5], "connector");
						finalStruct.addPair(n1[11], n0[8], "connector");
					}
				}

			}

		}
	}

	btVector3 convertVectors(Eigen::Vector3d Vector)
	{
		return btVector3(Vector.x(),Vector.y(),Vector.z());
	}

	Eigen::Vector3d convertVectors(btVector3 Vector)
	{
		return Eigen::Vector3d(Vector.getX(),Vector.getY(),Vector.getZ());
	}

    void mapActuators(MovingBallModel::ActuatorMap& actuatorMap,
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
    std::vector<Module> readData ()
    {
    	//trial input

		std::ifstream inFile;

        inFile.open("./Tensoft/src/dev/StructureStudy/Icosahedron/MovingBall_passive/input_passive.txt");

		if (!inFile) {
			std::cerr << "Unable to open file input.txt";
			exit(1);   // call system to stop
		}
		std::string line="";

		std::vector<Module> moduleData;
		std::string buffer="";
		while (std::getline(inFile, line))
		{
			Module temp;

			//loading line
			std::cout << line << std::endl;
			std::istringstream iss(line);

			//skipping "order:"
			iss >> buffer;

			//loading order
			iss >> temp.order;

			//skipping "connectedModules"
			iss >> buffer;

			//loading first connected modules
			iss >> buffer;

			//loading each connected modules
			int comptor=0;
			while(buffer != "connectedFaces:")
			{
				temp.connected_modules.push_back(atoi( buffer.c_str() ));
				comptor++;
				iss >> buffer;
			}
			comptor=0;

			//loading first connected face
			iss >> buffer;

			//loading each connected face
			while(buffer != "freq:")
			{
				temp.connected_faces.push_back(atoi( buffer.c_str() ));
				comptor++;
				iss >> buffer;
			}
			std::cout << std::endl;
			comptor=0;

			//loading actuator data

			iss >> temp.frequency;
			iss >> buffer;
			iss >> temp.amplitude;
			iss >> buffer;
			iss >> temp.phase;

			moduleData.push_back(temp);

		}
		inFile.close();

		return moduleData;
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

    std::vector<Module> findPositions(std::vector<Module> moduleData)
    {
    	double dist=sqrt(2)*config.rod_space;
    	btVector3 position[8]={btVector3(dist,dist,dist) , btVector3(-dist,dist,dist) ,
    			btVector3(-dist,dist,-dist) , btVector3(dist,dist,-dist) , btVector3(-dist,-dist,-dist) ,
				btVector3(dist,-dist,-dist) , btVector3(dist,-dist,dist) , btVector3(-dist,-dist,dist)};

    	for(int i=0; i < moduleData.size(); i++)
    	{
    		if(i==0)
    		{
    			moduleData[i].position = btVector3(0,0,0);
    		}
    		if(moduleData[i].connected_modules[0]!=0)
    		{
    			for(int j=0;j<moduleData[i].connected_modules.size();j++)
				{
					moduleData[moduleData[i].connected_modules[j]-1].position=moduleData[i].position
							+position[moduleData[i].connected_faces[j]-1];

					moduleData[moduleData[i].connected_modules[j]-1].parent_face=
							moduleData[i].connected_faces[j];
				}
    		}


    	}

    	return moduleData;
    }

} // namespace

void MovingBallModel::setup(tgWorld& world)
{

	//rigid structure config
	const tgRod::Config rodConfig(config.radius, config.density, config.friction,
				config.rollFriction, config.restitution);
	//motor rod structure config
	const tgRod::Config lightRodConfig(config.radius, config.sphere_density, config.friction,
						config.rollFriction, config.restitution);
	//weightless structure config
	const tgRod::Config weightlessRodConfig(config.radius, 0, config.friction,
							config.rollFriction, config.restitution);
	//sphere config
	const tgSphere::Config sphereConfig(config.sphere_radius, config.sphere_density);

	// weight config
	const tgSphere::Config weightConfig(config.sphere_radius/2, 0*config.density);
	const tgSphere::Config sensorConfig(config.sphere_radius/2, config.density);

	//muscle config
	const tgBasicActuator::Config pmuscleConfig(config.muscle_stiffness, config.muscle_damping, config.muscle_pretension, config.hist,
						config.muscle_maxTens, config.muscle_targetVelocity);

	//muscle config
	const tgBasicActuator::Config amuscleConfig(config.muscle_stiffness, config.muscle_damping, config.muscle_pretension, config.hist,
						config.muscle_maxTens, config.muscle_targetVelocity);

	//connector config
	const tgBasicActuator::Config connectorConfig(config.connector_stiffness, config.connector_damping, config.connector_pretension, config.hist,
							config.connector_maxTens, config.connector_targetVelocity);

	//custom muscle config
	const tgBasicActuator::Config customMuscleConfig(config.muscle_stiffness, 10*config.muscle_damping, force*config.muscle_stiffness*10, config.hist,
			config.muscle_maxTens, 10*config.muscle_targetVelocity);



	//load the data structure

	m_moduleData=readData();

	m_moduleData=findPositions(m_moduleData);

	// Start creating the structure
	tgStructure oneUnit;
	addNodes(oneUnit);
	addPairs(oneUnit);

	double edge=sqrt(2)*config.rod_space;
	tgStructure finalStruct;


	addSegments(finalStruct,oneUnit,m_moduleData);

	std::cout<< "test" << std::endl;

	//connectUnits(finalStruct);

	connectUnits(finalStruct,m_moduleData);

	finalStruct.move(btVector3(0, 10*m_moduleData.size(), 0));

	// Add a rotation. This is needed if the ground slopes too much,
	// otherwise  glitches put a rod below the ground.
	btVector3 rotationPoint = btVector3(0, 0, 0); // origin
	btVector3 rotationAxis = btVector3(0, 1, 0);  // y-axis
	double rotationAngle = M_PI/2;
	finalStruct.addRotation(rotationPoint, rotationAxis, rotationAngle);

	finalStruct.addRotation(rotationPoint, btVector3(1,0,1),0.303889*M_PI+M_PI);

	finalStruct.move(btVector3(0, 40, 0));


	// Create the build spec that uses tags to turn the structure into a real model
	tgBuildSpec spec;
	spec.addBuilder("rod", new tgRodInfo(rodConfig));
	spec.addBuilder("motor rod", new tgRodInfo(lightRodConfig));
	spec.addBuilder("weightless rod", new tgRodInfo(weightlessRodConfig));
	spec.addBuilder("sphere", new tgSphereInfo(sphereConfig));
	spec.addBuilder("down sensor", new tgSphereInfo(sensorConfig));
	spec.addBuilder("up sensor", new tgSphereInfo(sensorConfig));

	spec.addBuilder("weight", new tgSphereInfo(weightConfig));

	spec.addBuilder("passive muscle", new tgBasicActuatorInfo(pmuscleConfig));
	spec.addBuilder("active muscle", new tgBasicActuatorInfo(amuscleConfig));
	spec.addBuilder("connector", new tgBasicActuatorInfo(connectorConfig));
	spec.addBuilder("custom muscle", new tgBasicActuatorInfo(customMuscleConfig));

	// Create your structureInfo
	tgStructureInfo structureInfo(finalStruct, spec);

	// Use the structureInfo to build ourselves
	structureInfo.buildInto(*this, world);

    // We could now use tgCast::filter or similar to pull out the models (e.g. actuators)
    // that we want to control.    
    allActuators = tgCast::filter<tgModel, tgBasicActuator> (getDescendants());
    mapActuators(actuatorMap, *this);

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

void MovingBallModel::step(double dt)
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
MovingBallModel::getActuators (const std::string& key) const
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

