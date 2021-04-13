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
#include "MovingBallModel_sim.h"

#include <fstream>

#include <iostream>
#include <sstream>
#include <stdexcept>

#include <Eigen/Dense>


#include <stdio.h>
#include <unistd.h>


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
        1*11520.0,   //  muscle stiffness (mN / cm) so must be A(cm2) * E(mN/cm2) / L(cm)
        70.0,    // muscle damping (mN /(cm*sec))
        8.2,     // rod_length (cm)
        3.75,      // rod_space (cm)
        //0.99,      // friction (unitless)
        0.6,      // friction (unitless)
        0.005,     // rollFriction (unitless)
        0.0,      // restitution (?)
        1*11520*5*0.150,        // muscle pretension -> set to 4 * 613, the previous value of the rest length controller
        0,      // History logging (boolean)
        100000,   // muscle maxTens
        25,    // muscle targetVelocity

        0.4,      //sphere radius
        20.7, // with motor 20.7,    //sphere density

        60000.0,  //connector stiffness
        20.0,  //connector damping
        0,      //connector pretension
        100000,  //connector max tension
        10,    //connector target Velocity

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
MovingBallModel::MovingBallModel(int argc, char *argv[]) :
    m_argc(argc),m_argv(argv),m_initPos(0)
{

}

/**
 * Anonomous namespace for helper functions
 */
namespace
{

	void addNodes(tgStructure& oneUnit)
    {
    	const double half_length = config.rod_length / 2;

		oneUnit.addNode(-config.rod_space,  -half_length, 0);            // 0	real:7		connected to sphere
		oneUnit.addNode(-config.rod_space,   half_length, 0);            // 1   real:1
		oneUnit.addNode( config.rod_space,  -half_length, 0);            // 2	real:12
		oneUnit.addNode( config.rod_space,   half_length, 0);            // 3	real:5		connected to sphere
		oneUnit.addNode(0,           -config.rod_space,   -half_length); // 4	real:8		connected to sphere
		oneUnit.addNode(0,           -config.rod_space,    half_length); // 5	real:11
		oneUnit.addNode(0,            config.rod_space,   -half_length); // 6	real:4
		oneUnit.addNode(0,            config.rod_space,    half_length); // 7	real:6		connected to sphere
		oneUnit.addNode(-half_length, 0,            config.rod_space);   // 8	real:2
		oneUnit.addNode( half_length, 0,            config.rod_space);   // 9	real:10		connected to sphere
		oneUnit.addNode(-half_length, 0,           -config.rod_space);   // 10	real:3		connected to sphere
		oneUnit.addNode( half_length, 0,           -config.rod_space);   // 11	real:9
                oneUnit.addNode( -config.rod_space+4.4705/2, 0, 0,"sphere");	     // 12	sphere


		oneUnit.addNode(-config.rod_space,  0, 0);						 // 13  rigid       connection to sphere
    }

	void addPairs(tgStructure& oneUnit, int moduleOrder)
	{

		std::string stringOrder;
		std::stringstream ss;
		ss << moduleOrder;
		stringOrder = ss.str();

		//rigid structure
		oneUnit.addPair( 0,  13, "rod");
		oneUnit.addPair( 13,  1, "rod");
		oneUnit.addPair( 2,  3, "rod");
		oneUnit.addPair( 4,  5, "rod");
		oneUnit.addPair( 6,  7, "rod");
		oneUnit.addPair( 8,  9, "rod");
		oneUnit.addPair(10, 11, "rod");
		oneUnit.addPair(12, 13, "light rod");
		//flexible structure
		oneUnit.addPair(0, 4,  "passive muscle"+stringOrder);
		oneUnit.addPair(0, 5,  "passive muscle"+stringOrder);
		oneUnit.addPair(0, 8,  "passive muscle"+stringOrder);
		oneUnit.addPair(0, 10, "passive muscle"+stringOrder);

		oneUnit.addPair(1, 6,  "passive muscle"+stringOrder);
		oneUnit.addPair(1, 7,  "passive muscle"+stringOrder);
		oneUnit.addPair(1, 8,  "passive muscle"+stringOrder);
		oneUnit.addPair(1, 10, "passive muscle"+stringOrder);

		oneUnit.addPair(2, 4,  "passive muscle"+stringOrder);
		oneUnit.addPair(2, 5,  "passive muscle"+stringOrder);
		oneUnit.addPair(2, 9,  "passive muscle"+stringOrder);
		oneUnit.addPair(2, 11, "passive muscle"+stringOrder);

		oneUnit.addPair(3, 7,  "passive muscle"+stringOrder);
		oneUnit.addPair(3, 6,  "passive muscle"+stringOrder);
		oneUnit.addPair(3, 9,  "passive muscle"+stringOrder);
		oneUnit.addPair(3, 11, "passive muscle"+stringOrder);

		oneUnit.addPair(4, 2,  "passive muscle"+stringOrder);
		oneUnit.addPair(4, 10, "passive muscle"+stringOrder);
		oneUnit.addPair(4, 11, "passive muscle"+stringOrder);

		oneUnit.addPair(5, 8,  "passive muscle"+stringOrder);
		oneUnit.addPair(5, 9,  "passive muscle"+stringOrder);

		oneUnit.addPair(6, 10, "passive muscle"+stringOrder);
		oneUnit.addPair(6, 11, "passive muscle"+stringOrder);

		oneUnit.addPair(7, 8,  "passive muscle"+stringOrder);
		oneUnit.addPair(7, 9,  "passive muscle"+stringOrder);

		// new center node


	}

	void addActuator(tgStructure& oneUnit, int rotation)
	{
		if(rotation==0)
		{
			oneUnit.addPair(12,0, "active muscle");
			oneUnit.addPair(12,3, "active muscle");
			oneUnit.addPair(12,4, "active muscle");
			oneUnit.addPair(12,7, "active muscle");
			oneUnit.addPair(12,9, "active muscle");
			oneUnit.addPair(12,10, "active muscle");
		}
		if(rotation==1)
		{
			oneUnit.addPair(12,7, "active muscle");
			oneUnit.addPair(12,8, "active muscle");
			oneUnit.addPair(12,1, "active muscle");
			oneUnit.addPair(12,11, "active muscle");
			oneUnit.addPair(12,4, "active muscle");
			oneUnit.addPair(12,2, "active muscle");
		}
		if(rotation==2)
		{
			oneUnit.addPair(12,6, "active muscle");
			oneUnit.addPair(12,1, "active muscle");
			oneUnit.addPair(12,10, "active muscle");
			oneUnit.addPair(12,2, "active muscle");
			oneUnit.addPair(12,9, "active muscle");
			oneUnit.addPair(12,5, "active muscle");
		}
		if(rotation==3)
		{
			oneUnit.addPair(12,3, "active muscle");
			oneUnit.addPair(12,6, "active muscle");
			oneUnit.addPair(12,11, "active muscle");
			oneUnit.addPair(12,5, "active muscle");
			oneUnit.addPair(12,8, "active muscle");
			oneUnit.addPair(12,0, "active muscle");
		}
	}

	void addSegments(tgStructure& finalStruct, std::vector<tgStructure> allModules, std::vector<Module> moduleData)
	{
		double dist=1;
		// faces position vectors
		btVector3 position[8]={btVector3(dist,dist,dist) , btVector3(-dist,dist,dist) ,
		    	btVector3(-dist,dist,-dist) , btVector3(dist,dist,-dist) , btVector3(-dist,-dist,-dist) ,
				btVector3(dist,-dist,-dist) , btVector3(dist,-dist,dist) , btVector3(-dist,-dist,dist)};

		const btVector3 center(0, 0, 0);

		// create structures and place them correctly
		for (size_t i = 0; i < moduleData.size(); ++i)
		{
		  tgStructure temp = allModules[i];
		  addActuator(temp,moduleData[i].rotation);
		  tgStructure* const t = new tgStructure(temp);

		  t->addTags(tgString("Unit nb", i ));
		  if(i!=0)
		  {
			  for(int l=0; l<moduleData[i].rotation_directions.size();l++)
			  {
				  t->addRotation(center,moduleData[i].rotation_directions[l], M_PI);
			  }
			  t->move(moduleData[i].position);
		  }
		  // Add a child to the structure
		  finalStruct.addChild(t);
		}
	}

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

    std::vector<Module> readData (int argc, char **argv)
    {
    	//trial input
	//std::cout << m_inputStr << std::endl;
	
	std::vector<Module> moduleData;

	if(argc==0)
	{
		char buf[255];
         getcwd(buf, sizeof(buf));
        // std::cout << "Current working directory is " << buf << std::endl;

		std::ifstream inFile;

                //home/bejjani/Desktop/NTRTsim-master/src/dev/MovingBall_sim/

                inFile.open("./Tensoft/src/dev/MovingBall_sim/input_sim.txt");

		if (!inFile) {
            std::cerr << "Unable to open file input_sim.txt \n";
			exit(1);   // call system to stop
		}
		std::string line="";

		
		std::string buffer="";
		while (std::getline(inFile, line))
		{
			Module temp;

			//loading line
			//std::cout << line << std::endl;
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
			iss >> buffer;
			iss >> temp.rotation;
			iss >> buffer;
			iss >> temp.stiffness;



			moduleData.push_back(temp);
		}
		inFile.close();
	}
        else
	{
		int nbModules=0;
		int buffer=2;
		int datalength=0;
		std::vector<int> connectedModules;
		
		
		for(int i=2; i<argc-1 ;i++)
		{
			if(argv[i+1]==std::string("--"))
			{
				nbModules++;
				//std::cout << "argv: " << i+1 << "dataL" << datalength << std::endl;
				connectedModules.push_back((((i+1-datalength)/2)-5)/2);
				datalength=i;
			}
			//std::cout << argv[i+1] << std::endl;
		}
		for(int i=0;i<nbModules;i++)
		{
			//std::cout << connectedModules[i]<< std::endl;
			Module temp;
			temp.order=atoi(argv[buffer]);
			buffer+=2;
			for(int j=0;j<connectedModules[i];j++)
			{
				temp.connected_modules.push_back(atoi(argv[buffer]));
				buffer+=2;
			}
			for(int j=0;j<connectedModules[i];j++)
			{
				temp.connected_faces.push_back(atoi(argv[buffer]));
				buffer+=2;
			}
			temp.frequency=atof(argv[buffer]);
			buffer+=2;
			temp.amplitude=atof(argv[buffer]);
			buffer+=2;
			temp.phase=atof(argv[buffer]);
			buffer+=2;
			temp.rotation=atoi(argv[buffer]);
			buffer+=2;
			temp.stiffness=atof(argv[buffer]);
			buffer+=2;

			moduleData.push_back(temp);
			
		}
	}
	for(int i=0;i<moduleData.size();i++)
	{
		std::cout << "order " << moduleData[i].order << " connected Modules: ";
		for(int j=0;j<moduleData[i].connected_modules.size();j++)
		{
			std::cout << moduleData[i].connected_modules[j] << " ";
		}
		std::cout << "connected Faces ";
		for(int j=0;j<moduleData[i].connected_faces.size();j++)
		{
			std::cout << moduleData[i].connected_faces[j] << " ";
		}
		std::cout << "freq " << moduleData[i].frequency;
		std::cout << " amp " << moduleData[i].amplitude;
		std::cout << " phase " << moduleData[i].phase;
		std::cout << " rot " << moduleData[i].rotation << std::endl;
		std::cout << " stiff " << moduleData[i].stiffness << std::endl;
	}
	
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
		//std::cout << rotationMatrix << std::endl;
		return convertVectors(rotationMatrix*convertVectors(Vector));
    }


    std::vector<Module> findPositions(std::vector<Module> moduleData)
    {
    	double dist=sqrt(2)*config.rod_space;
    	btVector3 position[8]={btVector3(dist,dist,dist) , btVector3(-dist,dist,dist) ,
    			btVector3(-dist,dist,-dist) , btVector3(dist,dist,-dist) , btVector3(-dist,-dist,-dist) ,
				btVector3(dist,-dist,-dist) , btVector3(dist,-dist,dist) , btVector3(-dist,-dist,dist)};

    	Eigen::Matrix3d R;

    	for(int i=0; i < moduleData.size(); i++)
    	{


    		if(i==0)
    		{
    			moduleData[i].e1 = btVector3(1,0,0);
    			moduleData[i].e2 = btVector3(0,1,0);
    			moduleData[i].e3 = btVector3(0,0,1);

    			moduleData[i].position = btVector3(0,0,0);
    			moduleData[i].parent_face=0;
    		}
    		//std::cout << moduleData[i].connected_modules[0] << std::endl;
    		if(moduleData[i].connected_modules[0]>0)
    		{
    			//std::cout << moduleData[i].connected_modules.size() << std::endl;
    			for(int j=0;j<moduleData[i].connected_modules.size();j++)
				{

    				// the problem lies in the position vector as we should rotate the coord around the relative positions


					R <<    moduleData[i].e1.getX(),
							moduleData[i].e1.getY(),
							moduleData[i].e1.getZ(),

							moduleData[i].e2.getX(),
							moduleData[i].e2.getY(),
							moduleData[i].e2.getZ(),

							moduleData[i].e3.getX(),
							moduleData[i].e3.getY(),
							moduleData[i].e3.getZ();

					btVector3 relativePosition = convertVectors(R.inverse()*convertVectors(position[moduleData[i].connected_faces[j]-1]));

					/*moduleData[moduleData[i].connected_modules[j]-1].e1 =
							rotateAxis(position[moduleData[i].connected_faces[j]-1].normalized(), M_PI, moduleData[i].e1 );
					moduleData[moduleData[i].connected_modules[j]-1].e2 =
							rotateAxis(position[moduleData[i].connected_faces[j]-1].normalized(), M_PI, moduleData[i].e2 );
					moduleData[moduleData[i].connected_modules[j]-1].e3 =
							rotateAxis(position[moduleData[i].connected_faces[j]-1].normalized(), M_PI, moduleData[i].e3 );*/

					moduleData[moduleData[i].connected_modules[j]-1].e1 =
							rotateAxis(relativePosition.normalized(), M_PI, moduleData[i].e1 );
					moduleData[moduleData[i].connected_modules[j]-1].e2 =
							rotateAxis(relativePosition.normalized(), M_PI, moduleData[i].e2 );
					moduleData[moduleData[i].connected_modules[j]-1].e3 =
							rotateAxis(relativePosition.normalized(), M_PI, moduleData[i].e3 );




					moduleData[moduleData[i].connected_modules[j]-1].position = moduleData[i].position
												+convertVectors(R.inverse()*convertVectors(position[moduleData[i].connected_faces[j]-1]));

					//std::cout << "delta vector:" << R.inverse()*convertVectors(position[moduleData[i].connected_faces[j]-1]) << std::endl;

					for(int l=0; l<moduleData[i].rotation_directions.size();l++)
					{

						moduleData[moduleData[i].connected_modules[j]-1].rotation_directions.push_back(moduleData[i].rotation_directions[l]);
					}
					moduleData[moduleData[i].connected_modules[j]-1].rotation_directions.push_back(moduleData[moduleData[i].
							connected_modules[j]-1].position-moduleData[i].position);


					moduleData[moduleData[i].connected_modules[j]-1].parent_face = moduleData[i].connected_faces[j];


				}
    		}
    		//std::cout << std::endl;
    	}

    	return moduleData;
    }

} // namespace

void MovingBallModel::setup(tgWorld& world)
{

	if(m_argc>2)
	{
		m_initPos=atoi(m_argv[1]);
		std::cout << "init pos: "<< m_initPos << std::endl;
	}
	

	//load the data structure

	m_moduleData=readData(m_argc,m_argv);

	m_moduleData=findPositions(m_moduleData);

	// Start creating the structure
	std::vector<tgStructure> allModules;
	for(int i; i < m_moduleData.size();i++)
	{
		tgStructure oneUnit;
		addNodes(oneUnit);
		addPairs(oneUnit,i);
		allModules.push_back(oneUnit);
	}
	


	tgStructure finalStruct;


	addSegments(finalStruct, allModules, m_moduleData);

	//std::cout<< "test" << std::endl;

	//connectUnits(finalStruct);

	connectUnits(finalStruct,m_moduleData);

	//finalStruct.move(btVector3(0, 3*m_moduleData.size(), 0));
	//finalStruct.move(btVector3(0, config.rod_length, 0));
	// Add a rotation. This is needed if the ground slopes too much,
	// otherwise  glitches put a rod below the ground.
	btVector3 rotationPoint = btVector3(0, 0, 0); // origin
	
	btVector3 zAxis = btVector3(0, 1, 0);  // z-axis
	btVector3 yAxis = btVector3(0, 0, 1);  // y-axis
	btVector3 xAxis = btVector3(1, 0, 0);  // x-axis
	
	double straightAngle = M_PI/2;
	double tiltAngle = M_PI/9;

	finalStruct.addRotation(rotationPoint, zAxis, straightAngle);

    finalStruct.move(btVector3(0, 20, 0));

	switch(m_initPos)
	{
		case 0:
		{
			std::cout << "Pos 0" << std::endl;
		} 
		break;
		case 1:
		{
			std::cout << "Pos 1"<< std::endl;
			finalStruct.move(btVector3(0, 20, 0));
			
		}
		break;
		case 2:
		{
			std::cout << "Pos 2"<< std::endl;
			finalStruct.addRotation(rotationPoint, yAxis, -tiltAngle);
		}
		break;
		
	}


/////part for aligning with obstacle with face 2
	/*finalStruct.addRotation(rotationPoint, btVector3(0, 1, 0), M_PI/4-0.15);
	finalStruct.move(btVector3(-30, 0, -18));*/




 ///////part for making the module straight on the ground for face 1
	/*finalStruct.addRotation(rotationPoint, btVector3(1,0,1),0.303889*M_PI+M_PI);

	finalStruct.move(btVector3(0, 40, 0));

	finalStruct.addRotation(rotationPoint, btVector3(1, 0, 0), rotationAngle);

	finalStruct.addRotation(rotationPoint, rotationAxis, rotationAngle);
*/
	//finalStruct.move(btVector3(-20, 0, 0));
/*////part for aligning the module on the ground for face 2
		finalStruct.addRotation(rotationPoint, btVector3(1, 1, 0), rotationAngle);
		finalStruct.addRotation(rotationPoint, btVector3(0, 1, 0), M_PI);
		finalStruct.move(btVector3(0, 10, 0));
		finalStruct.move(btVector3(30, 0, 0));


///////*/

	const tgRod::Config rodConfig(config.radius, config.density, config.friction,
				config.rollFriction, config.restitution);
	const tgRod::Config lightrodConfig(config.radius, config.density/4, config.friction,
					config.rollFriction, config.restitution);
	const tgSphere::Config sphereConfig(config.sphere_radius, config.sphere_density);
	/// @todo acceleration constraint was removed on 12/10/14 Replace with tgKinematicActuator as appropreate
	const tgBasicActuator::Config muscleConfig(config.muscle_stiffness, config.muscle_damping, config.muscle_pretension, config.hist,
						config.muscle_maxTens, config.muscle_targetVelocity);
	const tgBasicActuator::Config connectorConfig(config.connector_stiffness, config.connector_damping, config.connector_pretension, config.hist,
							config.connector_maxTens, config.connector_targetVelocity);
	//////////////////////////

	tgBuildSpec spec;
	for(int i; i < m_moduleData.size();i++)
	{
		std::string stringOrder;
		std::stringstream ss;
		ss << i;
		stringOrder = ss.str();
		const tgBasicActuator::Config pmuscleConfig(m_moduleData[i].stiffness*config.muscle_stiffness, config.muscle_damping,m_moduleData[i].stiffness*config.muscle_pretension, config.hist,
							config.muscle_maxTens, config.muscle_targetVelocity);
		spec.addBuilder("passive muscle"+stringOrder, new tgBasicActuatorInfo(pmuscleConfig));
		
	}
    
	///////////////////////////

	// Create the build spec that uses tags to turn the structure into a real model
	
	spec.addBuilder("rod", new tgRodInfo(rodConfig));
	spec.addBuilder("light rod", new tgRodInfo(lightrodConfig));
	spec.addBuilder("sphere", new tgSphereInfo(sphereConfig));

	
	spec.addBuilder("active muscle", new tgBasicActuatorInfo(muscleConfig));
	spec.addBuilder("connector", new tgBasicActuatorInfo(connectorConfig));

	// Create your structureInfo
	tgStructureInfo structureInfo(finalStruct, spec);

	// Use the structureInfo to build ourselves
	structureInfo.buildInto(*this, world);

    // We could now use tgCast::filter or similar to pull out the models (e.g. actuators)
    // that we want to control.    
    allActuators = tgCast::filter<tgModel, tgBasicActuator> (getDescendants());
    mapActuators(actuatorMap, *this);
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


		/* btVector3 totalCenterOfMassInit(0,0,0);

        // Notify observers (controllers) of the step so that they can take action
		std::vector<tgBaseRigid*> allRods = this->find<tgBaseRigid>("rod");
		btVector3 minPositionX=allRods[0]->centerOfMass();
		btVector3 maxPositionX=allRods[0]->centerOfMass();

		btVector3 totalCenterOfMassFinal(0,0,0);
		for (int j=0; j<allRods.size();j++)
		{
			if(allRods[j]->centerOfMass().getX()<minPositionX.getX())
			{
				minPositionX=allRods[j]->centerOfMass();
			}
			if(allRods[j]->centerOfMass().getX()>maxPositionX.getX())
			{
				maxPositionX=allRods[j]->centerOfMass();
			}
			//std::cout << "rod position: " << allRods[j]->centerOfMass()  << std::endl;
			totalCenterOfMassFinal+=allRods[j]->centerOfMass();
		}

		totalCenterOfMassFinal = totalCenterOfMassFinal/allRods.size();

		totalCenterOfMassFinal.setY(0);

		double distance = (totalCenterOfMassFinal-totalCenterOfMassInit).length();

		std::cout<<distance<<std::endl; */

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

