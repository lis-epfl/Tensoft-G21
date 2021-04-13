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
 * either express or implied. See the License for the specfic language
 * governing permissions and limitations under the License.
*/

/**
 * @author Jean Marc Bejjani, Daniele Bissoli, Enrico Zardini
 * @copyright Copyright (C) 2019 LIS
 * $Id$
 */

// The C++ Standard Library
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <cstdio>
#include <unistd.h>
#include <math.h>

#include <Eigen/Dense>

// Application library
#include "robotModel.h"

namespace
{
    // see tgBasicActuator and tgRod for a description of these rod parameters
    // (specifically, those related to the motor moving the strings.)
    // NOTE that any parameter that depends on units of length will scale
    // with the current gravity scaling. E.g., with gravity as 98.1,
    // the length units below are in decimeters.

    // Note: This current model of the SUPERball rod is 1.5m long by 3 cm radius,
    // which is 0.00424 m^3.
    // For SUPERball v1.5, mass = 3.5kg per strut, which comes out to
    // 0.825 kg / (decimeter^3).

    // similarly, frictional parameters are for the tgRod objects.

    const struct GlobalConfig {
        double friction;                        // unit-less
        double rollFriction;                    // unit-less
        double restitution;
        double lowStiffness;                    // relative
        double midStiffness;                    // relative
        double highStiffness;                   // relative
        double medCharacterizationStiffness;    // relative
        bool   hist;                            // history logging
    } globalSettings {
        1.0,
        0.005,
        0.8,
        0.1,
        0.5,
        1.0,
        0.31,
        0
    };

    const struct RodConfig {
        double density;
        double radius;
        double length;
        double space;
    } rodSettings {
        1.73,
        0.15,
        8.2,
        3.75
    };

    const struct LightRodConfig {
        double density;     // [g/cm^3]
        double radius;
        double length;      // [cm]
    } lightRodSettings {
    #ifdef TEST_SINGLE_MODULE
        0,
    #else
        10.1,
    #endif
        0.15,
        1.79 // distance from the rod where the actuator is connected
    };

    const struct ConnectorConfig {
        double tensileStiffness;        // [nM/cm]
        double compressiveStiffness;    // [nM/cm]
        double lengthRingSide;          // [cm]
        double lengthRingCenter;        // [cm]
        double preStretch; // pretension
        double damping;
        double maxTension;
        double targetVelocity;
    } connectorSettings {
        438000,
        66000,
        0.25,
        1.0,
        0.0,
        20.0,               // connector damping
        100000,             // connector max tension
        10,                 // connector target Velocity
    };

    const struct MuscleConfig {
        double maxStiffness;
        double damping;
        double pretension;
        double activePretension;
        double maxActiveTension;
        double maxPassiveTension;
        double targetVelocity; // force vs velocity (max responsiveness)
    } muscleSettings {
        11520.0,
        70.0,
        11520.0*5*0.65, // maxStiff by 65% of 5 cable len in cm [mN]
        11520.0*5*0.01, // maxStiff by  1% of 5 cable len in cm [mN]
        6540,
        100000,
        25
    };

    const struct ActuatedConfig {
        double actuatorDensity;     // [g/cm^3] -> assign to half rod
        double sphereRadius;        // [cm]
        double sphereDensity;       // [g/cm^3]
    } actuatedSettings {
    #ifdef TEST_SINGLE_MODULE
        0,
    #else
        27.6,
    #endif
        0.4,
    #ifdef TEST_SINGLE_MODULE
        0,
    #else
        10.1,
    #endif
    };

    const struct ServoConfig {
        double length;
        double radius;
        double density;    // [g/cm^3]
        double x_offset;
        double y_offset;
    } servoSettings {
        1.7,
        0.55,
        #ifdef TEST_SINGLE_MODULE
            0,
        #else
            0.0029,
        #endif
        0.75,
        0.4
    };
} // namespace

robotModel::robotModel() : tgModel() {
    mov_dir = 1;
    rotation_angle = 0.0;
}

robotModel::robotModel(int argc, char *argv[]) :
m_argc(argc), m_argv(argv), m_initPos(0) {
    activePreStretches = { 0.0, 0.0, 0.0, 0.0, 0.0 };
    mov_dir = 1;
    rotation_angle = 0.0;
}

robotModel::robotModel(int argc, char *argv[], std::vector<double>& activePreStretches) :
m_argc(argc), m_argv(argv), m_initPos(0), activePreStretches(activePreStretches) {
    mov_dir = 1;
    rotation_angle = 0.0;
}

/**
 * Anonymous namespace for helper functions
 */
namespace
{
    // note faces nodes order are modified wrt the correct ones to allow correct
    // alignment when faces of upper part are connected to modules of lower part
    const int faces[8][3] = {
            {3, 7, 9},
            {7, 1, 8},
            {1, 6, 10},
            {6, 3, 11},
            {4, 0, 10},
            {2, 4, 11},
            {5, 2, 9},
            {0, 5, 8}
    };
    // map a face ID given by the position in the array -1 to its parallel face in the module
    const int facesMap[8] = { 5, 6, 7, 8, 1, 2, 3, 4 };

    void addNodes(tgStructure& oneUnit)
    {
        const double half_length = rodSettings.length / 2;

        oneUnit.addNode(-rodSettings.space, -half_length, 0, "rod_node n0");            // 0    real:0A connected to sphere
        oneUnit.addNode(-rodSettings.space,  half_length, 0, "rod_node n1");            // 1    real:0B
        oneUnit.addNode( rodSettings.space, -half_length, 0, "rod_node n2");            // 2    real:9
        oneUnit.addNode( rodSettings.space,  half_length, 0, "rod_node n3");            // 3    real:10 connected to sphere
        oneUnit.addNode(0, -rodSettings.space, -half_length, "rod_node n4");            // 4    real:6  connected to sphere
        oneUnit.addNode(0, -rodSettings.space,  half_length, "rod_node n5");            // 5    real:5
        oneUnit.addNode(0,  rodSettings.space, -half_length, "rod_node n6");            // 6    real:8
        oneUnit.addNode(0,  rodSettings.space,  half_length, "rod_node n7");            // 7    real:7  connected to sphere
        oneUnit.addNode(-half_length, 0,  rodSettings.space, "rod_node n8");            // 8    real:3
        oneUnit.addNode( half_length, 0,  rodSettings.space, "rod_node n9");            // 9    real:4	connected to sphere
        oneUnit.addNode(-half_length, 0, -rodSettings.space, "rod_node n10");           // 10   real:1  connected to sphere
        oneUnit.addNode( half_length, 0, -rodSettings.space, "rod_node n11");           // 11   real:2
        oneUnit.addNode(-rodSettings.space+lightRodSettings.length, 0, 0, "sphere");    // 12   sphere (point of compression)
        oneUnit.addNode(-rodSettings.space,  0, 0, "sphere_conn");                      // 13   rigid   connection to sphere

        #ifndef NO_SERVO
            oneUnit.addNode(-rodSettings.space+servoSettings.x_offset,
                            -3*servoSettings.radius+servoSettings.y_offset, 0, "servo_ext_node n1");      // 14 servo external node
            oneUnit.addNode(-rodSettings.space-(servoSettings.length-servoSettings.x_offset),
                            -3*servoSettings.radius+servoSettings.y_offset, 0, "servo_ext_node n2");      // 15 servo external node
            oneUnit.addNode(-rodSettings.space+servoSettings.x_offset,
                            servoSettings.y_offset-servoSettings.radius, 0, "servo_ext_node n3");         // 16 servo external node
            oneUnit.addNode(-rodSettings.space-(servoSettings.length-servoSettings.x_offset),
                            servoSettings.y_offset-servoSettings.radius, 0, "servo_int_node n4");         // 17 servo external node
            oneUnit.addNode(-rodSettings.space+servoSettings.x_offset-servoSettings.radius,
                            -3*servoSettings.radius+servoSettings.y_offset, 0, "servo_int_node n1");                         // 18 servo internal node
            oneUnit.addNode(-rodSettings.space-(servoSettings.length-servoSettings.x_offset)+servoSettings.radius,
                            -3*servoSettings.radius+servoSettings.y_offset, 0, "servo_int_node n2");                         // 19 servo internal node
            oneUnit.addNode(-rodSettings.space+servoSettings.x_offset-servoSettings.radius,
                            servoSettings.y_offset-servoSettings.radius, 0, "servo_int_node n3");                            // 20 servo internal node
            oneUnit.addNode(-rodSettings.space-(servoSettings.length-servoSettings.x_offset)+servoSettings.radius,
                            servoSettings.y_offset-servoSettings.radius, 0, "servo_int_node n4");                            // 21 servo internal node
        #endif

#ifdef COM_COLLECTION
            oneUnit.addNode(-half_length-rodSettings.radius, -half_length, 0, "pivot_node");      // 14 or 22
            oneUnit.addNode(-half_length-rodSettings.radius,  half_length, 0, "pivot_node");      // 15 or 23
#endif
/*
        // original precision
        std::streamsize precision = std::cout.precision();

        // front(bottom)-back(up)
        tgNode n_0b = oneUnit.findNode("rod_node n1");
        tgNode n_2 = oneUnit.findNode("rod_node n2");

        // left(up)-right(up)
        tgNode n_11 = oneUnit.findNode("rod_node n11");
        tgNode n_5 = oneUnit.findNode("rod_node n5");

        // left(bottom)-right(bottom)
        tgNode n_6 = oneUnit.findNode("rod_node n6");
        tgNode n_8 = oneUnit.findNode("rod_node n8");

        std::cout << "Distance 0B-2: " << std::setprecision(10) << n_0b.distance(n_2) << std::endl;
        std::cout << "Distance 11-5: " << n_11.distance(n_5) << std::endl;
        std::cout << "Distance 6-8: " << n_6.distance(n_8) << std::endl;

        std::cout << "Distance 0B-11: " << n_0b.distance(n_11) << std::endl;
        std::cout << "Distance 0B-5: " << n_0b.distance(n_5) << std::endl;
        std::cout << "Distance 0B-6: " << n_0b.distance(n_6) << std::endl;
        std::cout << "Distance 0B-8: " << n_0b.distance(n_8) << std::endl;

        std::cout << "Distance 2-11: " << n_2.distance(n_11) << std::endl;
        std::cout << "Distance 2-5: " << n_2.distance(n_5) << std::endl;
        std::cout << "Distance 2-6: " << n_2.distance(n_6) << std::endl;
        std::cout << "Distance 2-8: " << n_2.distance(n_8) << std::endl;

        std::cout << "Distance 11-6: " << n_11.distance(n_6) << std::endl;
        std::cout << "Distance 11-8: " << n_11.distance(n_8) << std::endl;

        std::cout << "Distance 5-6: " << n_5.distance(n_6) << std::endl;
        std::cout << "Distance 5-8: " << n_5.distance(n_8) << std::setprecision(precision) << std::endl;
*/
    }

    void addPairs(tgStructure& oneUnit, int moduleOrder)
    {
        std::string modId = std::to_string(moduleOrder);

        /* Note on tags:
         *  - rX -> rod id
         *  - mX -> module id
         */

        //rigid structure
        oneUnit.addPair( 0, 13, "half rod r0 m" + modId); // this is a half rod, on which the actuator is attached
        oneUnit.addPair(13,  1, "half rod r1 m" + modId); // this is a half rod, on which the actuator is attached
        oneUnit.addPair( 2,  3, "rod r2 m" + modId);
        oneUnit.addPair( 4,  5, "rod r3 m" + modId);
        oneUnit.addPair( 6,  7, "rod r4 m" + modId);
        oneUnit.addPair( 8,  9, "rod r5 m" + modId);
        oneUnit.addPair(10, 11, "rod r6 m" + modId);
        oneUnit.addPair(12, 13, "light rod m" + modId); // connects rods halves to the actuator (the sphere)

        #ifndef NO_SERVO
            oneUnit.addPair(14, 18, "servo hr1_p1 m"+modId);
            oneUnit.addPair(18, 19, "servo hr1_p2 m"+modId);
            oneUnit.addPair(19, 15, "servo hr1_p3 m"+modId);
            oneUnit.addPair(16, 20, "servo hr2_p1 m"+modId);
            oneUnit.addPair(20, 21, "servo hr2_p2 m"+modId);
            oneUnit.addPair(21, 17, "servo hr2_p3 m"+modId);
            oneUnit.addPair(18, 20, "servo vr1 m"+modId);
            oneUnit.addPair(19, 21, "servo vr2 m"+modId);
            oneUnit.addPair(13, 20, "servo_rod_connection m"+modId); // connects servo to the rod (half rods joint)
        #endif

        #ifdef COM_COLLECTION
            #ifdef NO_SERVO
                oneUnit.addPair(14, 15, "pivot m" + modId); // only for measurements purposes
            #else
                oneUnit.addPair(22, 23, "pivot m" + modId); // only for measurements purposes
            #endif
        #endif

        //flexible structure
        oneUnit.addPair(0, 4,  "passive muscle"+modId);
        oneUnit.addPair(0, 5,  "passive muscle"+modId);
        oneUnit.addPair(0, 8,  "passive muscle"+modId);
        oneUnit.addPair(0, 10, "passive muscle"+modId);

        oneUnit.addPair(1, 6,  "passive muscle"+modId);
        oneUnit.addPair(1, 7,  "passive muscle"+modId);
        oneUnit.addPair(1, 8,  "passive muscle"+modId);
        oneUnit.addPair(1, 10, "passive muscle"+modId);

        oneUnit.addPair(2, 4,  "passive muscle"+modId);
        oneUnit.addPair(2, 5,  "passive muscle"+modId);
        oneUnit.addPair(2, 9,  "passive muscle"+modId);
        oneUnit.addPair(2, 11, "passive muscle"+modId);

        oneUnit.addPair(3, 7,  "passive muscle"+modId);
        oneUnit.addPair(3, 6,  "passive muscle"+modId);
        oneUnit.addPair(3, 9,  "passive muscle"+modId);
        oneUnit.addPair(3, 11, "passive muscle"+modId);

        oneUnit.addPair(4, 10, "passive muscle"+modId);
        oneUnit.addPair(4, 11, "passive muscle"+modId);

        oneUnit.addPair(5, 8,  "passive muscle"+modId);
        oneUnit.addPair(5, 9,  "passive muscle"+modId);

        oneUnit.addPair(6, 10, "passive muscle"+modId);
        oneUnit.addPair(6, 11, "passive muscle"+modId);

        oneUnit.addPair(7, 8,  "passive muscle"+modId);
        oneUnit.addPair(7, 9,  "passive muscle"+modId);
    }

    void addActuator(tgStructure& oneUnit)
    {
        #ifndef NO_ACTIVE_CABLES
            oneUnit.addPair(12, 10, "active muscle am1");   // 1
            oneUnit.addPair(12,  9, "active muscle am4");   // 4
            oneUnit.addPair(12,  4, "active muscle am6");   // 6
            oneUnit.addPair(12,  7, "active muscle am7");   // 7
            oneUnit.addPair(12,  3, "active muscle am10");  // 10
            oneUnit.addPair(12,  0, "steady muscle sm0");
        #endif
        #ifdef COM_COLLECTION
            #ifdef NO_SERVO
                oneUnit.addPair(14,  0, "pivot_cable pc1");
                oneUnit.addPair(15,  1, "pivot_cable pc2");
            #else
                oneUnit.addPair(22,  0, "pivot_cable pc1");
                oneUnit.addPair(23,  1, "pivot_cable pc2");
            #endif
        #endif
    }

    void addSegments(tgStructure& finalStruct,
            std::vector<tgStructure>& allModules, const std::vector<Module>& moduleData)
    {
        const btVector3 center(0, 0, 0);

        // create structures and place them correctly
        for (size_t i = 0; i < moduleData.size(); ++i)
        {
            tgStructure temp = allModules[i];
            addActuator(temp);
            tgStructure* const t = new tgStructure(temp);

            t->addTags(tgString("Unit nb", i ));

            // rotate the module so that it is always facing the same
            // direction, even in the case of different connected face
            const btVector3 axes1(1, 0, 0);
            const btVector3 axes2(0, 1, 0);
            const btVector3 axes3(0, 0, 1);

            switch (moduleData[i].connected_faces[0]) {
                case 2:
                    t->addRotation(center, axes2, M_PI/2);
                    break;
                case 3:
                    t->addRotation(center, axes2, M_PI);
                    break;
                case 4:
                    t->addRotation(center, axes2, -M_PI/2);
                    break;
                case 5:
                    t->addRotation(center, axes1, M_PI);
                    t->addRotation(center, axes2, M_PI/2);
                    break;
                case 6:
                    t->addRotation(center, axes1, M_PI);
                    break;
                case 7:
                    t->addRotation(center, axes1, M_PI);
                    t->addRotation(center, axes2, -M_PI/2);
                    break;
                case 8:
                    t->addRotation(center, axes1, M_PI);
                    t->addRotation(center, axes2, M_PI);
                    break;

                case 1:
                default:
                    // do not apply any rotation for this face
                    break;
            }

            if (i > 0)
            {
                for(int l = 0; l < moduleData[i].rotation_directions.size(); l++)
                {
                    t->addRotation(center, moduleData[i].rotation_directions[l], M_PI);
                }
                t->move(moduleData[i].position);
            }
            // Add a child to the structure
            finalStruct.addChild(t);
        }
    }

    void getFace(int connectedFace, int face[]) {
        if (connectedFace > 0) {
            // -1 added to specify correct index, since faces ID start from 1
            for (int j = 0; j < 3; j++) {
                face[j] = faces[facesMap[connectedFace-1]-1][j];
            }
        }
        else {
            // by default return the parallel face to the 1 (the 5th)
            for (int j = 0; j < 3; j++) {
                face[j] = faces[4][j];
            }
        }
    }

    void connectUnits(tgStructure& finalStruct, const std::vector<Module>& moduleData)
    {
        const std::vector<tgStructure*> children = finalStruct.getChildren();
        for (size_t i = 0; i < children.size(); ++i)
        {
            tgNodes n0 = children[i]->getNodes();
            for (size_t j = 0; j < moduleData[i].connected_modules.size(); j++)
            {
                if (moduleData[i].connected_modules[0] != 0 && i+1 < children.size())
                {
                    tgNodes n1 = children[moduleData[i].connected_modules[j]-1]->getNodes();

                    int face[3] = {0, 0, 0};
                    // STRONG ASSUMPTION: a module has only a single child connected
                    getFace(moduleData[i+1].connected_faces[0], face);

                    switch (moduleData[i].connected_faces[j]) {
                        case 1: {
                            finalStruct.addPair(n0[7], n1[face[0]], "connector");
                            finalStruct.addPair(n0[3], n1[face[1]], "connector");
                            finalStruct.addPair(n0[9], n1[face[2]], "connector");
                            break;
                        }
                        case 2: {
                            finalStruct.addPair(n0[1], n1[face[0]], "connector");
                            finalStruct.addPair(n0[7], n1[face[1]], "connector");
                            finalStruct.addPair(n0[8], n1[face[2]], "connector");
                            break;
                        }
                        case 3: {
                            finalStruct.addPair(n0[6],  n1[face[0]], "connector");
                            finalStruct.addPair(n0[1],  n1[face[1]], "connector");
                            finalStruct.addPair(n0[10], n1[face[2]], "connector");
                            break;
                        }
                        case 4: {
                            finalStruct.addPair(n0[3],  n1[face[0]], "connector");
                            finalStruct.addPair(n0[6],  n1[face[1]], "connector");
                            finalStruct.addPair(n0[11], n1[face[2]], "connector");
                            break;
                        }
                        case 5: {
                            finalStruct.addPair(n0[0],  n1[face[0]], "connector");
                            finalStruct.addPair(n0[4],  n1[face[1]], "connector");
                            finalStruct.addPair(n0[10], n1[face[2]], "connector");
                            break;
                        }
                        case 6: {
                            finalStruct.addPair(n0[4],  n1[face[0]], "connector");
                            finalStruct.addPair(n0[2],  n1[face[1]], "connector");
                            finalStruct.addPair(n0[11], n1[face[2]], "connector");
                            break;
                        }
                        case 7: {
                            finalStruct.addPair(n0[2], n1[face[0]], "connector");
                            finalStruct.addPair(n0[5], n1[face[1]], "connector");
                            finalStruct.addPair(n0[9], n1[face[2]], "connector");
                            break;
                        }
                        case 8: {
                            finalStruct.addPair(n0[5], n1[face[0]], "connector");
                            finalStruct.addPair(n0[0], n1[face[1]], "connector");
                            finalStruct.addPair(n0[8], n1[face[2]], "connector");
                            break;
                        }
                        default:
                        break;
                    }
                }
            }
        }
    }

    btVector3 convertVectors(const Eigen::Vector3d& Vector)
    {
        return btVector3(Vector.x(), Vector.y(), Vector.z());
    }

    Eigen::Vector3d convertVectors(const btVector3& Vector)
    {
        return Eigen::Vector3d(Vector.getX(), Vector.getY(), Vector.getZ());
    }

    void mapActuators(robotModel::ActuatorMap& actuatorMap, tgModel& model)
    {
        // Note that tags don't need to match exactly, we could create
        // supersets if we wanted to
        actuatorMap["motor"]  = model.find<tgBasicActuator>("active muscle");
    }

    void getFaceRods(int module_id, std::vector<tgRod*>& faceRods, int faceId, tgModel& model, const std::vector<Module>& moduleData) {
        int face[3] = {0, 0, 0};
        for (int i = 0; i < 3; i++) {
            face[i] = faces[faceId-1][i];
        }

        for (auto node: face) {
            if (node == 0 || node == 1) {
                std::vector<tgRod*> rod0 = model.find<tgRod>("rod r0 m"+std::to_string(module_id));
                std::vector<tgRod*> rod1 = model.find<tgRod>("rod r1 m"+std::to_string(module_id));
                faceRods.insert(faceRods.begin(), rod0.begin(), rod0.end());
                faceRods.insert(faceRods.begin(), rod1.begin(), rod1.end());
            } else {
                int id = node/2 + 1;
                std::string id_str = std::to_string(id);
                std::vector<tgRod*> rod = model.find<tgRod>("rod r"+id_str+" m"+std::to_string(module_id));
                faceRods.insert(faceRods.end(), rod.begin(), rod.end());
            }
        }
    }

    void getFrontFaceAndParallel(std::vector<tgRod*>& frontFace, std::vector<tgRod*>& frontFaceParallel, int mov_dir,
            tgModel& model, const std::vector<Module>& moduleData) {
        int module_id, frontFaceId, parallelFaceId;

        if (mov_dir == 1) {
            module_id = 0;
            frontFaceId = facesMap[moduleData[0].connected_faces[0]-1];
            parallelFaceId = moduleData[0].connected_faces[0];
        } else {
            module_id = moduleData.size()-1;
            frontFaceId = moduleData[moduleData.size()-1].connected_faces[0];
            parallelFaceId = facesMap[moduleData[moduleData.size()-1].connected_faces[0]-1];;
        }

        getFaceRods(module_id, frontFace, frontFaceId, model, moduleData);
        getFaceRods(module_id, frontFaceParallel, parallelFaceId, model, moduleData);
    }

    void trace(const tgStructureInfo& structureInfo, tgModel& model)
    {
        std::cout << "StructureInfo:" << std::endl
                  << structureInfo    << std::endl
                  << "Model: "        << std::endl
                  << model            << std::endl;
    }

    std::vector<Module> readData(int argc, char **argv)
    {
        std::vector<Module> moduleData;

        if (argc == 1)
        {
            std::ifstream inFile;
            inFile.open(argv[0]);

            if (!inFile.is_open()) {
                std::cerr << "Unable to open file "<< argv[0] << std::endl;
                exit(1);   // call system to stop
            }

            std::string line = "";
            std::string buffer = "";
            while (std::getline(inFile, line))
            {
                Module temp;
                std::cout << line << std::endl;
                // loading line
                std::istringstream iss(line);
                // skipping "order:"
                iss >> buffer;

                if (buffer == "fitness:") {
                    // stop reading because the module conf is finished
                    break;
                }
                // loading order
                iss >> temp.order;
                // skipping "connectedModules"
                iss >> buffer;
                // loading first connected modules
                iss >> buffer;

                // loading each connected modules
                while (buffer != "connectedFaces:")
                {
                    temp.connected_modules.push_back(atoi(buffer.c_str()));
                    iss >> buffer;
                }

                // loading first connected face
                iss >> buffer;

                // loading each connected face
                while (buffer != "freq:")
                {
                    temp.connected_faces.push_back(atoi(buffer.c_str()));
                    iss >> buffer;
                }
                std::cout << std::endl;

                // loading actuator data
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
            // TODO: maybe a smarter manner for reading the parameters is possible - no time right now!!
            int nbModules = 0;
            int buffer = 1; // 1 -> skip initial value (the position from which robot starts)
            int datalength = 0;
            std::vector<int> connectedModules;

            for (int i = buffer; i < argc-1; i++)
            {
                if (argv[i+1] == std::string("--"))
                {
                    nbModules++;
                    connectedModules.push_back((((i+1-datalength)/2)-5)/2);
                    datalength = i;
                }
            }
            for (int i = 0; i < nbModules; i++)
            {
                Module temp;
                temp.order = atoi(argv[buffer]);
                buffer += 2;

                for (int j = 0; j < connectedModules[i]; j++)
                {
                    temp.connected_modules.push_back(atoi(argv[buffer]));
                    buffer += 2;
                }
                for (int j = 0; j < connectedModules[i]; j++)
                {
                    temp.connected_faces.push_back(atoi(argv[buffer]));
                    buffer += 2;
                }

                temp.frequency = atof(argv[buffer]);
                buffer += 2;
                temp.amplitude = atof(argv[buffer]);
                buffer += 2;
                temp.phase = atof(argv[buffer]);
                buffer += 2;
                temp.rotation = atoi(argv[buffer]);
                buffer += 2;
                temp.stiffness = atof(argv[buffer]);
                buffer += 2;

                moduleData.push_back(temp);
            }
        }
        for (int i = 0; i < moduleData.size(); i++)
        {
            std::cout << "order " << moduleData[i].order << " connected Modules: ";
            for (int j = 0; j < moduleData[i].connected_modules.size(); j++)
            {
                std::cout << moduleData[i].connected_modules[j] << " ";
            }
            std::cout << "connected Faces ";
            for (int j = 0; j < moduleData[i].connected_faces.size(); j++)
            {
                std::cout << moduleData[i].connected_faces[j] << " ";
            }

            std::cout << "freq " << moduleData[i].frequency;
            std::cout << " amp " << moduleData[i].amplitude;
            std::cout << " phase " << moduleData[i].phase;
            std::cout << " rot " << moduleData[i].rotation;
            std::cout << " stiff " << moduleData[i].stiffness << std::endl;
        }

        return moduleData;
    }

    btVector3 rotateAxis(const btVector3& Axis, double angle, const btVector3& Vector)
    {
        Eigen::Matrix3d rotationMatrix;
        rotationMatrix <<
            cos(angle) + pow(Axis.getX(), 2) * (1-cos(angle)),
            Axis.getX() * Axis.getY() * (1-cos(angle)) - Axis.getZ() * sin(angle),
            Axis.getX() * Axis.getZ() * (1-cos(angle)) + Axis.getY() * sin(angle),

            Axis.getY() * Axis.getX() * (1-cos(angle)) + Axis.getZ() * sin(angle),
            cos(angle) + pow(Axis.getY(), 2) * (1-cos(angle)),
            Axis.getY() * Axis.getZ() * (1-cos(angle)) - Axis.getX() * sin(angle),

            Axis.getX() * Axis.getZ() * (1-cos(angle)) - Axis.getY() * sin(angle),
            Axis.getY() * Axis.getZ() * (1-cos(angle)) + Axis.getX() * sin(angle),
            cos(angle) + pow(Axis.getZ(), 2) * (1-cos(angle));

        return convertVectors(rotationMatrix*convertVectors(Vector));
    }

    void findPositions(std::vector<Module>& moduleData)
    {
        double dist = sqrt(2) * rodSettings.space;
        // guess -> these represents the central position
        // of each Tensegrity face (the equilateral triangles)
        btVector3 position[8] = {
            btVector3( dist,  dist,  dist),
            btVector3(-dist,  dist,  dist),
            btVector3(-dist,  dist, -dist),
            btVector3( dist,  dist, -dist),
            btVector3(-dist, -dist, -dist),
            btVector3( dist, -dist, -dist),
            btVector3( dist, -dist,  dist),
            btVector3(-dist, -dist,  dist)
        };

        Eigen::Matrix3d R;

        for (int i = 0; i < moduleData.size(); i++)
        {
            if (i == 0)
            {
                moduleData[i].e1 = btVector3(1,0,0);
                moduleData[i].e2 = btVector3(0,1,0);
                moduleData[i].e3 = btVector3(0,0,1);

                moduleData[i].position = btVector3(0,0,0);
                moduleData[i].parent_face = 0;
            }

            if (moduleData[i].connected_modules[0] > 0)
            {
                for (int j = 0; j < moduleData[i].connected_modules.size(); j++)
                {
                    // the problem lies in the position vector
                    // as we should rotate the coordinates
                    // around the relative positions
                    R << moduleData[i].e1.getX(),
                         moduleData[i].e1.getY(),
                         moduleData[i].e1.getZ(),

                         moduleData[i].e2.getX(),
                         moduleData[i].e2.getY(),
                         moduleData[i].e2.getZ(),

                         moduleData[i].e3.getX(),
                         moduleData[i].e3.getY(),
                         moduleData[i].e3.getZ();

                    btVector3 relativePosition =
                            convertVectors(R.inverse()*convertVectors(position[moduleData[i].connected_faces[j]-1]));

                    moduleData[moduleData[i].connected_modules[j]-1].e1 =
                            rotateAxis(relativePosition.normalized(), M_PI, moduleData[i].e1);
                    moduleData[moduleData[i].connected_modules[j]-1].e2 =
                            rotateAxis(relativePosition.normalized(), M_PI, moduleData[i].e2);
                    moduleData[moduleData[i].connected_modules[j]-1].e3 =
                            rotateAxis(relativePosition.normalized(), M_PI, moduleData[i].e3);

                    // always move the module of the same distance (and in the same direction)
                    moduleData[moduleData[i].connected_modules[j]-1].position =
                            moduleData[i].position + position[0];

                    for (int l = 0; l < moduleData[i].rotation_directions.size(); l++)
                    {
                        moduleData[moduleData[i].connected_modules[j]-1].rotation_directions.push_back(
                                moduleData[i].rotation_directions[l]
                        );
                    }

                    moduleData[moduleData[i].connected_modules[j]-1].rotation_directions.push_back(
                            moduleData[moduleData[i].connected_modules[j]-1].position - moduleData[i].position
                    );

                    moduleData[moduleData[i].connected_modules[j]-1].parent_face = moduleData[i].connected_faces[j];
                }
            }
        }
    }

    void addMarkers(tgStructure& structure, robotModel& model, const std::vector<Module>& modulesData)
    {
        /* NOTE: nonetheless the many efforts, this functions does not properly
                 connect the markers in the proper position for all the robot modules. */

        btVector3 colors[12] = {
            btVector3(1.f, 0.f, 0.f),
            btVector3(1.f, 1.0f, 0.f),
            btVector3(0.f, 0.f, 1.f),
            btVector3(0.f, 1.f, 1.f),
            btVector3(0.59f, 0.29f, 0.f),
            btVector3(1.f, 0.5f, 0.f),
            btVector3(0.f, 0.59f, 0.29f),
            btVector3(0.f, 1.f, 0.f),
            btVector3(0.f, 0.f, 0.f),
            btVector3(1.f, 1.f, 1.f),
            btVector3(0.42f, 0.15f, 0.75f),
            btVector3(1.0f, 0.71f, 0.76f)
        };

        const std::vector<tgStructure*> modules = structure.getChildren();

        for (int kd = 0; kd < modules.size(); kd++) {
            auto module = modules[kd];


            auto components = module->getPairs().getPairs();
            auto nodes = module->getNodes().getNodes();

            int i = 0;
            for (auto &component : components) {
                if (component.hasTag("rod") && !component.hasTag("light")) {

                    std::string strTags = component.getTags().joinTags(" ");
                    btRigidBody *body1 = model.find<tgBaseRigid>(strTags)[0]->getPRigidBody();

                    if (component.hasTag("r0") || component.hasTag("r1") || component.hasTag("r2")) {
                        component.setFrom(component.getFrom() - body1->getCenterOfMassPosition());
                        component.setTo(component.getTo() - body1->getCenterOfMassPosition());
                    }

                    if (component.hasTag("r3")) {
                        component.setFrom(component.getFrom() - body1->getCenterOfMassPosition());
                        component.setTo(component.getTo() - body1->getCenterOfMassPosition());
                        component.addRotation(btVector3(0, 0, 0), modulesData[kd].e3, M_PI / 2);
                    }
                    if (component.hasTag("r4")) {
                        component.setFrom(component.getFrom() - body1->getCenterOfMassPosition());
                        component.setTo(component.getTo() - body1->getCenterOfMassPosition());
                        component.addRotation(btVector3(0, 0, 0), modulesData[kd].e3, M_PI / 2);
                    }
                    if (component.hasTag("r5")) {
                        component.addRotation(body1->getCenterOfMassPosition(), modulesData[kd].e1, M_PI/2);
                        component.setFrom(component.getFrom() - body1->getCenterOfMassPosition());
                        component.setTo(component.getTo() - body1->getCenterOfMassPosition());
                    }
                    if (component.hasTag("r6")) {
                        component.addRotation(body1->getCenterOfMassPosition(), btVector3(1, 0, 0), M_PI/2);
                        component.setFrom(component.getFrom() - body1->getCenterOfMassPosition());
                        component.setTo(component.getTo() - body1->getCenterOfMassPosition());
                    }

                    if (!component.hasTag("r1")) {
                        abstractMarker marker1(body1, component.getFrom(), colors[i], i);
                        model.addMarker(marker1);
                        i++;
                    }
                    if (!component.hasTag("r0")) {
                        abstractMarker marker2(body1, component.getTo(), colors[i], i);
                        model.addMarker(marker2);
                        i++;
                    }
                }
            }
        }
    }
} // namespace

void robotModel::setup(tgWorld& world)
{
    if (m_argc > 1)
    {
        m_initPos = atoi(m_argv[0]);
        std::cout << "init pos: "<< m_initPos << std::endl;
    }

    //load the data structure
    m_moduleData = readData(m_argc, m_argv);
    findPositions(m_moduleData);

    // Start creating the structure
    std::vector<tgStructure> allModules;
    for (int i = 0; i < m_moduleData.size(); i++)
    {
        tgStructure oneUnit;
        addNodes(oneUnit);
        addPairs(oneUnit, i);
        allModules.push_back(oneUnit);
    }

#ifndef MEASURE_CABLES
    tgStructure finalStruct;
#endif
    addSegments(finalStruct, allModules, m_moduleData);
    connectUnits(finalStruct, m_moduleData);

    // Add a rotation. This is needed if the ground slopes too much,
    // otherwise glitches put a rod below the ground.
    btVector3 rotationPoint = btVector3(0, 0, 0); // origin
    btVector3 zAxis = btVector3(0, 1, 0);  // z-axis
    btVector3 yAxis = btVector3(0, 0, 1);  // y-axis
    btVector3 xAxis = btVector3(1, 0, 0);  // x-axis

    double straightAngle = M_PI/2;
    double tiltAngle = -M_PI/9;

    // place the robot parallel to the terrain
    finalStruct.addRotation(rotationPoint, yAxis, -M_PI/4);

    if(std::abs(rotation_angle - 0.0) >= std::numeric_limits<double>::epsilon()) {
        finalStruct.addRotation(rotationPoint, zAxis, rotation_angle);
    }

    // place the robot just above the ground to avoid
    // having any rod stuck in the horizontal plane
    finalStruct.move(btVector3(0, rodSettings.length, 0));

    switch(m_initPos)
    {
        case 0: {
            std::cout << "Pos 0" << std::endl;
            #ifdef TEST_SINGLE_MODULE
                // NOTE: this position is only for simulation of a single module
                finalStruct.move(btVector3(0, -rodSettings.length, 0));
                finalStruct.addRotation(rotationPoint, yAxis, M_PI/4);
                finalStruct.addRotation(finalStruct.getCentroid(), xAxis, M_PI/2);
                finalStruct.move(btVector3(0, 1.2*rodSettings.length, 0));
            #endif
            #ifdef COM_COLLECTION
                // NOTE: this position is only for simulation of a single module
                finalStruct.move(btVector3(0, -rodSettings.length, 0));
                finalStruct.addRotation(rotationPoint, yAxis, M_PI/4);
                finalStruct.addRotation(finalStruct.getCentroid(), xAxis, -M_PI/2);
                finalStruct.move(btVector3(0, 2*rodSettings.length, 0));
            #endif
            break;
        }
        case 1: {
            std::cout << "Pos 1"<< std::endl;
            // let the robot fall from higher altitude
            finalStruct.move(btVector3(0, 35-rodSettings.length, 0)); // 35 cm above the ground plane
            break;
        }
        case 2: {
            std::cout << "Pos 2"<< std::endl;
            // reset to original position
            finalStruct.move(btVector3(0, -rodSettings.length, 0));
            finalStruct.addRotation(rotationPoint, yAxis, M_PI/4);

            // rotate of 180 degrees along X axis (upside-down robot)
            finalStruct.addRotation(finalStruct.getCentroid(), xAxis, M_PI);
            finalStruct.addRotation(finalStruct.getCentroid(), yAxis, M_PI/4);
            finalStruct.move(btVector3(0, rodSettings.length, 0));
            break;
        }
        default:
            break;
    }

    // RODS
    const tgRod::Config rodConfig(rodSettings.radius,
                                  rodSettings.density,
                                  globalSettings.friction,
                                  globalSettings.rollFriction,
                                  globalSettings.restitution);

    #ifdef COM_COLLECTION
        const tgRod::Config pivotConfig(rodSettings.radius, 0,
                                        globalSettings.friction,
                                        globalSettings.rollFriction,
                                        globalSettings.restitution);
    #endif

    // ROD CONNECTION TO THE SERVO MOTOR
    const tgRod::Config lightrodConfig(lightRodSettings.radius,
                                       lightRodSettings.density,
                                       globalSettings.friction,
                                       globalSettings.rollFriction,
                                       globalSettings.restitution);

   const tgRod::Config motorRodConfig(rodSettings.radius,
                                      actuatedSettings.actuatorDensity,
                                      globalSettings.friction,
                                      globalSettings.rollFriction,
                                      globalSettings.restitution);

    // SERVO MOTOR
    const tgSphere::Config sphereConfig(actuatedSettings.sphereRadius, actuatedSettings.sphereDensity);

    // STEADY CABLE (cable connected to the actuator rod - ID 0/0a)
    const tgBasicActuator::Config steadyConfig(muscleSettings.maxStiffness,
                                               muscleSettings.damping,
                                               muscleSettings.activePretension,
                                               globalSettings.hist,
                                               muscleSettings.maxActiveTension,
                                               muscleSettings.targetVelocity);

    #ifndef NO_SERVO
        // SERVO CONFIG
        const tgRod::Config servoConfig(servoSettings.radius,
                                        servoSettings.density,
                                        globalSettings.friction,
                                        globalSettings.rollFriction,
                                        globalSettings.restitution);

        // SERVO-ROD CONNECTION CONFIG
        const tgRod::Config servoRodConnectionConfig(lightRodSettings.radius,
                                                     servoSettings.density,
                                                     globalSettings.friction,
                                                     globalSettings.rollFriction,
                                                     globalSettings.restitution);
    #endif

    #ifdef COM_COLLECTION
        // PIVOT CABLE
        const tgBasicActuator::Config pivotCableConfig(connectorSettings.compressiveStiffness,
                0, connectorSettings.compressiveStiffness*0.16, globalSettings.hist,
                connectorSettings.maxTension, 0);
    #endif


    // JOINT BETWEEN MODULES
    const tgBasicActuator::Config connectorConfig(connectorSettings.compressiveStiffness,
                                                  connectorSettings.damping,
                                                  connectorSettings.preStretch,
                                                  globalSettings.hist,
                                                  connectorSettings.maxTension,
                                                  connectorSettings.targetVelocity);

    //////////////////////////
    tgBuildSpec spec;
    for (int i = 0; i < m_moduleData.size(); i++)
    {
        // PASSIVE CABLE
        const tgBasicActuator::Config pmuscleConfig(
                m_moduleData[i].stiffness*muscleSettings.maxStiffness,
                muscleSettings.damping,
                m_moduleData[i].stiffness*muscleSettings.pretension,
                globalSettings.hist,
                muscleSettings.maxPassiveTension, muscleSettings.targetVelocity
        );
        spec.addBuilder("passive muscle"+std::to_string(i), new tgBasicActuatorInfo(pmuscleConfig));
    }
    ///////////////////////////

    // Create the build spec that uses tags to turn the structure into a real model
    spec.addBuilder("rod", new tgRodInfo(rodConfig));
    spec.addBuilder("half rod", new tgRodInfo(motorRodConfig));
    spec.addBuilder("light rod", new tgRodInfo(lightrodConfig));
    spec.addBuilder("sphere", new tgSphereInfo(sphereConfig));

    #ifndef NO_SERVO
        spec.addBuilder("servo", new tgRodInfo(servoConfig));
        spec.addBuilder("servo_rod_connection", new tgRodInfo(servoRodConnectionConfig));
    #endif

    #ifdef COM_COLLECTION
        spec.addBuilder("pivot", new tgRodInfo(pivotConfig));
        spec.addBuilder("pivot_cable", new tgBasicActuatorInfo(pivotCableConfig));
    #endif

    const int actuateCables[5] = { 1, 4, 6, 7, 10 };

    for (int i = 0; i < 5; i++) {
        // ACTIVE CABLE
        const tgBasicActuator::Config muscleConfig(
            muscleSettings.maxStiffness,
            muscleSettings.damping,
            muscleSettings.maxStiffness * 5 * activePreStretches[i],
            globalSettings.hist,
            muscleSettings.maxActiveTension,
            muscleSettings.targetVelocity
        );
        spec.addBuilder("active muscle am"+std::to_string(actuateCables[i]),
                        new tgBasicActuatorInfo(muscleConfig));
    }

    spec.addBuilder("steady muscle", new tgBasicActuatorInfo(steadyConfig));
    spec.addBuilder("connector", new tgBasicActuatorInfo(connectorConfig));

    // Create your structureInfo
    tgStructureInfo structureInfo(finalStruct, spec);

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    // We could now use tgCast::filter or similar to pull out the models (e.g. actuators)
    // that we want to control.
    allActuators = tgCast::filter<tgModel, tgBasicActuator> (getDescendants());
    mapActuators(actuatorMap, *this);

    // Extract the rods related to the front face and the one parallel to it
    getFrontFaceAndParallel(frontFace, frontFaceParallel, mov_dir, *this, m_moduleData);

#ifdef ADD_MARKERS
    // add marker points on the nodes (for construction purporse)
    addMarkers(finalStruct, *this, m_moduleData);
#endif

    // Notify controllers that setup has finished.
    notifySetup();

    // Actually setup the children
    tgModel::setup(world);
//    trace(structureInfo, *this);
}

void robotModel::step(double dt)
{
    if (dt < 0.0)
    {
        throw std::invalid_argument("dt is not positive");
    }
    else
    {
        notifyStep(dt);
        // Step any children
        tgModel::step(dt);
    }
}

void robotModel::onVisit(tgModelVisitor& r)
{
    tgModel::onVisit(r);
}

const std::vector<tgBasicActuator*>&
robotModel::getActuators (const std::string& key) const
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

double robotModel::getModuleSize() {
    return module_size;
}

double robotModel::getLength(int num_modules) {
    return m_moduleData.size() != 0 ? module_size * m_moduleData.size() : module_size *num_modules;
}

int robotModel::getMovDir() {
    return mov_dir;
}

void robotModel::setMovDir(int moving_direction) {
    mov_dir = moving_direction;
}

void robotModel::setRotationAngle(double rot_angle) {
    rotation_angle = rot_angle;
}

void robotModel::getCurrentPosition(btVector3 &position) {
    position.setZero();

    for (int i=0; i < frontFace.size(); i++) {
        position += frontFace[i]->centerOfMass();
        if (i == 1 && frontFace.size() == 4) {
            position /= 2;
        }
    }
    position /= 3;
    position.setY(0);
}

void robotModel::getFirstModulePosition(btVector3 &f_position, btVector3 &p_position) {
    f_position.setZero();
    for (int i=0; i < frontFace.size(); i++) {
        f_position += frontFace[i]->centerOfMass();
        if (i == 1 && frontFace.size() == 4) {
            f_position /= 2;
        }
    }
    f_position /= 3;
    f_position.setY(0);

    p_position.setZero();
    for (int i=0; i < frontFaceParallel.size(); i++) {
        p_position += frontFaceParallel[i]->centerOfMass();
        if (i == 1 && frontFaceParallel.size() == 4) {
            p_position /= 2;
        }
    }
    p_position /= 3;
    p_position.setY(0);
}


void robotModel::getCoordinatesFromDistAndBearing(double distance, double bearing, btVector3 &target) {
    // Compute the position of the CoM of the front face
    btVector3 frontFaceCoM(0.0, 0.0, 0.0);
    for (int i=0; i < frontFace.size(); i++) {
        frontFaceCoM += frontFace[i]->centerOfMass();
        if (i == 1 && frontFace.size() == 4) {
            frontFaceCoM /= 2;
        }
    }
    frontFaceCoM /= 3;
    frontFaceCoM.setY(0);

    // Compute the CoM of the face parallel to the front one and set y coord. to 0
    btVector3 parallelFaceCoM(0.0, 0.0, 0.0);
    for (int i=0; i < frontFaceParallel.size(); i++) {
        parallelFaceCoM += frontFaceParallel[i]->centerOfMass();
        if (i == 1 && frontFaceParallel.size() == 4) {
            parallelFaceCoM /= 2;
        }
    }
    parallelFaceCoM /= 3;
    parallelFaceCoM.setY(0);

    // Get the vector representing the axis of the first module
    btVector3 moduleVec = frontFaceCoM - parallelFaceCoM;

    // Get the normalized vector representing the direction of the target
    btVector3 tVecDir(0.0, 0.0, 0.0);

    tVecDir.setX(moduleVec.getX()*cos(bearing)-moduleVec.getZ()*sin(bearing));
    tVecDir.setZ(moduleVec.getX()*sin(bearing)+moduleVec.getZ()*cos(bearing));

    tVecDir.normalize();

    // Obtain the translation vector to apply to the frontFaceCoM
    btVector3 translationVec = tVecDir * distance;

    // Compute target position
    target.setZero();
    target.setX(frontFaceCoM.getX()+translationVec.getX());
    target.setZ(frontFaceCoM.getZ()+translationVec.getZ());
}

void robotModel::getDistAndBearingToGoal(btVector3 target, double& distance, double& bearing){
    // Compute the position of the CoM of the front face
    btVector3 frontFaceCoM(0.0, 0.0, 0.0);

    for (int i=0; i < frontFace.size(); i++) {
        frontFaceCoM += frontFace[i]->centerOfMass();
        if (i == 1 && frontFace.size() == 4) {
            frontFaceCoM /= 2;
        }
    }
    frontFaceCoM /= 3;

    // Set Y coordinate to 0
    frontFaceCoM.setY(0);
    target.setY(0);

    // Compute the distance btw target and front face CoM
    distance = frontFaceCoM.distance(target);

    // Compute the CoM of the face parallel to the front one and set y coord. to 0
    btVector3 parallelFaceCoM(0.0, 0.0, 0.0);
    for (int i=0; i < frontFaceParallel.size(); i++) {
        parallelFaceCoM += frontFaceParallel[i]->centerOfMass();
        if (i == 1 && frontFaceParallel.size() == 4) {
            parallelFaceCoM /= 2;
        }
    }
    parallelFaceCoM /= 3;
    parallelFaceCoM.setY(0);

    // Get the vectors needed to compute the bearing to goal
    btVector3 moduleVec = frontFaceCoM - parallelFaceCoM;
    btVector3 toTargetVec = target - frontFaceCoM;

    if(std::abs(toTargetVec.length() - 0.0) < std::numeric_limits<double>::epsilon()) {
        toTargetVec = moduleVec;
    }

    // Compute and normalize bearing to interval [-pi,+pi]
    double bearingToGoal = atan2(toTargetVec.getZ(), toTargetVec.getX()) - atan2(moduleVec.getZ(), moduleVec.getX());
    if (bearingToGoal < -M_PI) {
        bearingToGoal += 2*M_PI;
    } else if (bearingToGoal > M_PI) {
        bearingToGoal -= 2*M_PI;
    }
    bearing = bearingToGoal;

    /*
    std::cout << "Robot front face " << frontFaceCoM << std::endl;
    for (int i=0; i < frontFace.size(); i++) {
        std::cout << "    CoM " << i << ": " << frontFace[i]->centerOfMass() << std::endl;
    }
    std::cout << "Robot parallel face " << parallelFaceCoM << std::endl;
    for (int i=0; i < frontFaceParallel.size(); i++) {
        std::cout << "    CoM " << i << ": " << frontFaceParallel[i]->centerOfMass() << std::endl;
    }
    std::cout << "Target " << target << std::endl;
    // std::cout << "    y coordinate: " << y << std::endl;
    std::cout << "Distance " << distance << std::endl;
    std::cout << "Bearing " << bearing << " (" << bearing/M_PI << " PI)" << std::endl;
    */
}

void robotModel::teardown() {
    notifyTeardown();
    tgModel::teardown();

    frontFace.clear();
    frontFaceParallel.clear();
}