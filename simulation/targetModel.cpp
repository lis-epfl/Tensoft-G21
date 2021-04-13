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
 * @author Enrico Zardini
 * @copyright Copyright (C) 2019 LIS
 * $Id$
 */

// The C++ Standard Library
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <cstdio>
#include <unistd.h>

#include <Eigen/Dense>

// Application library
#include "targetModel.h"

namespace
{
    // NOTE that any parameter that depends on units of length will scale
    // with the current gravity scaling. E.g., with gravity as 98.1,
    // the length units below are in decimeters.

    const struct GlobalConfig {
        double friction;                        // unit-less
        double rollFriction;                    // unit-less
        double restitution;
        double ground_offset;
    } globalSettings {
            1.0,
            0.005,
            0.8,
            1.5
    };

    const struct RodConfig {
        double rod_ground_offset;
        double density;
        double radius;
        double length;
    } rodSettings {
        0.01,
        0,
        1,
        2
    };
} // namespace

targetModel::targetModel() {}

targetModel::targetModel(btVector3 position): target_position(position) {}

/**
 * Anonymous namespace for helper functions
 */
namespace {
    void addNodes(tgStructure &finalStruct) {
        finalStruct.addNode(0, globalSettings.ground_offset+rodSettings.rod_ground_offset, 0, "t_node n0");
        finalStruct.addNode(0, globalSettings.ground_offset+rodSettings.rod_ground_offset+rodSettings.length, 0, "t_node n1");
    }

    void addPairs(tgStructure &finalStruct) {
        finalStruct.addPair(0, 1, "t_rod r0");
    }
}

void targetModel::setup(tgWorld& world)
{
    tgStructure finalStruct;

    addNodes(finalStruct);
    addPairs(finalStruct);

    finalStruct.move(target_position);

    // ROD
    const tgRod::Config rodConfig(rodSettings.radius,
                                  rodSettings.density,
                                  globalSettings.friction,
                                  globalSettings.rollFriction,
                                  globalSettings.restitution);

    tgBuildSpec spec;

    // Create the build spec that uses tags to turn the structure into a real model
    spec.addBuilder("t_rod", new tgRodInfo(rodConfig));

    // Create your structureInfo
    tgStructureInfo structureInfo(finalStruct, spec);

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    rods = this->find<tgRod>("t_rod");

    // Notify controllers that setup has finished.
    notifySetup();

    // Actually setup the children
    tgModel::setup(world);
}

void targetModel::step(double dt)
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

void targetModel::onVisit(tgModelVisitor& r)
{
    tgModel::onVisit(r);
}

void targetModel::getPosition(btVector3& position) {
    position.setZero();
    position += target_position;
}

void targetModel::setPosition(btVector3 &position) {
    target_position.setZero();
    target_position += position;
}

void targetModel::teardown() {
    notifyTeardown();
    tgModel::teardown();
}
