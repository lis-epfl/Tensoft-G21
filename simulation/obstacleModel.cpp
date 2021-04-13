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
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <cstdio>
#include <unistd.h>
#include "math.h"
#include<bits/stdc++.h>

#include <Eigen/Dense>

// Application library
#include "obstacleModel.h"

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
            1,
            0.005,
            0.8,
            1.5
    };

    const struct BoxConfig {
        double box_ground_offset;
        double opening_size;
        double distance_scaling_factor;
        double half_width;
        double half_height;
        double density;
    } boxSettings {
        0.01,
        8.00108, // 72% of 11.11260546 (module's size)
        4,
        8.33445, // 75% of 11.11260546 (module's size)
        6,
        0
    };
} // namespace

obstacleModel::obstacleModel() {}

obstacleModel::obstacleModel(btVector3 frontFaceCoM, double rb_len, btVector3 target_pos, double rb_target_bearing) {
    initialize(frontFaceCoM, rb_len, target_pos, rb_target_bearing);
}

void obstacleModel::initialize(btVector3 frontFaceCoM, double rb_len, btVector3 target_pos, double rb_target_bearing) {
    // ignore Y coordinate
    frontFaceCoM.setY(0);
    target_pos.setY(0);

    // set rbInitFrontFace class variable
    rbInitFrontFace = frontFaceCoM;

    // set robot_len variable
    robot_len = rb_len;

    // direction towards target
    target_dir = target_pos - frontFaceCoM;

    // set target bearing variable
    t_bearing = rb_target_bearing;

    // Compute and normalize angle wrt to x-axis to interval [-pi,+pi]
    double angle = atan2(target_dir.getZ(), target_dir.getX()) - atan2(0.0, 1.0);
    if (angle < -M_PI) {
        angle += 2*M_PI;
    } else if (angle > M_PI) {
        angle -= 2*M_PI;
    }
    t_angle = angle;
}

/*
    std::cout << "Target " << target_dir << std::endl;
    std::cout << "X dist " << x_dist << std::endl;
    std::cout << "Robot len " << robot_len << std::endl;
    std::cout << "T bearing " << t_bearing << std::endl;
    std::cout << "T angle " << t_angle << std::endl;
    std::cout << "F " << f_scale << std::endl;
    std::cout << "B " << b_scale << std::endl;
*/

/**
 * Anonymous namespace for helper functions
 */
namespace {

    double clamp(double val, double low, double high){
        return std::min(std::max(val, low), high);
    }

    void buildWalls(tgStructure &walls, btVector3 target_dir, double t_angle, double t_bearing, double robot_len, std::vector<btVector3>& opening_entrance){
        btVector3 x_axis(1.0, 0.0, 0.0);
        btVector3 z_axis(0.0, 0.0, 1.0);

        btVector3 opening_pos(0.0,0.0,0.0);
        btVector3 tlv_position(0.0,0.0,0.0);
        btVector3 tlh_position(0.0,0.0,0.0);
        btVector3 trv_position(0.0,0.0,0.0);
        btVector3 trh_position(0.0,0.0,0.0);
        btVector3 blv_position(0.0,0.0,0.0);
        btVector3 blh_position(0.0,0.0,0.0);
        btVector3 brv_position(0.0,0.0,0.0);
        btVector3 brh_position(0.0,0.0,0.0);

        double f_scale, b_scale;
        opening_entrance.clear();

        if (t_angle > -M_PI/4 && t_angle <= M_PI/4 ){
            double x_dist = target_dir.getX();
            f_scale = std::abs(t_bearing) <= M_PI/2 ? x_dist/boxSettings.distance_scaling_factor : robot_len + (x_dist - robot_len)/boxSettings.distance_scaling_factor;
            f_scale += boxSettings.half_width;
            b_scale = std::max({f_scale, 2*robot_len, std::abs(target_dir.getZ())}) + boxSettings.opening_size/2;

            blv_position = f_scale * x_axis + (-b_scale) * z_axis;
            tlv_position = f_scale * x_axis +  b_scale  * z_axis;
            tlh_position = (f_scale+boxSettings.half_width) * x_axis +  (b_scale+boxSettings.half_width) * z_axis;
            trh_position = (-(b_scale+boxSettings.half_width)) * x_axis +  (b_scale+boxSettings.half_width) * z_axis;
            trv_position = (-b_scale) * x_axis +  b_scale * z_axis;
            brv_position = (-b_scale) * x_axis + (-b_scale) * z_axis;
            brh_position = (-(b_scale+boxSettings.half_width)) * x_axis + (-(b_scale+boxSettings.half_width)) * z_axis;
            blh_position = (f_scale+boxSettings.half_width) * x_axis + (-(b_scale+boxSettings.half_width)) * z_axis;


            opening_pos.setX(f_scale);
            opening_pos.setZ(target_dir.getZ());

            // Target on line
            // opening_pos.setZ((f_scale*target_dir.getZ())/target_dir.getX());

            walls.addNode(blv_position.getX(), globalSettings.ground_offset+boxSettings.box_ground_offset+boxSettings.half_height, blv_position.getZ(), "w_node n0");
            walls.addNode(opening_pos.getX(), globalSettings.ground_offset+boxSettings.box_ground_offset+boxSettings.half_height, opening_pos.getZ()-boxSettings.opening_size/2, "w_node n1");
            walls.addNode(opening_pos.getX(), globalSettings.ground_offset+boxSettings.box_ground_offset+boxSettings.half_height, opening_pos.getZ()+boxSettings.opening_size/2, "w_node n2");
            walls.addNode(tlv_position.getX(), globalSettings.ground_offset+boxSettings.box_ground_offset+boxSettings.half_height, tlv_position.getZ(), "w_node n3");
            walls.addNode(tlh_position.getX(), globalSettings.ground_offset+boxSettings.box_ground_offset+boxSettings.half_height, tlh_position.getZ(), "w_node n4");
            walls.addNode(trh_position.getX(), globalSettings.ground_offset+boxSettings.box_ground_offset+boxSettings.half_height, trh_position.getZ(), "w_node n5");
            walls.addNode(trv_position.getX(), globalSettings.ground_offset+boxSettings.box_ground_offset+boxSettings.half_height, trv_position.getZ(), "w_node n6");
            walls.addNode(brv_position.getX(), globalSettings.ground_offset+boxSettings.box_ground_offset+boxSettings.half_height, brv_position.getZ(), "w_node n7");
            walls.addNode(brh_position.getX(), globalSettings.ground_offset+boxSettings.box_ground_offset+boxSettings.half_height, brh_position.getZ(), "w_node n8");
            walls.addNode(blh_position.getX(), globalSettings.ground_offset+boxSettings.box_ground_offset+boxSettings.half_height, blh_position.getZ(), "w_node n9");

            opening_entrance.push_back(btVector3(opening_pos.getX()-boxSettings.half_width, 0, opening_pos.getZ()-boxSettings.opening_size/2));
            opening_entrance.push_back(btVector3(opening_pos.getX()-boxSettings.half_width, 0, opening_pos.getZ()+boxSettings.opening_size/2));

            walls.addPair(0,1,"box2");
            walls.addPair(2,3,"box2");
            walls.addPair(4,5,"box1");
            walls.addPair(6,7,"box2");
            walls.addPair(8,9,"box1");

        } else if (t_angle > M_PI/4 && t_angle <= 3*M_PI/4){
            double z_dist = target_dir.getZ();
            f_scale = std::abs(t_bearing) <= M_PI/2 ? z_dist/boxSettings.distance_scaling_factor : robot_len + (z_dist - robot_len)/boxSettings.distance_scaling_factor;
            f_scale += boxSettings.half_width;
            b_scale = std::max({f_scale, 2*robot_len, std::abs(target_dir.getX())}) + boxSettings.opening_size/2;

            blv_position = (b_scale+boxSettings.half_width) * x_axis + (-(b_scale+boxSettings.half_width)) * z_axis;
            tlv_position = (b_scale+boxSettings.half_width) * x_axis + (f_scale+boxSettings.half_width) * z_axis;
            tlh_position = b_scale  * x_axis +  f_scale * z_axis;
            trh_position = (-b_scale) * x_axis +  f_scale * z_axis;
            trv_position = (-(b_scale+boxSettings.half_width)) * x_axis + (f_scale+boxSettings.half_width) * z_axis;
            brv_position = (-(b_scale+boxSettings.half_width)) * x_axis + (-(b_scale+boxSettings.half_width)) * z_axis;
            brh_position = (-b_scale) * x_axis + (-b_scale) * z_axis;
            blh_position = b_scale * x_axis + (-b_scale) * z_axis;

            opening_pos.setZ(f_scale);
            opening_pos.setX(target_dir.getX());

            // Target on line
            // opening_pos.setX((f_scale*target_dir.getX())/target_dir.getZ());

            walls.addNode(blv_position.getX(), globalSettings.ground_offset+boxSettings.box_ground_offset+boxSettings.half_height, blv_position.getZ(), "w_node n0");
            walls.addNode(tlv_position.getX(), globalSettings.ground_offset+boxSettings.box_ground_offset+boxSettings.half_height, tlv_position.getZ(), "w_node n1");
            walls.addNode(tlh_position.getX(), globalSettings.ground_offset+boxSettings.box_ground_offset+boxSettings.half_height, tlh_position.getZ(), "w_node n2");
            walls.addNode(opening_pos.getX()+boxSettings.opening_size/2, globalSettings.ground_offset+boxSettings.box_ground_offset+boxSettings.half_height, opening_pos.getZ(), "w_node n3");
            walls.addNode(opening_pos.getX()-boxSettings.opening_size/2, globalSettings.ground_offset+boxSettings.box_ground_offset+boxSettings.half_height, opening_pos.getZ(), "w_node n4");
            walls.addNode(trh_position.getX(), globalSettings.ground_offset+boxSettings.box_ground_offset+boxSettings.half_height, trh_position.getZ(), "w_node n5");
            walls.addNode(trv_position.getX(), globalSettings.ground_offset+boxSettings.box_ground_offset+boxSettings.half_height, trv_position.getZ(), "w_node n6");
            walls.addNode(brv_position.getX(), globalSettings.ground_offset+boxSettings.box_ground_offset+boxSettings.half_height, brv_position.getZ(), "w_node n7");
            walls.addNode(brh_position.getX(), globalSettings.ground_offset+boxSettings.box_ground_offset+boxSettings.half_height, brh_position.getZ(), "w_node n8");
            walls.addNode(blh_position.getX(), globalSettings.ground_offset+boxSettings.box_ground_offset+boxSettings.half_height, blh_position.getZ(), "w_node n9");

            opening_entrance.push_back(btVector3(opening_pos.getX()+boxSettings.opening_size/2, 0, opening_pos.getZ()-boxSettings.half_width));
            opening_entrance.push_back(btVector3(opening_pos.getX()-boxSettings.opening_size/2, 0, opening_pos.getZ()-boxSettings.half_width));

            walls.addPair(0,1,"box2");
            walls.addPair(2,3,"box1");
            walls.addPair(4,5,"box1");
            walls.addPair(6,7,"box2");
            walls.addPair(8,9,"box1");

        } else if (t_angle > -3*M_PI/4 && t_angle <= -M_PI/4){
            double z_dist = -target_dir.getZ();
            f_scale = std::abs(t_bearing) <= M_PI/2 ? z_dist/boxSettings.distance_scaling_factor : robot_len + (z_dist - robot_len)/boxSettings.distance_scaling_factor;
            f_scale += boxSettings.half_width;
            b_scale = std::max({f_scale, 2*robot_len, std::abs(target_dir.getX())}) + boxSettings.opening_size/2;

            blv_position = (b_scale+boxSettings.half_width) * x_axis + (-(f_scale+boxSettings.half_width)) * z_axis;
            tlv_position = (b_scale+boxSettings.half_width) * x_axis + (b_scale+boxSettings.half_width) * z_axis;
            tlh_position = b_scale * x_axis + b_scale * z_axis;
            trh_position = (-b_scale) * x_axis +  b_scale * z_axis;
            trv_position = (-(b_scale+boxSettings.half_width)) * x_axis + b_scale * z_axis;
            brv_position = (-(b_scale+boxSettings.half_width)) * x_axis + (-(f_scale+boxSettings.half_width)) * z_axis;
            brh_position = (-b_scale) * x_axis + (-f_scale) * z_axis;
            blh_position =   b_scale  * x_axis + (-f_scale) * z_axis;

            opening_pos.setZ(-f_scale);
            opening_pos.setX(target_dir.getX());

            // Target on line
            // opening_pos.setX((-f_scale*target_dir.getX())/target_dir.getZ());

            walls.addNode(blv_position.getX(), globalSettings.ground_offset+boxSettings.box_ground_offset+boxSettings.half_height, blv_position.getZ(), "w_node n0");
            walls.addNode(tlv_position.getX(), globalSettings.ground_offset+boxSettings.box_ground_offset+boxSettings.half_height, tlv_position.getZ(), "w_node n1");
            walls.addNode(tlh_position.getX(), globalSettings.ground_offset+boxSettings.box_ground_offset+boxSettings.half_height, tlh_position.getZ(), "w_node n2");
            walls.addNode(trh_position.getX(), globalSettings.ground_offset+boxSettings.box_ground_offset+boxSettings.half_height, trh_position.getZ(), "w_node n3");
            walls.addNode(trv_position.getX(), globalSettings.ground_offset+boxSettings.box_ground_offset+boxSettings.half_height, trv_position.getZ(), "w_node n4");
            walls.addNode(brv_position.getX(), globalSettings.ground_offset+boxSettings.box_ground_offset+boxSettings.half_height, brv_position.getZ(), "w_node n5");
            walls.addNode(brh_position.getX(), globalSettings.ground_offset+boxSettings.box_ground_offset+boxSettings.half_height, brh_position.getZ(), "w_node n6");
            walls.addNode(opening_pos.getX()-boxSettings.opening_size/2, globalSettings.ground_offset+boxSettings.box_ground_offset+boxSettings.half_height, opening_pos.getZ(), "w_node n7");
            walls.addNode(opening_pos.getX()+boxSettings.opening_size/2, globalSettings.ground_offset+boxSettings.box_ground_offset+boxSettings.half_height, opening_pos.getZ(), "w_node n8");
            walls.addNode(blh_position.getX(), globalSettings.ground_offset+boxSettings.box_ground_offset+boxSettings.half_height, blh_position.getZ(), "w_node n9");

            opening_entrance.push_back(btVector3(opening_pos.getX()-boxSettings.opening_size/2, 0, opening_pos.getZ()+boxSettings.half_width));
            opening_entrance.push_back(btVector3(opening_pos.getX()+boxSettings.opening_size/2, 0, opening_pos.getZ()+boxSettings.half_width));

            walls.addPair(0,1,"box2");
            walls.addPair(2,3,"box1");
            walls.addPair(4,5,"box2");
            walls.addPair(6,7,"box1");
            walls.addPair(8,9,"box1");

        } else {
            double x_dist = -target_dir.getX();
            f_scale = std::abs(t_bearing) <= M_PI/2 ? x_dist/boxSettings.distance_scaling_factor : robot_len + (x_dist - robot_len)/boxSettings.distance_scaling_factor;
            f_scale += boxSettings.half_width;
            b_scale = std::max({f_scale, 2*robot_len, std::abs(target_dir.getZ())}) + boxSettings.opening_size/2;

            blv_position = b_scale * x_axis + (-b_scale) * z_axis;
            tlv_position = b_scale * x_axis +  b_scale * z_axis;
            tlh_position = (b_scale+boxSettings.half_width) * x_axis + (b_scale+boxSettings.half_width) * z_axis;
            trh_position = (-(f_scale+boxSettings.half_width)) * x_axis + (b_scale+boxSettings.half_width) * z_axis;
            trv_position = (-f_scale) * x_axis +  b_scale * z_axis;
            brv_position = (-f_scale) * x_axis + (-b_scale) * z_axis;
            brh_position = (-(f_scale+boxSettings.half_width)) * x_axis + (-(b_scale+boxSettings.half_width)) * z_axis;
            blh_position = (b_scale+boxSettings.half_width) * x_axis + (-(b_scale+boxSettings.half_width)) * z_axis;

            opening_pos.setX(-f_scale);
            opening_pos.setZ(target_dir.getZ());

            // Target on line
            //opening_pos.setZ((-f_scale*target_dir.getZ())/target_dir.getX());

            walls.addNode(blv_position.getX(), globalSettings.ground_offset+boxSettings.box_ground_offset+boxSettings.half_height, blv_position.getZ(), "w_node n0");
            walls.addNode(tlv_position.getX(), globalSettings.ground_offset+boxSettings.box_ground_offset+boxSettings.half_height, tlv_position.getZ(), "w_node n1");
            walls.addNode(tlh_position.getX(), globalSettings.ground_offset+boxSettings.box_ground_offset+boxSettings.half_height, tlh_position.getZ(), "w_node n2");
            walls.addNode(trh_position.getX(), globalSettings.ground_offset+boxSettings.box_ground_offset+boxSettings.half_height, trh_position.getZ(), "w_node n3");
            walls.addNode(trv_position.getX(), globalSettings.ground_offset+boxSettings.box_ground_offset+boxSettings.half_height, trv_position.getZ(), "w_node n4");
            walls.addNode(opening_pos.getX(), globalSettings.ground_offset+boxSettings.box_ground_offset+boxSettings.half_height, opening_pos.getZ()+boxSettings.opening_size/2, "w_node n5");
            walls.addNode(opening_pos.getX(), globalSettings.ground_offset+boxSettings.box_ground_offset+boxSettings.half_height, opening_pos.getZ()-boxSettings.opening_size/2, "w_node n6");
            walls.addNode(brv_position.getX(), globalSettings.ground_offset+boxSettings.box_ground_offset+boxSettings.half_height, brv_position.getZ(), "w_node n7");
            walls.addNode(brh_position.getX(), globalSettings.ground_offset+boxSettings.box_ground_offset+boxSettings.half_height, brh_position.getZ(), "w_node n8");
            walls.addNode(blh_position.getX(), globalSettings.ground_offset+boxSettings.box_ground_offset+boxSettings.half_height, blh_position.getZ(), "w_node n9");

            opening_entrance.push_back(btVector3(opening_pos.getX()+boxSettings.half_width, 0, opening_pos.getZ()+boxSettings.opening_size/2));
            opening_entrance.push_back(btVector3(opening_pos.getX()+boxSettings.half_width, 0, opening_pos.getZ()-boxSettings.opening_size/2));

            walls.addPair(0,1,"box2");
            walls.addPair(2,3,"box1");
            walls.addPair(4,5,"box2");
            walls.addPair(6,7,"box2");
            walls.addPair(8,9,"box1");
        }

        opening_entrance.insert(opening_entrance.begin(), ((opening_entrance[0]+opening_entrance[1])/2));
    }

    /*
    void addNodes(tgStructure &walls, double f_scale, double o_scale) {
        btVector3 position(0.0, 0.0, 0.0);
        btVector3 x_axis(1.0, 0.0, 0.0);
        btVector3 z_axis(0.0, 0.0, 1.0);

        // Top left node
        position = o_scale*x_axis + f_scale*z_axis;
        walls.addNode(position.getX(), globalSettings.ground_offset+boxSettings.box_ground_offset+boxSettings.half_height, position.getZ(), "w_node n0");

        // Top center-left node
        position = 2*x_axis + f_scale*z_axis;
        walls.addNode(position.getX(), globalSettings.ground_offset+boxSettings.box_ground_offset+boxSettings.half_height, position.getZ(), "w_node n1");

        // Top center-right node
        position = (-2)*x_axis + f_scale*z_axis;
        walls.addNode(position.getX(), globalSettings.ground_offset+boxSettings.box_ground_offset+boxSettings.half_height, position.getZ(), "w_node n2");

        // Top right node
        position = (-o_scale)*x_axis + f_scale*z_axis;
        walls.addNode(position.getX(), globalSettings.ground_offset+boxSettings.box_ground_offset+boxSettings.half_height, position.getZ(), "w_node n3");

        // Bottom right node
        position = (-o_scale)*x_axis + (-o_scale)*z_axis;
        walls.addNode(position.getX(), globalSettings.ground_offset+boxSettings.box_ground_offset+boxSettings.half_height, position.getZ(), "w_node n4");

        // Bottom left node
        position = o_scale*x_axis + (-o_scale)*z_axis;
        walls.addNode(position.getX(), globalSettings.ground_offset+boxSettings.box_ground_offset+boxSettings.half_height, position.getZ(), "w_node n5");
    }

    void addPairs(tgStructure &walls) {
        walls.addPair(0,1,"box1");
        walls.addPair(2,3,"box1");
        walls.addPair(3,4,"box2");
        walls.addPair(4,5,"box1");
        walls.addPair(5,0,"box2");
    }
    */
}

void obstacleModel::setup(tgWorld& world)
{
    tgStructure walls;

    buildWalls(walls, target_dir, t_angle, t_bearing, robot_len, opening_entrance);

    walls.move(rbInitFrontFace);

    // get walls pairs
    std::vector<tgPair> pairs = walls.getPairs().getPairs();
    for (auto pair: pairs){
        if (std::abs(pair.getFrom().getX() - pair.getTo().getX()) >= std::numeric_limits<double>::epsilon() ||
            std::abs(pair.getFrom().getZ() - pair.getTo().getZ()) >= std::numeric_limits<double>::epsilon()) {
            if (std::abs(pair.getFrom().getX() - pair.getTo().getX()) < std::numeric_limits<double>::epsilon()) {
                v_walls.push_back(pair);
            } else {
                h_walls.push_back(pair);
            }
        }
    }

    // translate opening entrance
    for(int i=0; i < opening_entrance.size(); i++){
        opening_entrance[i] = opening_entrance[i]+rbInitFrontFace;
    }

    // Wall parallel to X-axis
    const tgBox::Config boxConfig1(boxSettings.half_height,
                                  boxSettings.half_width,
                                  boxSettings.density);

    // Wall parallel to Z-axis
    const tgBox::Config boxConfig2(boxSettings.half_width,
                                   boxSettings.half_height,
                                   boxSettings.density);

    // Specification
    tgBuildSpec spec;

    // Create the build spec that uses tags to turn the structure into a real model
    spec.addBuilder("box1", new tgBoxInfo(boxConfig1));
    spec.addBuilder("box2", new tgBoxInfo(boxConfig2));

    // Create your structureInfo
    tgStructureInfo structureInfo(walls, spec);

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    // Notify controllers that setup has finished.
    notifySetup();

    // Actually setup the children
    tgModel::setup(world);
}

void obstacleModel::step(double dt)
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

bool obstacleModel::isOnPath(btVector3 frontFaceCoM, btVector3 frontFaceDir, double sensitivity) {
    // ignore Y coordinate
    frontFaceCoM.setY(0);
    frontFaceDir.setY(0);

    // compute direction line parameters
    double m = (std::abs(frontFaceDir.getX() - 0.0) >= std::numeric_limits<double>::epsilon()) ?
            frontFaceDir.getZ()/frontFaceDir.getX() : DBL_MAX;
    double q = frontFaceCoM.getZ() - m*frontFaceCoM.getX();

    bool hit = false;

    /*
    std::cout << "------------------------------------------------------------------" << std::endl;
    std::cout << "Front face CoM " << frontFaceCoM << std::endl;
    std::cout << "Front face dir " << frontFaceDir*sensitivity << std::endl;
    std::cout << "m " << m << std::endl;
    std::cout << "q " << q << std::endl;
    for (auto pair: h_walls) {
        std::cout << "Wall from " << pair.getFrom() << " to " << pair.getTo() << std::endl;
    }
    for (auto pair: v_walls) {
        std::cout << "Wall from " << pair.getFrom() << " to " << pair.getTo() << std::endl;
    }
    std::cout << std::endl;
    */

    for (auto pair: h_walls) {
        if ((std::abs(frontFaceDir.getX() - 0.0) >= std::numeric_limits<double>::epsilon()) && !hit) {
            double w_x_min = std::min(pair.getFrom().getX(), pair.getTo().getX());
            double w_x_max = std::max(pair.getFrom().getX(), pair.getTo().getX());

            double x1 = ((pair.getFrom().getZ() + boxSettings.half_width) - q) / m;
            double x2 = ((pair.getFrom().getZ() - boxSettings.half_width) - q) / m;

            btVector3 v1 = btVector3(x1, 0, pair.getFrom().getZ() + boxSettings.half_width) - frontFaceCoM;
            btVector3 v2 = btVector3(x2, 0, pair.getFrom().getZ() - boxSettings.half_width) - frontFaceCoM;

            if (signbit(v1.getX()) == signbit(frontFaceDir.getX()) && signbit(v1.getZ()) == signbit(frontFaceDir.getZ()) &&
                signbit(v2.getX()) == signbit(frontFaceDir.getX()) && signbit(v2.getZ()) == signbit(frontFaceDir.getZ())) {
                double d1 = -1.0;
                double d2 = -1.0;

                if (x1 >= w_x_min && x1 <= w_x_max && x2 >= w_x_min && x2 <= w_x_max) {
                    d1 = v1.length();
                    d2 = v2.length();
                } else if ((x1 >= w_x_min && x1 <= w_x_max) || (x2 >= w_x_min && x2 <= w_x_max)) {
                    double x_border = ( (m*w_x_min + q) >= (pair.getFrom().getZ() - boxSettings.half_width) &&
                                        (m*w_x_min + q) <= (pair.getFrom().getZ() + boxSettings.half_width) ) ? w_x_min : w_x_max;

                    btVector3 border_point(x_border, 0, m * x_border + q);
                    double d_border = (border_point - frontFaceCoM).length();

                    d1 = (x1 >= w_x_min && x1 <= w_x_max) ? v1.length() : v2.length();
                    d2 = d_border;
                }

                if (d1 > 0 && d2 > 0) {
                    /*
                    std::cout << "H wall" << std::endl;
                    std::cout << "Pair from " << pair.getFrom() << " to " << pair.getTo() << std::endl;
                    std::cout << "Intersec 1 " << btVector3(x1, 0, pair.getFrom().getZ() + boxSettings.half_width) << std::endl;
                    std::cout << "D v1 " << v1.length() << std::endl;
                    std::cout << "Intersec 2 " << btVector3(x2, 0, pair.getFrom().getZ() - boxSettings.half_width) << std::endl;
                    std::cout << "D v2 " << v2.length() << std::endl;
                    std::cout << "d1 " << d1 << std::endl;
                    std::cout << "d2 " << d2 << std::endl;
                    std::cout << "Result " << (std::min(d1, d2) <= sensitivity) << std::endl;
                    */
                    hit = (std::min(d1, d2) <= sensitivity);
                }
            }
        }
    }

    for (auto pair: v_walls) {
        if ((std::abs(frontFaceDir.getX() - DBL_MAX) >= std::numeric_limits<double>::epsilon()) && !hit) {
            double w_z_min = std::min(pair.getFrom().getZ(), pair.getTo().getZ());
            double w_z_max = std::max(pair.getFrom().getZ(), pair.getTo().getZ());

            double z1 = m*(pair.getFrom().getX() + boxSettings.half_width) + q;
            double z2 = m*(pair.getFrom().getX() - boxSettings.half_width) + q;

            btVector3 v1 = btVector3(pair.getFrom().getX() + boxSettings.half_width, 0, z1) - frontFaceCoM;
            btVector3 v2 = btVector3(pair.getFrom().getX() - boxSettings.half_width, 0, z2) - frontFaceCoM;

            if (signbit(v1.getX()) == signbit(frontFaceDir.getX()) && signbit(v1.getZ()) == signbit(frontFaceDir.getZ()) &&
                signbit(v2.getX()) == signbit(frontFaceDir.getX()) && signbit(v2.getZ()) == signbit(frontFaceDir.getZ())) {
                double d1 = -1.0;
                double d2 = -1.0;

                if (z1 >= w_z_min && z1 <= w_z_max && z2 >= w_z_min && z2 <= w_z_max) {
                    d1 = v1.length();
                    d2 = v2.length();
                } else if ((z1 >= w_z_min && z1 <= w_z_max) || (z2 >= w_z_min && z2 <= w_z_max)) {
                    double z_border = (((w_z_min - q) / m) >= (pair.getFrom().getX() - boxSettings.half_width) &&
                            ((w_z_min - q) / m) <= (pair.getFrom().getX() + boxSettings.half_width)) ? w_z_min : w_z_max;

                    btVector3 border_point((z_border - q) / m, 0, z_border);
                    double d_border = (border_point - frontFaceCoM).length();

                    d1 = (z1 >= w_z_min && z1 <= w_z_max) ? v1.length() : v2.length();
                    d2 = d_border;
                }

                if (d1 > 0 && d2 > 0) {
                    /*
                    std::cout << "V wall" << std::endl;
                    std::cout << "Pair from " << pair.getFrom() << " to " << pair.getTo() << std::endl;
                    std::cout << "Intersec 1 " << btVector3(pair.getFrom().getX() + boxSettings.half_width, 0, z1) << std::endl;
                    std::cout << "D v1 " << v1.length() << std::endl;
                    std::cout << "Intersec 2 " << btVector3(pair.getFrom().getX() - boxSettings.half_width, 0, z2) << std::endl;
                    std::cout << "D v2 " << v2.length() << std::endl;
                    std::cout << "d1 " << d1 << std::endl;
                    std::cout << "d2 " << d2 << std::endl;
                    std::cout << "Result " << (std::min(d1, d2) <= sensitivity) << std::endl;
                    */
                    hit = (std::min(d1, d2) <= sensitivity);
                }
            }
        }
    }

    // std::cout << "------------------------------------------------------------------" << std::endl;

    return hit;
}

void obstacleModel::get_opening_pos(btVector3& opening_pos) {
    opening_pos.setZero();
    opening_pos += opening_entrance[0];
}

int obstacleModel::get_distance_sign(btVector3 robot_pos) {
    int dist_sign = 0;

    std::vector<double> h_walls_centers;
    for(auto pair: h_walls) {
        h_walls_centers.push_back(pair.getFrom().getZ());
    }

    std::vector<double> v_walls_centers;
    for(auto pair: v_walls) {
        v_walls_centers.push_back(pair.getFrom().getX());
    }

    dist_sign = robot_pos.getZ() > (*std::min_element(h_walls_centers.begin(), h_walls_centers.end()) + boxSettings.half_width) &&
                robot_pos.getZ() < (*std::max_element(h_walls_centers.begin(), h_walls_centers.end()) - boxSettings.half_width) &&
                robot_pos.getX() > (*std::min_element(v_walls_centers.begin(), v_walls_centers.end()) + boxSettings.half_width) &&
                robot_pos.getX() < (*std::max_element(v_walls_centers.begin(), v_walls_centers.end()) - boxSettings.half_width) ? 1 : -1;

    return dist_sign;
}

void obstacleModel::onVisit(tgModelVisitor& r)
{
    tgModel::onVisit(r);
}

void obstacleModel::teardown() {
    notifyTeardown();
    tgModel::teardown();

    v_walls.clear();
    h_walls.clear();
}