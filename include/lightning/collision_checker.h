/*
# Software License Agreement (BSD License)
#
# Copyright (c) 2012, University of California, Berkeley
# All rights reserved.
# Authors: Cameron Lee (cameronlee@berkeley.edu) and Dmitry Berenson (
        berenson@eecs.berkeley.edu)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of University of California, Berkeley nor the names
of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef COLLISION_CHECKER_H_
#define COLLISION_CHECKER_H_

#include <ros/ros.h>
#include <math.h>
#include <vector>

#include "planning_environment/models/collision_models_interface.h"
#include "arm_navigation_msgs/PlanningScene.h"
#include "lightning/collision_utils.h"

class CollisionChecker {
    public:
        CollisionChecker(double step_size);
        ~CollisionChecker();

        bool collisionModelsInterfaceLoadedModels();
        const arm_navigation_msgs::PlanningScene& getPlanningScene();
        const std::vector<std::string>& getJointNames();
        bool acquireScene(std::string group_name);
        void releaseScene();
        bool checkStateValid(const std::vector<double> &point);
        bool checkMiddle(const std::vector<double> &first, const std::vector<double> &second);
        bool checkMiddleAndReturnPoints(const std::vector<double> &first, const std::vector<double> &second, std::vector< std::vector<double> > &new_points);

    private:
        planning_environment::CollisionModelsInterface *collision_models_interface_;
        std::vector<std::string> arm_names_;
        std::vector<std::string> joint_names_;
        int num_joints_;
        double step_size_;

        bool isStateValid(const planning_models::KinematicState &state);
};

#endif //for COLLISION_CHECKER_H_
