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
