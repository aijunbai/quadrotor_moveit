//
// Created by baj on 8/4/15.
//

#ifndef PROJECT_QUADROTOR_H
#define PROJECT_QUADROTOR_H

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <std_srvs/Empty.h>

#include <quadrotor_moveit/utils.h>


class Quadrotor {
public:
    Quadrotor(bool display=false, bool debug=false);

    ~Quadrotor() {
        _group.clearPoseTargets();
    }

    bool engage() {
        _ready = true;
        return _call_service("/engage");
    }

    bool shutdown() {
        _ready = false;
        return _call_service("/shutdown");
    }

    std::vector<double> get_current_joint_values() {
        std::vector<double> group_variable_values;
        _group.getCurrentState()->copyJointGroupPositions(
                _group.getCurrentState()->getRobotModel()->getJointModelGroup(
                        _group.getName()), group_variable_values);

        return group_variable_values;
    }

    bool set_joint_value_target(std::vector<double> &joints) {
        return _group.setJointValueTarget(joints);
    }

    void display(const moveit::planning_interface::MoveGroup::Plan &plan) {
        moveit_msgs::DisplayTrajectory display_trajectory;

        display_trajectory.trajectory_start = plan.start_state_;
        display_trajectory.trajectory.push_back(plan.trajectory_);
        _display_publisher.publish(display_trajectory);
    }

    void moveto(
            std::vector<double> &joints, bool wait=true,
            double error=0.1, int iterations=10, bool verbose=false);

private:
    double _dist(std::vector<double> &a, std::vector<double> &b) {
        double sum = 0.0;

        for (int i = 0; i < a.size(); ++i) {
            sum += (a[i] - b[i]) * (a[i] - b[i]);
        }

        return sqrt(sum);
    }

    bool _call_service(const std::string &service_name) {
        if (_debug) {
            ROS_INFO("Calling service: %s", service_name.c_str());
        }

        std_srvs::Empty empty;
        return ros::service::call(service_name, empty);
    }

    bool _plan() {
        moveit::planning_interface::MoveGroup::Plan plan;
        return _group.plan(plan);
    }

private:
    bool _display;
    bool _debug;
    bool _ready;
    moveit::planning_interface::MoveGroup _group;
    ros::Publisher _display_publisher;
};


#endif //PROJECT_QUADROTOR_H
