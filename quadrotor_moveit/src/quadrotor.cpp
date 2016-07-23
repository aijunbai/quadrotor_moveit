//
// Created by baj on 8/4/15.
//

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

#include <quadrotor_moveit/quadrotor.h>
#include <angles/angles.h>
#include <geometry_msgs/Pose.h>

Quadrotor::Quadrotor(bool display, bool debug):
    _display(display),
    _debug(debug),
    _ready(false),
    _group("quadrotor")
{
    ros::NodeHandle node_handle;

    _display_publisher =
            node_handle.advertise<moveit_msgs::DisplayTrajectory>(
                "/move_group/display_planned_path", 1, true);
}

void Quadrotor::moveto(
        std::vector<double> &joints, bool wait, double error, int iterations, bool verbose)
{
    if (!_ready) {
        engage();
    }

    if (verbose) {
        ROS_INFO("Move to:");
        for (int i = 0; i < joints.size(); ++i) {
            std::cout << joints[i] << " ";
        }
        std::cout << std::endl;
    }

    ros::Rate rate(10);
    double err = error;

    for (int i = 0; i < iterations; ++i) {
        geometry_msgs::Pose start_pose = _group.getCurrentPose().pose;
        moveit_msgs::OrientationConstraint ocm;

        ocm.link_name = "base_link";
        ocm.header.frame_id = "/world";
        ocm.orientation = start_pose.orientation;
        ocm.absolute_x_axis_tolerance = 0.1;
        ocm.absolute_y_axis_tolerance = 0.1;
        ocm.absolute_z_axis_tolerance = M_PI;
        ocm.weight = 1.0;

        moveit_msgs::Constraints path_constraints;
        path_constraints.name = "fly";
        path_constraints.orientation_constraints.push_back(ocm);
        _group.setPathConstraints(path_constraints);

        set_joint_value_target(joints);

        if (_plan()) {
            if (wait) {
                _group.move();
            }
            else {
                _group.asyncMove();
            }

            std::vector<double> j = get_current_joint_values();
            err = _dist(j, joints);

            if (verbose) {
                ROS_INFO("Error: %f", err);
            }

            if (err < error) {
                break;
            }
        }

        rate.sleep();
    }

    if (err >= error) {
        ROS_INFO("Moveto failed");
    }

    _group.clearPathConstraints();

}
