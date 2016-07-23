/* Author: Aijun Bai */

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <quadrotor_moveit/quadrotor.h>

#include <boost/random/linear_congruential.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/generator_iterator.hpp>

typedef boost::minstd_rand base_generator_type;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "moveit_quadrotor");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    Quadrotor q(true, true);

    base_generator_type generator(42);
    boost::uniform_real<> uni_dist(-1.0, 1.0);
    boost::variate_generator<base_generator_type&, boost::uniform_real<> >
            uni(generator, uni_dist);

    ros::Rate rate(10);

    while (ros::ok()) {
        std::vector<double> joints = q.get_current_joint_values();
        for (int i = 0; i < 3; ++i) {
            joints[i] += uni();
        }
        joints[2] = std::max(joints[2], 0.5);
        q.moveto(joints, true, 0.1, 10, true);
        rate.sleep();
    }

    ros::shutdown();
    return 0;
}
