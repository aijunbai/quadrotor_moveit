#include <ros/ros.h>
#include <actionlib/server/action_server.h>
#include <pthread.h>
#include <boost/bind.hpp>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <action_controller/MultiDofFollowJointTrajectoryAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

class Controller {
private:
    typedef actionlib::ActionServer<action_controller::MultiDofFollowJointTrajectoryAction> ActionServer;
    typedef ActionServer::GoalHandle GoalHandle;

public:
    Controller(ros::NodeHandle &n):
        node_(n),
        action_server_(node_, "action_controller/follow_multi_dof_joint_trajectory",
                       boost::bind(&Controller::goalCB, this, _1),
                       boost::bind(&Controller::cancelCB, this, _1),
                       false),
        has_active_goal_(false)
    {
        creato=0;

        zero_twist.header.frame_id = "/world";
        zero_twist.twist.linear.x = 0.0;
        zero_twist.twist.linear.y = 0.0;
        zero_twist.twist.linear.z = 0.0;
        zero_twist.twist.angular.x = 0.0;
        zero_twist.twist.angular.y = 0.0;
        zero_twist.twist.angular.z = 0.0;

        pose_topic = node_.advertise<geometry_msgs::PoseStamped>("/command/pose", 1);
        twist_topic = node_.advertise<geometry_msgs::TwistStamped>("/command/twist", 1);

        action_server_.start();

        ROS_WARN_STREAM(node_name << " ready!");
    }

public:
    static const char *node_name;

private:
    ros::NodeHandle node_;
    ActionServer action_server_;
    ros::Publisher pose_topic;
    ros::Publisher twist_topic;
    geometry_msgs::TwistStamped zero_twist;
    pthread_t trajectoryExecutor;
    int creato;

    bool has_active_goal_;
    GoalHandle active_goal_;
    trajectory_msgs::MultiDOFJointTrajectory_<std::allocator<void> > toExecute;

    void cancelCB(GoalHandle gh){
        ROS_WARN_STREAM("cancelCB()");

        if (active_goal_ == gh)
        {
            // Stops the controller.
            if(creato){
                ROS_WARN_STREAM("Stop thread");
                pthread_cancel(trajectoryExecutor);
                creato=0;
            }
            zero_twist.header.stamp = ros::Time::now();
            twist_topic.publish(zero_twist);

            // Marks the current goal as canceled.
            active_goal_.setCanceled();
            has_active_goal_ = false;
        }
    }

    void goalCB(GoalHandle gh){
        ROS_WARN_STREAM("goalCB()");

        if (has_active_goal_)
        {
            // Stops the controller.
            if(creato){
                pthread_cancel(trajectoryExecutor);
                creato=0;
            }
            zero_twist.header.stamp = ros::Time::now();
            twist_topic.publish(zero_twist);

            // Marks the current goal as canceled.
            active_goal_.setCanceled();
            has_active_goal_ = false;
        }

        gh.setAccepted();
        active_goal_ = gh;
        has_active_goal_ = true;
        toExecute = gh.getGoal()->trajectory;

        //controllore solo per il giunto virtuale Base
        if(pthread_create(&trajectoryExecutor, NULL, threadWrapper, this)==0){
            creato=1;
            ROS_WARN_STREAM("Thread for trajectory execution created");
        } else {
            ROS_WARN_STREAM("Thread creation failed!");
        }

    }

    static void* threadWrapper(void* arg) {
        Controller * mySelf=(Controller*)arg;
        mySelf->executeTrajectory();
        return NULL;
    }

    void executeTrajectory() {
        if(toExecute.joint_names[0] == "virtual_joint" && toExecute.points.size()>0) {
            for(int k=0; k<toExecute.points.size(); k++) {
                geometry_msgs::PoseStamped pose_msg;

                pose_msg.header.frame_id = "/world";
                pose_msg.header.stamp = ros::Time::now();
                pose_msg.header.seq = k;
                pose_msg.pose.position.x = toExecute.points[k].transforms[0].translation.x;
                pose_msg.pose.position.y = toExecute.points[k].transforms[0].translation.y;
                pose_msg.pose.position.z = toExecute.points[k].transforms[0].translation.z;
                pose_msg.pose.orientation = toExecute.points[k].transforms[0].rotation;
                pose_topic.publish(pose_msg);

                if (k + 1 < toExecute.points.size()) {
                    ros::Duration duration = toExecute.points[k+1].time_from_start -
                            toExecute.points[k].time_from_start;
                    duration.sleep();
                }
            }
        }
        active_goal_.setSucceeded();
        has_active_goal_=false;
        creato=0;
    }
};

const char *Controller::node_name = "action_controller_node";


int main(int argc, char** argv)
{
    ros::init(argc, argv, Controller::node_name);

    ros::NodeHandle node;
    Controller control(node);

    ros::spin();

    return 0;
}
