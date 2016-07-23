from __future__ import division

import rospy
import utils
import moveit_commander
import moveit_msgs.msg
import math
import angles

from tf import transformations
from std_srvs.srv import Empty

__author__ = 'Aijun Bai'


class Quadrotor(object):

    class Pose(object):
        def __init__(self, x, y, z, yaw):
            self.x = x
            self.y = y
            self.z = z
            self.yaw = yaw

        def to_joints(self):
            q = transformations.quaternion_from_euler(0, 0, self.yaw)
            joints = [self.x, self.y, self.z, q[0], q[1], q[2], q[3]]

            return joints

        @classmethod
        def from_joints(cls, joints):
            q = (joints[3], joints[4], joints[5], joints[6])
            euler = transformations.euler_from_quaternion(q)

            return cls(joints[0], joints[1], joints[2], euler[2])

        def __repr__(self):
            return self.__str__()

        def __str__(self):
            return '[x: {}, y: {}, z: {}, yaw: {}]'.format(
                self.x, self.y, self.z, angles.r2d(self.yaw))


    def __init__(self, display=True, debug=False):
        self._display = display
        self._debug = debug
        self._robot = moveit_commander.RobotCommander()
        self._group = moveit_commander.MoveGroupCommander("quadrotor")
        self._ready = False

        if self._display:
            self._display_trajectory_publisher = rospy.Publisher(
                '/move_group/display_planned_path',
                moveit_msgs.msg.DisplayTrajectory)

    def __del__(self):
        self._group.clear_pose_targets()

    def _call_service(self, service_name):
        if self._debug:
            print 'calling: {}'.format(service_name)
        rospy.wait_for_service(service_name)

        try:
            service = rospy.ServiceProxy(service_name, Empty)
            response = service()
        except Exception, e:
            raise e

    def engage(self):
        self._call_service('/engage')
        self._ready = True

    def shutdown(self):
        self._call_service('/shutdown')
        self._ready = False

    def get_current_joint_values(self):
        return self._group.get_current_joint_values()

    def _plan(self, joints=None):
        p = self._group.plan(joints)
        self.display(p)

    def display(self, plan):
        if self._display:
            display_trajectory = moveit_msgs.msg.DisplayTrajectory()
            display_trajectory.trajectory_start = self._robot.get_current_state()
            display_trajectory.trajectory.append(plan)
            self._display_trajectory_publisher.publish(display_trajectory)

    def set_joint_value_target(self, joints):
        self._group.set_joint_value_target(joints)


    def get_current_pose(self):
        return Quadrotor.Pose.from_joints(self.get_current_joint_values())

    def moveto_pose(self, pose, wait=True, error=0.1, iterations=10):
        self.moveto_joints(pose.to_joints(), wait, error, iterations)

    def moveto_joints(self, joints, wait=True, error=0.1, iterations=10):
        if not self._ready:
            self.engage()

        if self._debug:
            utils.pv('Quadrotor.Pose.from_joints(joints)', prefix='move to: ')

        err = error
        rate = rospy.Rate(5)

        start_pose = self._group.get_current_pose('base_link')
        constraints = moveit_msgs.msg.Constraints()
        constraints.name = 'fly'
        orientation_constraint = moveit_msgs.msg.OrientationConstraint()
        orientation_constraint.header.frame_id = '/world'
        orientation_constraint.header.stamp = rospy.Time.now()
        orientation_constraint.link_name = 'base_link'
        orientation_constraint.orientation = start_pose.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = math.pi
        orientation_constraint.weight = 1
        constraints.orientation_constraints.append(orientation_constraint)

        self._group.set_path_constraints(constraints)

        for i in range(iterations):
            self._group.set_joint_value_target(joints)
            if self._group.plan() is not None:
                self._group.go(wait)
                j = self.get_current_joint_values()
                err = utils.norm(joints, j)

                if self._debug:
                    utils.pv('i', 'Quadrotor.Pose.from_joints(j)', 'err')
                if err < error:
                    break
            rate.sleep()

        if err >= error:
            rospy.logwarn('moveto {} failed'.format(Quadrotor.Pose.from_joints(joints)))

        self._group.clear_path_constraints()
