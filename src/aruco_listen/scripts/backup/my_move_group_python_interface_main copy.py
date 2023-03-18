#!/usr/bin/env python3

# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import tf
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

from moveit_commander.conversions import pose_to_list


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()

        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")

        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def go_to_rest_state(self):

        move_group = self.move_group
        #using Radians value
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        move_group.go(joint_goal, wait=True)
        joint_goal[1] = 0
        move_group.go(joint_goal, wait=True)
        joint_goal[2] = -2.617993878
        move_group.go(joint_goal, wait=True)
        joint_goal[3] = 0
        move_group.go(joint_goal, wait=True)
        joint_goal[4] = -1.5707963268
        move_group.go(joint_goal, wait=True)
        joint_goal[5] = -1.5707963268 
        move_group.go(joint_goal, wait=True)

        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_ready_state(self):

        move_group = self.move_group
        #using Radians value
        joint_goal = move_group.get_current_joint_values()
        
        joint_goal[5] = -1.5707963268 
        joint_goal[4] = -1.5707963268
        joint_goal[3] = -0.6108652382 
        joint_goal[2] = -2.617993878
        joint_goal[1] = 0
        move_group.go(joint_goal, wait=True)
        joint_goal[0] = 1.5707963268 
        move_group.go(joint_goal, wait=True)

        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self, x, y, z,w):
        current_pose = self.move_group.get_current_pose().pose
        move_group = self.move_group
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation = current_pose.orientation
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        print(pose_goal)

        move_group.set_pose_target(pose_goal)

        success = move_group.go(wait=True)

        move_group.stop()

        move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def plan_cartesian_path(self, scale=1):

        move_group = self.move_group

        waypoints = []

        wpose = move_group.get_current_pose().pose

        print(wpose)
        wpose.position.x = scale * -0.11
        wpose.position.y = scale * 0.12
        wpose.position.z = scale * 0.50
        waypoints.append(copy.deepcopy(wpose))
        
        print(wpose)


        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0 
        )
        move_group.execute(plan, wait=True)
        return plan, fraction


def tf_listen(id):
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('base_link', 'marker_'+id, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        x, y, z = trans[0], trans[1], trans[2]
        rate.sleep()
        break
    return x, y, z

def main():
    try:

        tutorial = MoveGroupPythonInterfaceTutorial()
        
        # input("============ Press `Enter` to go ready position using a joint state goal ...")
        # tutorial.go_to_rest_state()
        # input("============ Press `Enter` to go ready position using a joint state goal ...")
        tutorial.go_to_ready_state()
        input("============ Press `Enter` to go scan using a Cartesian path ...")
        tutorial.plan_cartesian_path()
        # tutorial.go_to_pose_goal(0.4,0.1,0.4,0.8)


        # input(
        #     "============ Press `Enter` to execute a movement using a joint state goal ..."
        # )
        # print(tf_listen('id1'))
        # x,y,z = tf_listen('id1')
        # x = float(input("Enter a value for x: "))
        # y = float(input("Enter a value for y: "))
        # z = float(input("Enter a value for z: "))
        # w = float(input("Enter a value for w: "))
        # input("============ Press `Enter` to execute a movement using a pose goal ...")
        
        # tutorial.go_to_pose_goal(x,y,z)

        # input(
        #     "============ Press `Enter` to plan and execute a path with an attached collision object ..."
        # )
        # cartesian_plan, fraction = tutorial.plan_cartesian_path(scale=-1)
        # tutorial.execute_plan(cartesian_plan)

        print("============ Python tutorial demo complete!")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return
    
if __name__ == "__main__":
    main()
