#!/usr/bin/env python3

# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg

import sys
import copy
import rospy
import tf
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg


class MoveGroupPythonInterface(object):
    """MoveGroupPythonInterfacemain"""

    def __init__(self):
        super(MoveGroupPythonInterface, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface", anonymous=True)

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
        joint_goal[5] = -1.5707963268 
        joint_goal[4] = -1.5707963268
        joint_goal[3] = 0
        joint_goal[2] = -2.617993878
        joint_goal[1] = 0
        move_group.go(joint_goal, wait=True)
        joint_goal[0] = 0
        move_group.go(joint_goal, wait=True)
        move_group.stop()

        return 

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
        joint_goal[0] = -1.5707963268
        move_group.go(joint_goal, wait=True)
        move_group.stop()

        return 

    def go_to_pose_goal(self, x, y, z, xo, yo, zo, wo):
        current_pose = self.move_group.get_current_pose().pose
        move_group = self.move_group
        pose_goal = geometry_msgs.msg.Pose()
        # pose_goal.orientation.x = 0.5837
        # pose_goal.orientation.y = 0.5835
        # pose_goal.orientation.z = 0.399
        # pose_goal.orientation.w = -0.399
        # # pose_goal.position.x = -0.16
        # # pose_goal.position.y = 0.12
        # # pose_goal.position.z = 0.50
        pose_goal.orientation = current_pose.orientation
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        pose_goal.orientation.x = xo
        pose_goal.orientation.y = yo
        pose_goal.orientation.z = zo
        pose_goal.orientation.w = wo
        pose_goal.orientation = current_pose.orientation
        print(pose_goal)

        move_group.set_pose_target(pose_goal)

        success = move_group.go(wait=True)

        move_group.stop()

        move_group.clear_pose_targets()

        return

    def plan_cartesian_path(self, scale=1):

        move_group = self.move_group

        waypoints = []

        wpose = move_group.get_current_pose().pose

        # wpose.position.x = scale * -0.11
        # wpose.position.y = scale * 0.12
        # wpose.position.z = scale * 0.50
        # waypoints.append(copy.deepcopy(wpose))

        wpose.position.x = scale * -0.11
        wpose.position.y = scale * 0.12
        wpose.position.z = scale * 0.50
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0 
        )
        move_group.execute(plan, wait=True)
        return plan, fraction

    def go_scan(self):
        
        self.go_to_pose_goal(-0.1,-0.35,0.37, 0,0,0,0)
        
        return

    def go_aruco(self, x, y, z):#(x = ลดRight เพิ่มLeft) (Y = ลดForward เพิ่มBack) (z = ลดDown เพิ่มUp)
        current_pose = self.move_group.get_current_pose().pose
        move_group = self.move_group
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation = current_pose.orientation
        pose_goal.position.x = x 
        pose_goal.position.y = y + 0.1
        pose_goal.position.z = z - 0.05


        move_group.set_pose_target(pose_goal)

        success = move_group.go(wait=True)

        move_group.stop()

        move_group.clear_pose_targets()

        return
        
    def plan_cartesian(self,x,y,z):

        move_group = self.move_group
        waypoints = []
        wpose = move_group.get_current_pose().pose

        wpose.position.x = x
        wpose.position.y = y
        wpose.position.z = z
        waypoints.append(copy.deepcopy(wpose))

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
            rate.sleep()
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    return trans, rot

def gripper_pub(inst):
    

    pub = rospy.Publisher(
        "Robotiq2FGripperRobotOutput", outputMsg.Robotiq2FGripper_robot_output
    )
    command = outputMsg.Robotiq2FGripper_robot_output()
    command.rACT = 1
    command.rGTO = 1
    command.rSP = 255
    command.rFR = 150
    if inst == "activate":
        command = outputMsg.Robotiq2FGripper_robot_output()
        command.rACT = 1
        command.rGTO = 1
        command.rSP = 255
        command.rFR = 150
    if inst == "open":
        command.rPR = 0
    if inst == "close":
        command.rPR = 255
    
    try:
        command.rPR = int(inst)
        if command.rPR > 255:
            command.rPR = 255
        if command.rPR < 0:
            command.rPR = 0
    except ValueError:
        pass

    pub.publish(command)
    rospy.sleep(0.1)
    

    return

def main():
    try:

        tutorial = MoveGroupPythonInterface()
        gripper_pub("activate")
        # input("============ Press `Enter` to go rest position ")
        # tutorial.go_to_rest_state()
        print(tf_listen('id0'))
        # input("============ Press `Enter` to go ready position ")
        # tutorial.go_to_ready_state()
       
        # input("============ Press `Enter` to go scan ")
        # tutorial.go_scan()
        #(x = ลดRight เพิ่มLeft) (Y = ลดForward เพิ่มBack) (z = ลดDown เพิ่มUp)
        # tutorial.go_aruco(-0.1,-0.35,0.37)
        # tutorial.plan_cartesian(x,x,0.5)
        
        # print(rospy.get_param('move_group_python_interface_run/sq'))

        # input("============ Press `Enter` to go press button in sequnce")
        # x,y,z = tf_listen('id0')
        # x1,y1,z1 = tf_listen('id5')
        # tutorial.go_to_pose_goal(x,y,z)
        # tutorial.go_aruco(x,y,z)
        # input("============ Press `Enter` to go scan ")
        
        # tutorial.go_aruco(x1,y1,z1)
        
        print("============ Python tutorial demo complete!")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return
    
if __name__ == "__main__":
    main()
