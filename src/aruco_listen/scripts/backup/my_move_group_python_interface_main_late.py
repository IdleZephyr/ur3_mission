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
        self.count = 0

    def go_to_rest_state(self):

        move_group = self.move_group
        # using Radians value
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
        # using Radians value
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

    def go_to_pre(self):

        move_group = self.move_group
        # using Radians value
        joint_goal = move_group.get_current_joint_values()
        joint_goal[5] = -1.5707963268
        joint_goal[4] = -1.5707963268
        joint_goal[3] = -3.1415926536
        joint_goal[2] = 1.5707963268
        joint_goal[1] = -1.5707963268
        move_group.go(joint_goal, wait=True)
        joint_goal[0] = -1.5707963268
        move_group.go(joint_goal, wait=True)
        move_group.stop()

        return

    def go_to_pose_goal(self, x, y, z):
        current_pose = self.move_group.get_current_pose().pose
        move_group = self.move_group
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation = current_pose.orientation
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        # pose_goal.orientation.x = current_pose.orientation.x
        # pose_goal.orientation.y = current_pose.orientation.y
        # pose_goal.orientation.z = current_pose.orientation.z
        # pose_goal.orientation.w = current_pose.orientation.w

        print(pose_goal)

        move_group.set_pose_target(pose_goal)

        success = move_group.go(wait=True)

        move_group.stop()

        move_group.clear_pose_targets()

        return

    def go_scan(self, count):
        current_pose = self.move_group.get_current_pose().pose

        x = current_pose.position.x
        y = current_pose.position.y
        z = current_pose.position.z
        # self.go_to_pose_goal(-0.1,-0.35,0.37, 0,0,0,0)
        if count == 1:
            self.plan_cartesian(x, y, z)
        elif count == 2:
            self.plan_cartesian(x+0.2, y, z)
        elif count == 3:
            self.plan_cartesian(x, y, z+0.1)
        elif count == 4:
            self.plan_cartesian(x-0.2, y, z)
            rospy.sleep(0.1)
            self.go_to_pose_goal(-0.141, y, z)
        elif count == 5:
            self.go_to_pose_goal(x, y, z-0.1)
        else:
            self.go_to_rest_state
            return

    # (x = ลดRight เพิ่มLeft) (Y = ลดForward เพิ่มBack) (z = ลดDown เพิ่มUp)
    def go_aruco(self, x, y, z):
        current_pose = self.move_group.get_current_pose().pose
        move_group = self.move_group
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation = current_pose.orientation
        pose_goal.position.x = x
        pose_goal.position.y = y + 0.2
        pose_goal.position.z = z - 0.05

        move_group.set_pose_target(pose_goal)

        success = move_group.go(wait=True)

        move_group.stop()

        move_group.clear_pose_targets()

        return

    def plan_cartesian(self, x, y, z):

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

    def tf_listen(self, id):
        listener = tf.TransformListener()
        rate = rospy.Rate(10.0)

        while not rospy.is_shutdown():
            try:
                (trans, rot) = listener.lookupTransform(
                    'base_link', 'marker_'+id, rospy.Time(0))
                rate.sleep()
                x = trans[0]
                y = trans[1]
                z = trans[2]
                return x, y, z
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                self.count = self.count+1
                self.go_scan(self.count)
                if self.count > 1 and self.count < 6:
                    print("marker not found start scan")
                    continue
                if self.count > 6:
                    print("marker not found ")
                    return
                continue

    def gripper_pub(self, inst):

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
        if inst == "reset":
            command = outputMsg.Robotiq2FGripper_robot_output()
            command.rACT = 0

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

    def push(self, x, y, z):
        # self.gripper_pub("activate")
        self.gripper_pub("close")

        self.go_aruco(x, y, z)
        self.plan_cartesian(x, y+0.075, z-0.05)
        self.plan_cartesian(x, y+0.2, z-0.05)

        return

    def release(self, x, y, z):
        move_group = self.move_group

        # self.gripper_pub("activate")
        self.gripper_pub("open")
        self.go_aruco(x, y, z)
        self.plan_cartesian(x, y+0.06, z-0.05)
        self.gripper_pub("close")
        joint_goal = move_group.get_current_joint_values()
        joint_goal[5] = 0
        move_group.go(joint_goal, wait=True)
        self.gripper_pub("open")
        joint_goal[5] = -1.5707963268
        move_group.go(joint_goal, wait=True)
        self.plan_cartesian(x, y+0.2, z-0.05)

        self.go_aruco(x, y, z)
        return


def main():
    try:
        # (x = ลดRight เพิ่มLeft) (Y = ลดForward เพิ่มBack) (z = ลดDown เพิ่มUp)
        ur3 = MoveGroupPythonInterface()
        
        ur3.gripper_pub("activate")
        ur3.gripper_pub("close")
        # print(ur3.move_group.get_current_pose().pose)
        # input("============ Press `Enter` to go rest position ")
        # ur3.go_to_rest_state()
        # print(ur3.tf_listen("id11"))
        input("============ Press `Enter` to go ready position ")
        ur3.go_to_ready_state()

        input("============ Press `Enter` to go scan ")
        sequnce = rospy.get_param(
            'move_group_python_interface_run/sq').split(",")
        action = rospy.get_param(
            'move_group_python_interface_run/ac').split(",")

        pos_store = []
        for i in sequnce:
            print("now id: "+i)
            pos = ur3.tf_listen(i)
            ur3.count = 0
            # pos = 1, 2, 3
            pos_store.append(pos)
            print(pos_store)

        # print(ur3.tf_listen('id0'))
        # print(rospy.get_param('move_group_python_interface_run/sq'))

        input("============ Press `Enter` to go press button in sequnce")

        for i in range(len(action)):
            print(i)
            print(pos_store[i][0])
            if action[i] == "push":
                ur3.go_to_pre()
                ur3.push(pos_store[i][0],pos_store[i][1], pos_store[i][2])
            if action[i] == "release":
                ur3.go_to_pre()
                ur3.release(pos_store[i][0], pos_store[i][1], pos_store[i][2])
        ur3.go_to_pre()
        ur3.go_to_ready_state()
        ur3.gripper_pub("reset")
        # ur3.go_to_rest_state()
        
        # x,y,z = ur3.tf_listen('id5')
        # ur3.go_to_pre ()
        # ur3.push(x,y,z)
        # ur3.release(x,y,z)
        # ur3.go_to_pre ()
        # ur3.go_to_ready_state()

        print("============ Python complete!")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
