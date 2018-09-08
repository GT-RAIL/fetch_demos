#!/usr/bin/env python

import argparse
import subprocess
import sys
from threading import Thread

import rospy
from sensor_msgs.msg import Joy
from moveit_msgs.msg import MoveItErrorCodes, PlanningScene
from moveit_python import MoveGroupInterface, PlanningSceneInterface

class MoveItThread(Thread):

    def __init__(self):
        Thread.__init__(self)
        self.start()

    def run(self):
        self.process = subprocess.Popen(["roslaunch", "fetch_moveit_config", "move_group.launch", "--wait"])
        _, _ = self.process.communicate()

    def stop(self):
        self.process.send_signal(subprocess.signal.SIGINT)
        self.process.wait()

def is_moveit_running():
    output = subprocess.check_output(["rosnode", "info", "move_group"], stderr=subprocess.STDOUT)
    if output.find("unknown node") >= 0:
        return False
    if output.find("Communication with node") >= 0:
        return False
    return True

class TuckThread(Thread):

    def __init__(self):
        Thread.__init__(self)
        self.client = None
        self.start()

    def run(self):
        move_thread = None
        if not is_moveit_running():
            rospy.loginfo("starting moveit")
            move_thread = MoveItThread()

        rospy.loginfo("Waiting for MoveIt...")
        self.client = MoveGroupInterface("arm_with_torso", "base_link")
        rospy.loginfo("...connected")

        # Padding does not work (especially for self collisions)
        # So we are adding a box above the base of the robot
        scene = PlanningSceneInterface("base_link")
        scene.addBox("keepout", 0.2, 0.5, 0.05, 0.15, 0.0, 0.375)

        joints = ["torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        pose = [0.05, 0.29, -0.60, -0.51, 1.45, 0.52, 0.88, -1.97]
        while not rospy.is_shutdown():
            result = self.client.moveToJointPosition(joints,
                                                     pose,
                                                     0.0,
                                                     max_velocity_scaling_factor=0.5)
            if result and result.error_code.val == MoveItErrorCodes.SUCCESS:
                scene.removeCollisionObject("keepout")
                if move_thread:
                    move_thread.stop()

                # On success quit
                # Stopping the MoveIt thread works, however, the action client
                # does not get shut down, and future tucks will not work.
                # As a work-around, we die and roslaunch will respawn us.
                rospy.signal_shutdown("done")
                sys.exit(0)
                return

    def stop(self):
        if self.client:
            self.client.get_move_action().cancel_goal()
        # Stopping the MoveIt thread works, however, the action client
        # does not get shut down, and future tucks will not work.
        # As a work-around, we die and roslaunch will respawn us.
        rospy.signal_shutdown("failed")
        sys.exit(0)

class TuckArmTeleop:

    def __init__(self):
        self.tuck_button = rospy.get_param("~untuck_button", 5)  # default button is the right button
        self.deadman = rospy.get_param("~deadman_button", 10)
        self.tucking = False

        self.pressed = False
        self.pressed_last = None

        self.joy_sub = rospy.Subscriber("joy", Joy, self.joy_callback)

    def joy_callback(self, msg):
        if self.tucking:
            # Only run once
            if msg.buttons[self.deadman] <= 0:
                # Deadman has been released, don't tuck
                rospy.loginfo("Stopping untuck thread")
                self.tuck_thread.stop()
            return
        try:
            if msg.buttons[self.tuck_button] > 0 and msg.buttons[self.deadman] > 0:
                if not self.pressed:
                    self.pressed_last = rospy.Time.now()
                    self.pressed = True
                elif self.pressed_last and rospy.Time.now() > self.pressed_last + rospy.Duration(1.0):
                    # Tuck the arm
                    self.tucking = True
                    rospy.loginfo("Starting untuck thread")
                    self.tuck_thread = TuckThread()
            else:
                self.pressed = False
        except KeyError:
            rospy.logwarn("tuck_button is out of range")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Untuck the arm, either immediately or as a joystck-controlled server.")
    parser.add_argument("--joystick", action="store_true", help="Run as server that untucks on command from joystick.")
    args, unknown = parser.parse_known_args()

    rospy.init_node("untuck_arm")
    rospy.loginfo("New untuck script running")

    if args.joystick:
        t = TuckArmTeleop()
        rospy.spin()
    else:
        rospy.loginfo("Untucking the arm")
        TuckThread()
