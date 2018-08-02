#!/usr/bin/env python
import rospy
import actionlib
from sensor_msgs.msg import Joy
from fetch_auto_dock_msgs.msg import DockAction, DockGoal

class DockExample:

    def __init__(self):
        # Create an action client
        rospy.loginfo("Connecting to the docking action server...")
        self.client = actionlib.SimpleActionClient("/dock", DockAction)
        self.client.wait_for_server()
        rospy.loginfo("Done!");

        self.dock_button = rospy.get_param("~dock_button", 13) # Circle button
        self.pressed = False
        self.pressed_last = None
        self.joy_sub = rospy.Subscriber("joy", Joy, self.joy_callback)

    def joy_callback(self, msg):
        try:
            if msg.buttons[self.dock_button] > 0:
                if not self.pressed:
                    self.pressed_last = rospy.Time.now()
                    self.pressed = True
                elif self.pressed_last and rospy.Time.now() > self.pressed_last + rospy.Duration(1.0):
                    self.do_action()
                    self.pressed_last = None
        except IndexError:
            rospy.logwarn("Button out of range")

    def do_action(self):
        # Create and send a goal
        goal = DockGoal()
        # goal.dock_pose.header.frame_id = "base_link"
        # goal.dock_pose.pose.position.x = 0.5
        # goal.dock_pose.pose.orientation.z = -1.0
        client.send_goal(goal)

if __name__ == "__main__":
    rospy.init_node("docking_example")
    c = DockExample()
    rospy.spin()