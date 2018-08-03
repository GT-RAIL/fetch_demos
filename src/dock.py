#!/usr/bin/env python
import rospy
import actionlib
import sys
from geometry_msgs.msg import PoseStamped
from fetch_auto_dock_msgs.msg import DockAction, DockGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class Dock:
    def __init__(self):
        # Create an action client
        rospy.loginfo("Connecting to the action servers...")
        self.dock_client = actionlib.SimpleActionClient("/dock", DockAction)
        self.dock_client.wait_for_server()
        self.move_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_client.wait_for_server()
        rospy.loginfo("Done!");

        self.navigate()

    def navigate(self):
        home = MoveBaseGoal()
        home.target_pose.header.frame_id = "map"
        home.target_pose.header.stamp = rospy.Time.now()
        # Must be updated to reflect dock position in current map
        # Up-to-date for map.pgm with SHA256 hash:
        # 48ad6b43d09cca62be72219b851db572062d18231da29ae27481495762f2dbba
        home.target_pose.pose.position.x = 8.601
        home.target_pose.pose.position.y = -25.801
        home.target_pose.pose.position.z = 0
        home.target_pose.pose.orientation.w = 0.985
        home.target_pose.pose.orientation.x = 0
        home.target_pose.pose.orientation.y = 0
        home.target_pose.pose.orientation.z = 0.173
        self.move_client.send_goal(home)
        self.move_client.wait_for_result()
        print "Navigation done!"
        self.dock()

    def dock(self):
        goal = DockGoal()
        goal.dock_pose.header.frame_id = "base_link"
        self.dock_client.send_goal(goal)
        print "Docking done!"
        sys.exit()

if __name__ == "__main__":
    rospy.init_node("docking")
    dock = Dock()
    rospy.spin()
