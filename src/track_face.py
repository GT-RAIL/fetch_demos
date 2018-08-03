#!/usr/bin/env python

from threading import Lock

import rospy
import actionlib
import time

from tf.listener import TransformListener
from tf.broadcaster import TransformBroadcaster
from tf2_py import ExtrapolationException

from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from control_msgs.msg import PointHeadAction, PointHeadGoal
from geometry_msgs.msg import PointStamped
from people_msgs.msg import PositionMeasurementArray

import math

class FollowFace:

    def __init__(self):
        self.moving = False

        self.listener = TransformListener()
        self.broadcaster = TransformBroadcaster()

        self.client = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)
        self.client.wait_for_server()

        self.face_sub = rospy.Subscriber("face_detector/people_tracker_measurements_array", PositionMeasurementArray, self.faceCallback)
        
        self.lastFaceCallback = time.time()
        self.lastHeadTarget = None

    def faceCallback(self, msg):
        self.lastFaceCallback = time.time()
        # Find closest face to look at
        closestFace = None
        closestDistance = 999999999
        for face in msg.people:
            pos = PointStamped()
            pos.header = face.header
            pos.point = face.pos
            try:
                pos = self.listener.transformPoint("base_link", pos)
            except ExtrapolationException:
                return

            distance = math.sqrt(pos.point.x ** 2 + pos.point.y ** 2 + pos.point.z ** 2)
            if distance < closestDistance:
                closestFace = pos
                closestDistance = distance
        
        if closestFace is None:
            return
        goal = PointHeadGoal()
        goal.min_duration = rospy.Duration(0.0)
        goal.target = closestFace
        
        distance = 999999999
        if self.lastHeadTarget is not None:
            distance = math.sqrt(
                (closestFace.point.x - self.lastHeadTarget.point.x) ** 2 +
                (closestFace.point.y - self.lastHeadTarget.point.y) ** 2 +
                (closestFace.point.z - self.lastHeadTarget.point.z) ** 2
            )
        # Prevents jitter from insignificant face movements
        if distance > 0.02:
            self.lastHeadTarget = goal.target
            self.client.send_goal(goal)
            self.client.wait_for_result()

    def loop(self):
        while not rospy.is_shutdown():
            # Reset head if no face found in 1 second
            if time.time() - self.lastFaceCallback > 1:
                self.lastHeadTarget = None
                # Reset head
                pos = PointStamped()
                pos.header.stamp = rospy.Time.now()
                pos.header.frame_id = "base_link"
                pos.point.x = 1
                pos.point.y = 0
                pos.point.z = 1.5
                goal = PointHeadGoal()
                goal.min_duration = rospy.Duration(0.5)
                goal.target = pos
                #self.client.cancel_all_goals()
                self.client.send_goal(goal)
            rospy.sleep(0.1)

if __name__=="__main__":
    rospy.init_node("follow_face_node")
    h = FollowFace()
    h.loop()
