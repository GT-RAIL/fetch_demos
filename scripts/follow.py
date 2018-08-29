#!/usr/bin/env python

from __future__ import print_function, division

import rospy
import actionlib

from tf.listener import TransformListener
from tf2_py import ExtrapolationException

from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from control_msgs.msg import PointHeadAction, PointHeadGoal
from geometry_msgs.msg import PointStamped, Twist
from sensor_msgs.msg import LaserScan
from rail_people_detection_msgs.msg import Person, DetectionContext

from visualization_msgs.msg import Marker
import math

class Follow:

    def __init__(self):
        self.moving = False

        self.listener = TransformListener()

        self.head_client = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)
        self.head_client.wait_for_server()

        self.person_sub = rospy.Subscriber(
            "rail_people_detector/closest_person", Person, self.personCallback
        )
        self.laser_sub = rospy.Subscriber("base_scan", LaserScan, self.laserCallback)
        self.laser_pub = rospy.Publisher("collision_scan", LaserScan, queue_size=1)

        self.controllerID = None
        self.controllerPosition = None
        self.safeToTrack = True
        self.lastFaceTimestamp = rospy.Time(0)
        self.lastHeadTarget = None

        self.debug = rospy.Publisher("follow_face/debug", Marker, queue_size=1)
        self.cmdvel = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.marker = None
        self.last_processed = 0

    def laserCallback(self, msg):
        pos = self.controllerPosition

        # angle = -math.atan2(pos.point.y, pos.point.x)
        # distance = math.sqrt((pos.point.x) ** 2 + (pos.point.y) ** 2 + (pos.point.z) ** 2)
        # BUFFER = 2 # arc length in meters
        # arc = BUFFER / distance # in radians
        # minAngle = max(angle - (arc / 2), msg.angle_min)
        # minAngleIndex = int(minAngle / msg.angle_increment + abs(msg.angle_min / msg.angle_increment))
        # maxAngle = min(angle + (arc / 2), msg.angle_max)
        # maxAngleIndex = int(maxAngle / msg.angle_increment + abs(msg.angle_min / msg.angle_increment))

        laserPoints = list(msg.ranges) # [minAngleIndex:maxAngleIndex]

        collisionScan = LaserScan()
        collisionScan.header = msg.header
        collisionScan.angle_min = msg.angle_min #minAngle
        collisionScan.angle_max = msg.angle_max #maxAngle
        collisionScan.angle_increment = msg.angle_increment
        collisionScan.time_increment = msg.time_increment
        collisionScan.scan_time = msg.scan_time
        collisionScan.range_min = msg.range_min
        collisionScan.range_max = msg.range_max

        minDist = 999999
        minDistAngle = 0
        for i, point in enumerate(laserPoints):
            #pointAngle = (i + minAngleIndex) * msg.angle_increment + msg.angle_min
            pointAngle = i * msg.angle_increment + msg.angle_min
            if self.controllerPosition is not None:
                distanceFromController = math.sqrt((math.cos(pointAngle) * point - pos.x) ** 2 + (math.sin(pointAngle) * point - pos.y) ** 2)
                if distanceFromController < 1:
                    laserPoints[i] = 5
            if point > msg.range_min and point < minDist:
                minDist = point
                minDistAngle = pointAngle
        if minDist < 0.6:
            rospy.loginfo("Minimum distance is {} at {} degrees".format(round(minDist, 3), round(math.degrees(minDistAngle))))
        if minDist < 0.3:
            self.safeToTrack = False
            cmd = Twist()
            cmd.linear.x = -0.3
            self.cmdvel.publish(cmd)
            if self.controllerID is not None or self.controllerPosition is not None:
                rospy.loginfo("Temporarily killed controller due to obstacle")
                # For permanent killing of controller
                # self.controllerID = None
                # self.controllerPosition = None
        else:
            self.safeToTrack = True

        collisionScan.ranges = laserPoints
        self.laser_pub.publish(collisionScan)

    def personCallback(self, person):
        # Get the distance of this person from the robot
        person_distance = math.sqrt(person.pose.position.x ** 2 + person.pose.position.y ** 2)
        person_angle = math.degrees(math.atan2(person.pose.position.y, person.pose.position.x))

        # First look at the closest person if we know that the person is being
        # detected using the face detector
        if person.detection_context.pose_source == DetectionContext.POSE_FROM_FACE:
            self.lastFaceTimestamp = rospy.Time.now()

            # Point to the person if the distance between their current position
            # and the position we were looking at earlier has changed by more
            # than 2 cm
            goal = PointHeadGoal()
            goal.min_duration = rospy.Duration(0.0)
            goal.target = PointStamped(header=person.header, point=person.pose.position)
            head_distance = 999999999
            if self.lastHeadTarget is not None:
                head_distance = math.sqrt(
                    (person.pose.position.x - self.lastHeadTarget.point.x) ** 2
                    + (person.pose.position.y - self.lastHeadTarget.point.y) ** 2
                    + (person.pose.position.z - self.lastHeadTarget.point.z) ** 2
                )
            if head_distance > 0.02:
                self.lastHeadTarget = goal.target
                self.head_client.send_goal(goal)  # It's OK to not wait

            # Regardless of whether they were the original controller or not,
            # this person is the new controller if they are within 1m of us
            # if person_distance <= 1.0:
            if self.controllerID != person.id:
                rospy.loginfo("Setting controller ID to {}".format(person.id))
            self.controllerID = person.id
            self.controllerPosition = person.pose.position

        # Then check to see if the person we were tracking still in view.
        if self.controllerID == person.id:
            self.controllerPosition = person.pose.position

            # Check to see if we are not in collision. If so, MOVE!!!
            if self.safeToTrack:
                MAX_SPEED = 0.7
                cmd = Twist()

                if person_distance > 1:
                    targetSpeed = 0.15 * person_distance + 0.1
                    cmd.linear.x = min(targetSpeed, MAX_SPEED)
                elif person_distance < 0.7:
                    cmd.linear.x = -0.3

                if abs(person_angle) > 5:
                    cmd.angular.z = person_angle / 50
                self.cmdvel.publish(cmd)

        # Otherwise, we should stop trying to follow the person
        elif self.controllerID is not None:
            rospy.loginfo("Lost controller {}".format(self.controllerID))
            self.controllerID = None
            self.controllerPosition = None

    def loop(self):
        while not rospy.is_shutdown():
            if self.marker is not None:
                self.debug.publish(self.marker)

            # Reset head if no face found in 1 second
            if rospy.Time.now() - self.lastFaceTimestamp > rospy.Duration(1):
                self.lastHeadTarget = None

                # Reset head
                pos = PointStamped()
                pos.header.stamp = rospy.Time.now()
                pos.header.frame_id = "base_link"
                if self.controllerPosition is None:
                    pos.point.x = 1
                    pos.point.y = 0
                else:
                    pos.point.x = self.controllerPosition.x
                    pos.point.y = self.controllerPosition.y
                pos.point.z = 1.5
                goal = PointHeadGoal()
                goal.min_duration = rospy.Duration(0.5)
                goal.target = pos
                self.head_client.send_goal(goal)

            rospy.sleep(0.1)

if __name__=="__main__":
    rospy.init_node("follow_node")
    h = Follow()
    h.loop()
