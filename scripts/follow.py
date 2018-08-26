#!/usr/bin/env python

import rospy
import actionlib
import time

from tf.listener import TransformListener
from tf.broadcaster import TransformBroadcaster
from tf2_py import ExtrapolationException

from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from control_msgs.msg import PointHeadAction, PointHeadGoal
from geometry_msgs.msg import PointStamped, Twist
from sensor_msgs.msg import LaserScan

from people_msgs.msg import PositionMeasurementArray
from leg_tracker.msg import Person, PersonArray

from visualization_msgs.msg import Marker
import math

class Follow:

    def __init__(self):
        self.moving = False

        self.listener = TransformListener()
        self.broadcaster = TransformBroadcaster()

        self.client = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)
        self.client.wait_for_server()

        self.face_sub = rospy.Subscriber("face_detector/people_tracker_measurements_array", PositionMeasurementArray, self.faceCallback)
        self.leg_sub = rospy.Subscriber("people_tracked", PersonArray, self.legCallback)
        self.laser_sub = rospy.Subscriber("base_scan", LaserScan, self.laserCallback)
        self.laser_pub = rospy.Publisher("collision_scan", LaserScan, queue_size=1)

        self.people = []
        self.controllerID = None
        self.controllerPosition = None
        self.safeToTrack = True
        self.lastFaceCallback = time.time()
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
                distanceFromController = math.sqrt((math.cos(pointAngle) * point - pos.point.x) ** 2 + (math.sin(pointAngle) * point - pos.point.y) ** 2)
                if distanceFromController < 1:
                    laserPoints[i] = 5
            if point > msg.range_min and point < minDist:
                minDist = point
                minDistAngle = pointAngle
        if minDist < 0.6:
            print "Minimum distance is {} at {} degrees".format(round(minDist, 3), round(math.degrees(minDistAngle)))
        if minDist < 0.3:
            self.safeToTrack = False
            cmd = Twist()
            cmd.linear.x = -0.3
            self.cmdvel.publish(cmd)
            if self.controllerID is not None or self.controllerPosition is not None:
                print("Temporarily killed controller due to obstacle")
                # For permanent killing of controller
                # self.controllerID = None
                # self.controllerPosition = None
        else:
            self.safeToTrack = True

        collisionScan.ranges = laserPoints
        self.laser_pub.publish(collisionScan)

    def legCallback(self, msg):
        self.people = []
        controllerFound = False
        for person in msg.people:
            pos = PointStamped()
            pos.header = msg.header
            pos.point = person.pose.position
            try:
                pos = self.listener.transformPoint("base_link", pos)
            except ExtrapolationException:
                return
            self.people.append((person.id, pos))

            if person.id == self.controllerID and self.safeToTrack:
                controllerFound = True
                self.controllerPosition = pos

                angle = math.degrees(math.atan2(pos.point.y, pos.point.x))
                distance = math.sqrt((pos.point.x) ** 2 + (pos.point.y) ** 2 + (pos.point.z) ** 2)
                # Positive to the left
                # Negative to the right
                #print("Angle is {}, distance is {}".format(round(angle, 3), round(distance, 3)))

                MAX_SPEED = 0.7
                cmd = Twist()

                if distance > 1:
                    # Connects (1, 0.25) and (4, 0.7)
                    targetSpeed = 0.15 * distance + 0.1
                    cmd.linear.x = min(targetSpeed, MAX_SPEED)
                elif distance < 0.7:
                    cmd.linear.x = -0.3

                if abs(angle) > 5:
                    cmd.angular.z = angle / 50
                self.cmdvel.publish(cmd)

        if self.controllerID is not None and not controllerFound:
            print("Killed controller")
            self.controllerID = None
            self.controllerPosition = None

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

        if len(self.people) > 0 and self.safeToTrack:
            closestID = None
            closestDistance = 999999999
            for person in self.people:
                pos = person[1]
                distance = math.sqrt((pos.point.x - closestFace.point.x) ** 2 + (pos.point.y - closestFace.point.y) ** 2)
                if distance < closestDistance:
                    closestID = person[0]
                    closestDistance = distance
            if closestDistance < 1:
                self.controllerID = closestID
                print("Controller ID is {}".format(self.controllerID))
            else:
                print("No controller because closest distance was {}".format(closestDistance))

        #self.client.cancel_all_goals()
        distance = 999999999
        if self.lastHeadTarget is not None:
            distance = math.sqrt(
                (closestFace.point.x - self.lastHeadTarget.point.x) ** 2 +
                (closestFace.point.y - self.lastHeadTarget.point.y) ** 2 +
                (closestFace.point.z - self.lastHeadTarget.point.z) ** 2
            )
        if distance > 0.02:
            self.lastHeadTarget = goal.target
            self.client.send_goal(goal)
            self.client.wait_for_result()

    def loop(self):
        while not rospy.is_shutdown():
            if self.marker is not None:
                self.debug.publish(self.marker)
            # Reset head if no face found in 1 second
            if time.time() - self.lastFaceCallback > 1:
                self.lastHeadTarget = None
                # Reset head
                pos = PointStamped()
                pos.header.stamp = rospy.Time.now()
                pos.header.frame_id = "base_link"
                if self.controllerPosition is None:
                    pos.point.x = 1
                    pos.point.y = 0
                else:
                    pos.point.x = self.controllerPosition.point.x
                    pos.point.y = self.controllerPosition.point.y
                pos.point.z = 1.5
                goal = PointHeadGoal()
                goal.min_duration = rospy.Duration(0.5)
                goal.target = pos
                #self.client.cancel_all_goals()
                self.client.send_goal(goal)
            rospy.sleep(0.1)

if __name__=="__main__":
    rospy.init_node("follow_node")
    h = Follow()
    h.loop()
