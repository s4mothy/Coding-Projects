#!/usr/bin/env python3

"""
Bug0 Path-Finding Algorithm for Use With ROS Stage
"""

import sys
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from bug_position import Position
from bug_sensor import Sensors
from tf.transformations import euler_from_quaternion

# MACROS
FORWARD = 0
LEFT = 1
RIGHT = 2
STOP = 3

# CONSTANTS
ALLOWABLE_GOAL_ERROR = 0.1 
WALL_BUFFER = 1.0
ALLOWABLE_WALL_DRIFT = 0.1
FAR_ENUF_AWAY = 10.0
USE_TARGET_THETA = False

bugPosition = Position()
bugSensors = Sensors()

def PositionHandler(data):
    p = data.pose.pose.position
    oriX = data.pose.pose.orientation.x
    oriY = data.pose.pose.orientation.y
    oriZ = data.pose.pose.orientation.z
    oriW = data.pose.pose.orientation.w
    q = (oriX, oriY, oriZ, oriW)
    t = euler_from_quaternion(q)[2]
    bugPosition.UpdatePosition(p.x, p.y, t)

def SensorHandler(data):
    bugSensors.UpdateReadings(data)

def GetTargetCoords():

    # converts input string into Tuple, or returns None
    def ParseTuple(string):
        try:
            s = eval(string)
            if type(s) == tuple:
                return s
            return 
        except:
            return

    # messages for user
    requestMsg = "Please enter the target coordinates in the following format (target_x, target_y, target_theta) or 'q' to quit: "
    requestMsgNT = "Please enter the target coordinates in the following format (target_x, target_y) or 'q' to quit: "
    typeErrorMsg = "Error: Coordinates must be a tuple of floats, formatted as (target_x, target_y, target_theta)!"
    typeErrorMsgNT = "Error: Coordinates must be a tuple of floats, formatted as (target_x, target_y)!"

    def GetInput(reqMsg, errMsg):
        tx, ty, tt = 0.0, 0.0, 0.0
        targetCoordsStr = input(reqMsg)
        if targetCoordsStr == "q":
            sys.exit()
        targetCoords = ParseTuple(targetCoordsStr)
        while(type(targetCoords) != tuple):
            print(errMsg)
            targetCoordsStr = input(reqMsg)
            if targetCoordsStr == "q":
                sys.exit()
            targetCoords = ParseTuple(targetCoordsStr)
        if USE_TARGET_THETA:
            tx, ty, tt = targetCoords[0], targetCoords[1], targetCoords[2]
        else:
            tx, ty = targetCoords[0], targetCoords[1]
        return (tx, ty, tt)

    if USE_TARGET_THETA:
        return GetInput(requestMsg, typeErrorMsg)
    else:
        return GetInput(requestMsgNT, typeErrorMsgNT)
    
    

class Bug0:
    def __init__(self, tx, ty, tt):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.tx = tx
        self.ty = ty
        self.tt = tt

    def move (self, dir):
        com = Twist()
        if dir == FORWARD:
            com.linear.x = 0.25
        elif dir == LEFT:
            com.angular.z = 0.75
            com.linear.x = 0.25
        elif dir == RIGHT:
            com.angular.z = -0.75
            com.linear.x = 0.25
        elif dir == STOP:
            pass
        self.pub.publish(com)

    # turns the bug until it is roughly facing the goal, then moves forward until it becomes close to an obstacle
    def MoveUntilObstacle(self, rate):
        print("moving until obstacle...")
        while bugPosition.DistToGoal(self.tx, self.ty) > ALLOWABLE_GOAL_ERROR:
            (frontReading, _) = bugSensors.GetReadings()
            if frontReading <= WALL_BUFFER:
                return True
            if bugPosition.FacingGoal(self.tx, self.ty):
                self.move(FORWARD)
            elif bugPosition.LocalGoalTheta(self.tx, self.ty) > 0:
                self.move(LEFT)
            else:
                self.move(RIGHT)
            rospy.sleep(0.1)
        return False

    # turns and travels left around the obstacle until it can head toward the goal again
    def FollowObstacle(self, rate):
        print("following obstacle")
        # turns left until front is somewhat facing along wall
        while bugSensors.GetReadings()[0] <= WALL_BUFFER:
            self.move(LEFT)
            rospy.sleep(0.1)
        # moves along wall by drifting out when too close, in when too far, and forward when in the sweet spot
        while not self.CanHeadTowardGoal():
            (frontReading, rightReading) = bugSensors.GetReadings()
            if frontReading <= WALL_BUFFER:
                self.move(LEFT)
            elif -ALLOWABLE_WALL_DRIFT <= rightReading - WALL_BUFFER <= ALLOWABLE_WALL_DRIFT:
                self.move(FORWARD)
            elif rightReading - WALL_BUFFER > ALLOWABLE_WALL_DRIFT:
                self.move(RIGHT)
            else:
                self.move(LEFT)
            rospy.sleep(0.1)

    # checks if goal heading is clear within the bugs vision
    def CanHeadTowardGoal(self):
        localGoalTheta = bugPosition.LocalGoalTheta(self.tx, self.ty)
        readingAtGoal = bugSensors.ReadingAtAngle(localGoalTheta)
        if readingAtGoal > FAR_ENUF_AWAY:
            print("heading to goal...")
            return True
        return False

    def TurnToTargetTheta(self):
        while(not bugPosition.AtTargetTheta(self.tt)):
            self.move(RIGHT)

def run_bug0_algorithm():
    print("starting up...")
    # Name your node
    rospy.init_node("bug_mover", anonymous=True)

    # The main loop will run at a rate of 10Hz, i.e., 10 times per second.
    rate = rospy.Rate(10)

    # Subscribe to topics for directing movement, then waits briefly to get data
    rospy.Subscriber('base_pose_ground_truth', Odometry, PositionHandler)
    rospy.Subscriber('base_scan', LaserScan, SensorHandler)
    rate.sleep()

    # Get final coordinates from user
    tx, ty, tt = GetTargetCoords()

    # Generate bug
    bug = Bug0(tx, ty, tt)

    # heads toward goal, until obstacle, then goes around and repeats until reaching the goal
    while (not rospy.is_shutdown()) and (bugPosition.DistToGoal(tx, ty) > ALLOWABLE_GOAL_ERROR):
        bonkObstacle = bug.MoveUntilObstacle(rate)
        if bonkObstacle:
            bug.FollowObstacle(rate)
        # rate.sleep()
    if USE_TARGET_THETA:
        bug.TurnToTargetTheta()
    print("arrived at goal. Current Coordinates: (%f, %f, %f)" % bugPosition.GetPosition())
    
if __name__ == "__main__":
    run_bug0_algorithm()
