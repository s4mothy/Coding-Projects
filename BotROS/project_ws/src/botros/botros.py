#!/usr/bin/env python3
import rospy, math
from PIL import Image
import numpy as np
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist, Pose, Point, PoseWithCovariance, Quaternion
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from matplotlib.image import imread
from tf.transformations import euler_from_quaternion


class BotRos:
    def __init__(self, img):
        rospy.init_node("botros_mover", anonymous=True)
        self.rate = rospy.Rate(10) # Rate of 10 hz
        self.image = np.asarray(Image.open(img)).copy() # The image as an array of Trues and Falses
        self.x = 0 # current x coord
        self.y = 0 # current y coord
        rospy.Subscriber("/odom", Odometry, self.position_handler) # Updates the position
        self.theta = 0 # Initializes theta
        self.ori = Quaternion() # Initializes the orientation
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1) # Changes position

        #added for visualization
        self.penEngaged = True
        self.vis_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=1)
        self.markerCount = 0

        self.currentPoint = [0, 0] # The next points destination
        self.nextClosest() # Sets it the closest point
        self.move() # Starts moving

    # Moves in the direction of the next point repeatedly until done.
    def move(self):
        while not rospy.is_shutdown():
            # Keeps track of distance and angle from next point as time goes on
            (roll, pitch, self.theta) = euler_from_quaternion([self.ori.x, self.ori.y, self.ori.z, self.ori.w])
            currdistance = math.sqrt((self.x - self.currentPoint[0])**2 + (self.y - self.currentPoint[1])**2)

            # If we're there AND we haven't drawn the point yet, draw it and move on.
            if currdistance < 0.1 and self.image[self.currentPoint[0],self.currentPoint[1]] != True:
                self.image[self.currentPoint[0],self.currentPoint[1]] = True
                self.nextClosest()

            # If there are still points left,
            # Either rotate in the direction of the next one
            # or move towards it.
            elif self.distance != math.inf:
                self.moveCmd = Twist()
                self.angleCalculator()
                if abs(self.angle - self.theta) >= 0.05:
                    self.moveCmd.angular.z = 0.5 * self.normalizeAngle(self.angle - self.theta)
                    self.moveCmd.linear.x = 0
                else:
                    self.moveCmd.angular.z = 0
                    self.moveCmd.linear.x = 0.5
                self.pub.publish(self.moveCmd)
                if self.penEngaged:
                    self.draw()

            # If there are no longer any points left to draw
            if self.distance == math.inf:
                print("done")
                self.moveCmd.angular.z = 0
                self.moveCmd.linear.x = 0
                self.pub.publish(self.moveCmd)
                if self.penEngaged:
                    self.draw()
                break
            self.rate.sleep()


    # Tracks the position x and y, as well as the angle around the Z axis
    def position_handler(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.ori = data.pose.pose.orientation

    # Determines the next closest point
    def nextClosest(self):
        self.distance = math.inf
        newX = 0
        newY = 0
        for i in range(len(self.image)):
            for j in range(len(self.image[0])):
                if self.image[j, i] == False: # If the pixel is black
                    newDistance = math.sqrt((self.x - j)**2 + (self.y - i)**2)
                    if newDistance < self.distance:
                        self.distance = newDistance
                        newX = j
                        newY = i
        self.currentPoint = [newX, newY]
        self.angleCalculator() # Calculates the angle for the next point

    # Calculates the angle needed to turn in order to face the next point
    def angleCalculator(self):
        diffX = self.currentPoint[0] - self.x
        diffY = self.currentPoint[1] - self.y
        self.angle = math.atan2(diffY, diffX)

    # Normalizes the angle so that BotRos does not overshoot
    def normalizeAngle(self, angle):
        if(math.fabs(angle) > math.pi):
            angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
        return angle

    def draw(self):       
       newCube = Marker()
       newCube.header.frame_id = "odom"
       newCube.header.stamp = rospy.get_rostime()
       newCube.ns = "cube_namespace"
       newCube.id = self.markerCount
       self.markerCount = self.markerCount + 1

       newCube.type = 1
       newCube.action = 0
       newCube.pose = Pose()
       newCube.pose.orientation.x = 0
       newCube.pose.orientation.y = 0
       newCube.pose.orientation.z = 0
       newCube.pose.orientation.w = 1

       newCube.pose.position.x = self.x
       newCube.pose.position.y = self.y
       newCube.pose.position.z = 0

       newCube.scale.x = 0.1
       newCube.scale.y = 0.1
       newCube.scale.z = 0.1
       
       newCube.color.r = 0
       newCube.color.g = 1
       newCube.color.b = 0
       newCube.color.a = 1
       
       newCube.lifetime = rospy.Duration(0)

       # publish cube to vision
       self.vis_pub.publish(newCube)
       self.rate.sleep()

                
if __name__ == "__main__":
    imagename = input("Please enter the image file you'd like to draw. ")
    filename = "src/botros/images/" + imagename
    bot = BotRos(filename)
