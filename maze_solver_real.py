#!/usr/bin/env python3
import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point, Pose, Twist
from tf.transformations import euler_from_quaternion

from pid import PID

DESIRED_DIST = 0.2 #works pretty well with 0.25
#WALL_THRESHOLD = 0.2 
#DETECT_WALL = 1.2#compare this to cos ratio of perpendicular and a lidar point 45 deg behind to see if it has found a wall
BASE_SPEED = 0.1 #default speed
SAFETY_THRESHOLD = 0.15 #reduce speed to the safety speed value if distance to wall is less than this value
TURN_THRESHOLD = 0.2 # initiate a lefthand turn in place if the front lidar is too close to the wall
SAFETY_SPEED = 0.01 #low speed to reduce likelihood of collision but not get stuck.
SPEED_MULT = 2 #Used to speed up the robot, should have lower value for a maze with tighter corners.
#TURN_ANGLE_TOLERANCE = 0.01

#MAX_TURN_SPEED = 0.5
#MIN_TURN_SPEED = 0.01
class MazeSolverSim:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        self.lidar = None
       # self.prevLidar = None
        self.rightLidar = None
        self.collisionLidar = None #make a front and a left lidar for collision avoidance.
        self.upperRightLidar = None
        self.frontLidar = None
        self.pid = PID(min_val = -0.8, max_val = 0.8, kp = 1, ki = 0.05, kd = 40) #best, kp=1, ki=0.05, kd=40
        self.speed = 0
        self.angular_vel = 0
        
        
    def scan_cb(self, msg):
        """Callback function for `self.scan_sub`."""
        # if self.lidar != None:
        #     self.prevLidar = self.lidar
        self.lidar = self.cleanLidar(list(msg.ranges)) 

        self.rightLidar = self.lidar[180:359] #main lidar to detect wall

        self.upperRightLidar = self.lidar[330:359] #used to determine if extra turn is needed

        #collisionLidar is used to detect if a wall is too close in the front or lefthand side to decide if speed should be slowed
        self.collisionLidar = self.lidar[358:359]
        self.collisionLidar.extend(self.lidar[0:160])

        #front lidar used to adjust speed and initiate extra turn if necessary
        self.frontLidar = self.lidar[358:359]
        self.frontLidar.extend(self.lidar[0:1])

        distFromWall = self.dist_to_wall(self.rightLidar)
        #print(self.rightLidar)
        #print(msg.ranges[270])
        #print('dist: ',distFromWall)
        #print('angular_vel: ',self.angular_vel)

        #PID used to determine change in yaw, since distFromWall takes the minimum right side lidar value, it can track
        # the wall when doing outside corner turns, but may fail if the turn is too tight.
        self.angular_vel = self.pid.compute(setpoint = DESIRED_DIST, measured_value = distFromWall)
        #print('dist: ',distFromWall)


    def follow_wall(self):
        """Makes the robot follow a wall. Uses right-hand rule"""
        rate = rospy.Rate(10)
        #waiting for lidar data to be populated.
        while(self.lidar == None or self.collisionLidar == None): #or self.prevLidar == None
            rate.sleep()
            print('hi')
        while not rospy.is_shutdown():
            self.move_straight()
            rate.sleep()

    def move_straight(self):
        """Uses PID control to help robot follow the wall, and also allows for outside corner turns."""
        rate = rospy.Rate(5)
        twist = Twist()

        collisionDist = self.dist_to_wall(self.collisionLidar)#self.collisionLidar)
        frontDist = self.dist_to_wall(self.frontLidar)
        upperRightDist = self.dist_to_wall(self.upperRightLidar)

        #speed is a function of distance to the wall of the left and front sides of the robot
        self.speed = SPEED_MULT*min(BASE_SPEED, BASE_SPEED * collisionDist,BASE_SPEED * frontDist)# self.dist_to_wall(self.collisionLidar)self.collisionLidar))
        if collisionDist < SAFETY_THRESHOLD: self.speed = SAFETY_SPEED #avoiding collisions on the left
        elif frontDist < TURN_THRESHOLD or upperRightDist < TURN_THRESHOLD: self.turn_in_place() #avoid collisions in the front and allows for inside corner turning

        twist.linear.x = self.speed
        twist.angular.z = self.angular_vel
       
        self.cmd_vel_pub.publish(twist)
    
    def turn_in_place(self):
        """Turns the robot to the left in place ~45deg"""
        rate = rospy.Rate(1)
        self.cmd_vel_pub.publish(Twist())
        twist = Twist()
        twist.angular.z = 0.6
        #does two quick turns, and the method will be called again if the front lidar is still too close to the wall.
        for i in range(2):
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
            

    def dist_to_wall(self, lidar):
        """Calculates distance to the wall by finding the minimum lidar value in the range"""
        min = 3.5 #3.5 is the max lidar value
        for i in range(len(lidar)):
            if lidar[i] != 0 and lidar[i] < min:
                min = lidar[i]
        if min == 3.5:
            #if the lidar values are all too close or too far, returning
            # DESIRED_DIST makes the PID controller drive straight so it doesn't turn out of control.
            return DESIRED_DIST 
        return min
    

    def cleanLidar(self, lidar):
        """Cleans lidar values that are outside of valid range"""
        for i in range(len(lidar)):
            if lidar[i] < 0.12 or lidar[i] > 3.5:
                lidar[i] = 0
        return lidar
    
    def avg(self, array):
        """returns average value of an array"""
        return sum(array)/len(array)

if __name__ == '__main__':
    """main method"""
    rospy.init_node('maze_solver_sim')
    MazeSolverSim().follow_wall()
