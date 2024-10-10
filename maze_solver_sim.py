#!/usr/bin/env python3
import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point, Pose, Twist
from tf.transformations import euler_from_quaternion

from pid import PID

DESIRED_DIST = 0.3
WALL_THRESHOLD = 0.3 #compare this to cos ratio of perpendicular and a lidar point 45 deg behind to see if it has found a wall
FORWARD_SPEED = 0.1
TURN_ANGLE_TOLERANCE = 0.01
MAX_TURN_SPEED = 0.5
MIN_TURN_SPEED = 0.01
TOLERANCE_AGAINST_ERROR = 0.0001
class MazeSolverSim:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        self.lidar = None
        self.prevLidar = None
        self.rightLidar = None
        self.frontLidar = None
        self.pid = PID(min_val = -0.6, max_val = 0.6, kp = 0.7, ki = 0.1, kd = 10)
        self.speed = 0
        self.angular_vel = 0
        self.my_odom_sub = rospy.Subscriber('my_odom', Point, self.my_odom_cb)
        # Current heading of the robot.
        self.cur_yaw = None

        #Current distance moved in a direction since last my_odom reading
        self.cur_dist = None



        
    def scan_cb(self, msg):
        """Callback function for `self.scan_sub`."""
        if self.lidar != None:
            self.prevLidar = self.lidar
        self.lidar = self.cleanLidar(list(msg.ranges))
        self.rightLidar = self.lidar[230:310]
        self.frontLidar = self.lidar[357:359]#.append(self.lidar[0:2])
        distFromWall = self.dist_to_wall(self.rightLidar)
        #print(self.rightLidar)
        #print(msg.ranges[270])
        #print('dist: ',distFromWall)
        #print('angular_vel: ',self.angular_vel)
        self.angular_vel = self.pid.compute(setpoint = DESIRED_DIST, measured_value = distFromWall)
        print('dist: ',distFromWall)


        
        #raise NotImplementedError

    def my_odom_cb(self, msg):
        """Callback function for `self.my_odom_sub`."""
        self.cur_yaw = self.convertNegativeAngles(msg.y) #Point is (Change in distance, yaw, 0)
        self.cur_dist = msg.x

    def follow_wall(self):
        """Makes the robot follow a wall."""
        rate = rospy.Rate(10)
        while(self.lidar == None or self.prevLidar == None or self.frontLidar == None):
            rate.sleep()
            print('hi')
        while not rospy.is_shutdown():
            if self.detectWall(self.lidar, self.prevLidar):
                self.move_straight()
                continue
            else:
                print("no wall!")
                #self.cmd_vel_pub.publish(Twist())
                #self.turn_to_heading(self.headingToRelative(1.5*math.pi))
            rate.sleep()
        #raise NotImplementedError

    def move_straight(self):
        twist = Twist()
        self.speed = min(0.1, 0.1 * (sum(self.frontLidar)/len(self.frontLidar)))
        twist.linear.x = self.speed
        twist.angular.z = self.angular_vel
       
        self.cmd_vel_pub.publish(twist)

    def dist_to_wall(self, rightLidar):
        min = 3.5
        for i in range(len(rightLidar)):
            if rightLidar[i] != 0 and rightLidar[i] < min:
                min = rightLidar[i]
        return min

    def cleanLidar(self, lidar):
        #print(lidar)
        for i in range(len(lidar)):
            if lidar[i] < 0.12 or lidar[i] > 3.5:
                lidar[i] = 0
        return lidar
    
    def detectWall(self, lidar, prevLidar):
        dif = lidar[270] - prevLidar[270]
        # print(lidar[270])
        # print(prevLidar[270])
        #if dif > WALL_THRESHOLD:
            #ratio = lidar[260] / lidar[215]
            # print(ratio)
            # print(math.cos(math.radians(270-225)))
            #return not ratio >= WALL_THRESHOLD * math.cos(math.radians(260-215))
        return True
    
    def turn_to_heading(self, target_yaw):
        """
        Turns the robot to heading `target_yaw`.
        """
        headingTwist = Twist()
        proportion = 0.5

        while not math.isclose(self.cur_yaw, target_yaw, abs_tol=TURN_ANGLE_TOLERANCE): #since going past pi rads turns heading value to negative
            #slows rotation as the target is approached, with a minimum and maximum rotation speed defined.
            headingTwist.angular.z = max(min(abs(proportion*(target_yaw - self.cur_yaw)) , MAX_TURN_SPEED) , MIN_TURN_SPEED) 
            self.cmd_vel_pub.publish(headingTwist)

            while self.cur_yaw == None:
                continue #waiting for new odometry values
       
        self.cmd_vel_pub.publish(Twist()) #a 0-rotation twist to stop movement

    #converts negative angles (past 1pi radians) to a positive value
    def convertNegativeAngles(self, angle):
        if angle >= 0:
            return angle
        else:
            return 2*math.pi + angle 
    
    #makes sure there are no angles over 2pi
    def normalizePosAngles(self, heading):
        return abs(heading) % (2*math.pi)

    #ensures yaw targets are relative to the robot, allowing it to move in the desired shapes from any initial orientation.
    def headingToRelative(self, target_yaw):
        return self.normalizePosAngles(self.cur_yaw + target_yaw)

    



if __name__ == '__main__':
    rospy.init_node('maze_solver_sim')
    MazeSolverSim().follow_wall()
