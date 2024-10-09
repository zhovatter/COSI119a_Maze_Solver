#!/usr/bin/env python3
import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from pid import PID

DESIRED_DIST = 0.2
WALL_THRESHOLD = 1.3 #compare this to cos ratio of perpendicular and a lidar point 45 deg behind to see if it has found a wall
class MazeSolverSim:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        self.lidar = None
        self.rightLidar = None
        self.pid = PID(min_val = -0.6, max_val = 0.6, kp = 0.5, ki = 0, kd = 0.2)
        self.speed = 0
        self.angular_vel = 0



        
    def scan_cb(self, msg):
        """Callback function for `self.scan_sub`."""
        self.lidar = self.cleanLidar(list(msg.ranges))
        self.rightLidar = self.lidar[230:310]
        distFromWall = self.dist_to_wall(self.rightLidar)
        #print(self.rightLidar)
        #print(msg.ranges[270])
        print('dist: ',distFromWall)
        #print('angular_vel: ',self.angular_vel)
        self.angular_vel = self.pid.compute(setpoint = DESIRED_DIST, measured_value = distFromWall)

        
        #raise NotImplementedError

    def follow_wall(self):
        """Makes the robot follow a wall."""
        rate = rospy.Rate(10)
        while(self.lidar == None):
            rate.sleep()
        while not rospy.is_shutdown():
            if self.detectWall(self.lidar):
                #self.move_straight()
                continue
            else:
                print("no wall!")
            rate.sleep()
        raise NotImplementedError

    def move_straight(self):
        twist = Twist()
        self.speed = 0.2
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
    
    def detectWall(self, lidar):
        ratio = lidar[270] / lidar[225]
        # print(ratio)
        # print(math.cos(math.radians(270-225)))
        return not ratio >= WALL_THRESHOLD * math.cos(math.radians(270-225))



if __name__ == '__main__':
    rospy.init_node('maze_solver_sim')
    MazeSolverSim().follow_wall()
