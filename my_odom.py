#!/usr/bin/env python3

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point 
from tf.transformations import euler_from_quaternion

class MyOdom:
    def __init__(self):
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_cb)
        self.my_odom_pub = rospy.Publisher('my_odom', Point, queue_size=1)
        self.old_pose = None 
        self.dist = 0.0
        self.yaw = 0.0
                
    def odom_cb(self, msg):
        """Callback function for `odom_sub`."""
        cur_pose = msg.pose.pose
        self.update_dist(cur_pose) 
        self.update_yaw(cur_pose.orientation)
        self.publish_data()


    def update_dist(self, cur_pose): 
        """
        Helper to `odom_cb`.
        Updates `self.dist` to the distance between `self.old_pose` and
        `cur_pose`.
        """
        while cur_pose == None:
            continue #waiting for new odom data
        if self.old_pose == None: #for first odom value
            xDist = cur_pose.position.x
            yDist = cur_pose.position.y
        else:
            xDist = cur_pose.position.x - self.old_pose.position.x
            yDist = cur_pose.position.y - self.old_pose.position.y 
        self.dist = math.sqrt(xDist**2 + yDist**2) #distance calculation from change in x and y values
        self.old_pose = cur_pose 

    def update_yaw(self, cur_orientation):
        """
        Helper to `odom_cb`.
        Updates `self.yaw` to current heading of robot.
        """
        euler = euler_from_quaternion([cur_orientation.x, cur_orientation.y, cur_orientation.z, cur_orientation.w]) #converting from quaternion to euler, more usable
        self.yaw = euler[2] #euler should be [roll, pitch, yaw], so yaw is third index (2)

    def publish_data(self):
        """
        Publish `self.dist` and `self.yaw` on the `my_odom` topic.
        """
        # The `Point` object should be used simply as a data container for
        # `self.dist` and `self.yaw` so we can publish it on `my_odom`.
        newPoint = Point(self.dist, self.yaw, 0)
        self.my_odom_pub.publish(newPoint)


        
if __name__ == '__main__':
    rospy.init_node('my_odom')
    MyOdom()
    rospy.spin()
