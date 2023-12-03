#!/usr/bin/env python

import sys
from termios import VREPRINT
import rospy
import numpy as np
import genpy
import math as m
from math import cos, sin
from geometry_msgs.msg import Twist, Pose, Vector3, PoseWithCovarianceStamped, PoseStamped, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from tf.transformations import euler_from_quaternion

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range
from std_msgs.msg import Int32

from cubic_spline_planner import calc_spline_course


class GoalMove():
    def __init__(self):
        rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, callback = self.posecallback)
        rospy.Subscriber("move_base_simple/goal", PoseStamped, callback = self.goalcallback)
        self.Cmdpub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.point_pub = rospy.Publisher("/goal_point", Marker, queue_size=10) 

    
        self.flag = False
        self.EstopSign = False        
        self.CmdMsg = Twist()


        self.state_x = 0
        self.state_y = 0
        self.state_yaw = 0
        self.goal_x = 7
        self.goal_y = 7
        
        self.cmd_yaw = 0

        self.ns = 1
        
        
        self.weight = 0.5
        #####
        
        # parameter
        self.minangle = rospy.get_param("~minAngle", 150)
        self.maxangle = rospy.get_param("~maxAngle", 210)

        self._disatnce = rospy.get_param("~distance", 0.7)
        
        self.range = rospy.get_param("~range", 20)

        self.scan_sub = rospy.Subscriber("scan", LaserScan, self.LidarCallback)
        
        self.ScanMsg = LaserScan()
        
        self.EstopSign = False
        self.ranges = []


    def LidarCallback(self, msg):
        _len = len(msg.ranges) #1152
        _resol = 360 / _len

        self.ranges = msg.ranges
    def DetectRange(self):
        
        obstacle = 0
        for i in self.ranges[int((self.minangle)/360.0*1152.0):int((self.maxangle)/360.0*1152.0)]:            
            if 0.0 < i <= self._disatnce:
                obstacle += 1
        
        if obstacle >= 50:
            self.EstopSign = True
        else:
            self.EstopSign = False

            
    def goalcallback(self, msg):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        
    def posecallback(self, msg):
        self.state_x = msg.pose.pose.position.x
        self.state_y = msg.pose.pose.position.y
        _, _, self.state_yaw = euler_from_quaternion([0., 0., msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        
    def CreatPath(self):
        xs = [self.state_x, self.goal_x]
        ys = [self.state_y, self.goal_y]
        
        self.cx, self.cy, self.cyaw, _, _ = calc_spline_course(xs[:], ys[:], ds=0.1)
        
    def calc_side(self):
        idx = self.calc_target_index(self.cx, self.cy, [self.state_x, self.state_y])
        if idx == None:
            idx = 0
        state_vec = np.array([m.cos(self.state_yaw), m.sin(self.state_yaw),0])
        path_vec = np.array([self.goal_x - self.state_x, self.goal_y - self.state_y, 0])
        up_vec = np.array([0.,0.,1.])
        cross_vec = np.cross(state_vec, path_vec)
        # if path yaw -> right : self.d < 0
        # if path yaw -> left : self.d > 0
        self.d = np.dot(cross_vec, up_vec)
        
        
    def SetOrientation(self):
        idx = self.calc_target_index(self.cx, self.cy, [self.state_x, self.state_y])
        if idx == None:
            idx = 0
        th = self.state_yaw - self.cyaw[idx]
        state_vec = np.array([m.cos(self.state_yaw), m.sin(self.state_yaw)])
        point_vec = np.array([m.cos(self.cyaw[idx]), m.sin(self.cyaw[idx])])
        dot = np.dot(state_vec, point_vec)
        self.theta = abs(m.acos(
            (dot) / (np.hypot(state_vec[0], state_vec[1]) * np.hypot(point_vec[0], point_vec[1]))))
        print(self.d)
        
        if self.d >= 0 and self.theta >= 0.1:#path yaw : left
            self.cmd_yaw = + (self.theta * self.weight)
            
            
        elif self.d < 0 and self.theta >= 0.1: #path yaw : right
            self.cmd_yaw = - (self.theta * self.weight)

        else :
            self.cmd_yaw = 0.0
            

    def CmdPub(self):
        dis = m.sqrt((self.state_x - self.goal_x)**2 + (self.state_x - self.goal_x)**2)
        
        if dis <= 0.2:
            self.flag = False
        else :
            self.flag = True
        
        print("flag : ", self.flag)
        print("Estop : ", self.EstopSign)
        if self.flag == True:
            if self.EstopSign == False:
                self.CmdMsg.linear = Vector3(0.2, 0., 0.)
                self.CmdMsg.angular = Vector3(0., 0., self.cmd_yaw)
            else :
                self.CmdMsg.linear = Vector3(0., 0., 0.)
                self.CmdMsg.angular = Vector3(0., 0., 0.)
        else:
            self.CmdMsg.linear = Vector3(0., 0., 0.)
            self.CmdMsg.angular = Vector3(0., 0., 0.)

        self.Cmdpub.publish(self.CmdMsg)

    def calc_target_index(self, cx, cy, point):
        fx = point[0]
        fy = point[1]
        dx = [fx - icx for icx in cx]
        dy = [fy - icy for icy in cy]
        d = np.hypot(dx, dy)
        target_idx = np.argmin(d)
        return target_idx


    def publishPoint(self, position):
        
        marker = Marker()

        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()

        marker.ns = str(self.ns)
        marker.id = 1

        marker.type = 3
        marker.action = 0

        marker.pose.position = Point(position[0], position[1], 0.)
        marker.pose.orientation = Quaternion(0., 0., 0., 1)
        marker.scale = Vector3(0.2, 0.2, 0.2)
        marker.color = ColorRGBA(1.0, 0.,0.,1.)

        marker.lifetime = genpy.Duration(secs=0.2)
        self.ns += 1
        self.point_pub.publish(marker)
    
    
if __name__ == "__main__":
    
    rospy.init_node("GoalMove")

    mv = GoalMove()
    r = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        mv.DetectRange()
        mv.CreatPath()
        mv.calc_side()
        mv.SetOrientation()
        mv.CmdPub()        
        mv.publishPoint([mv.goal_x, mv.goal_y])
        r.sleep()