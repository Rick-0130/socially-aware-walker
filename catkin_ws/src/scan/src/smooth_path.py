#!/usr/bin/env python3

# -*- coding: utf-8 -*-
'''
@Time          : 20/04/25 15:49
@Author        : huguanghao
@File          : demo.py
@Noice         :
@Modificattion :
    @Author    :
    @Time      :
    @Detail    :

'''

import numpy as np
import matplotlib.pyplot as plt
import sys
import time
import copy
from CubicSpline import cubic_spline_planner
from walker_msgs.srv import pathsmoothing, pathsmoothingResponse


import rospy
from geometry_msgs.msg import Twist, PoseStamped, Quaternion, Point, PointStamped, Pose2D
from nav_msgs.msg import Path, Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from std_msgs.msg import Float32


def smooth_path(req):
    print ("Returning")
    path_x_raw = []
    path_y_raw = []   
    for i in range(len(req.path_raw.poses)-1, -1, -1):
        if(i%2==0)or(i==0)or(i==len(req.path_raw.poses)):
            path_x_raw.append(req.path_raw.poses[i].pose.position.x)
            path_y_raw.append(req.path_raw.poses[i].pose.position.y)
            #print(i)
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(path_x_raw,
                                                                    path_y_raw,
                                                                    ds=0.01)
    updated_flat_path = []
    for i in range(len(cx)):
        updated_flat_path.append(Pose2D(x=cx[i], y=cy[i], theta=cyaw[i]))
    flag_path_update = True

    # Visualiztion
    flat_path_msg = Path()
    for i in range(len(cx)):
        tmp_pose = PoseStamped()
        tmp_pose.pose.position = Point(cx[i], cy[i], 0)
        q = quaternion_from_euler(0, 0, cyaw[i])
        tmp_pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
        flat_path_msg.poses.append(tmp_pose)
    flat_path_msg.header.frame_id = req.path_raw.header.frame_id
    flat_path_msg.header.stamp = rospy.Time.now()
   


    return pathsmoothingResponse(flat_path_msg)
    
def smooth_path_server():
    rospy.init_node('smooth_path_server')
    s = rospy.Service('smooth_path_service', pathsmoothing, smooth_path)
    print ('Ready to smooth the path')
    rospy.spin()

if __name__ == '__main__':
    smooth_path_server()



    