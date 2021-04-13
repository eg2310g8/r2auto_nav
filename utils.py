#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Apr  3 20:14:57 2021

@author: facestomperx
"""
import math
import numpy as np
from geometry_msgs.msg import PoseStamped,Point,Pose, Twist
from types import SimpleNamespace
#https://www.youtube.com/watch?v=J1F32aVSYaU

import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
#import tf_conversions.transformations

import time

import rclpy

"""
u
"""
"""
class Vector3f():
    def __init___(self, *args, ***kwargs):
        
"""

M_PI_2 = math.pi / 2

def hypotenuseLen(a, b):
    return math.sqrt(a**2 + b**2)

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians

#https://stackoverflow.com/questions/53033620/how-to-convert-euler-angles-to-quaternions-and-get-the-same-euler-angles-back-fr
def quaternion_from_euler(roll,pitch,yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]
        

def getYaw(quat):
    roll, pitch, yaw = euler_from_quaternion(quat.x, quat.y, quat.z, quat.w)
    return yaw

def getRoughEuclideanDistance(pt1,pt2):
    dist = (pt1.x - pt2.x) ** 2 + (pt1.y - pt2.y) ** 2
    #print(dist)
    return dist

def getEuclideanDistance(p1,p2):
    return math.sqrt(getRoughEuclideanDistance(p1, p2))

def getEuDist4Cells(c1,c2):
    return math.sqrt((c1[0] - c2[0]) ** 2 + (c1[1] - c2[1]) ** 2)

#returns manhatten distance between two points
def getManhattenDistance(p1,p2):
    return abs(p1.x - p2.x) + abs(p1.y - p2.y)

def current_milli_time():
    return round(time.time() * 1000)

def ros_time():
    return rclpy.time.Time().to_msg()


def mapToWorld(coords,world_origin, resolution):
    #convert_offset = 0.5
    wx = world_origin.x + coords.x * resolution
    wy = world_origin.y + coords.y * resolution
    return (wx,wy)

"""
Takes a row-col cell and returns a converted tuple (x,y)
"""
def mapToWorld2(coords,world_origin, resolution):
    #convert_offset = 0.5
    wx = world_origin.x + (coords[1] * resolution)
    wy = world_origin.y + (coords[0] * resolution)
    return (wx,wy)

def worldToMap(coords, world_origin, resolution):
    #print("World to map")
    #convert_offset = 0.5
    if (coords.x < world_origin.x or coords.y < world_origin.y):
        print("Does this imply smth")
    mx = round((coords.x - world_origin.x) / resolution)
    my = round((coords.y - world_origin.y) / resolution)
    return (my,mx)

"""
Plan is a sequence of cell coords. Returns a list of poseStamped
"""

def convertPlanToPoses(plan, world_origin, resolution):
    plan_time = ros_time()
    #print(world_origin)
   # print(resolution)
    #print(plan_time)
    posePlan = []
    for cell in plan:
        #https://docs.python.org/3/library/types.html#types.SimpleNamespace for dot notation
        #coords = SimpleNamespace()
        #coords.x = cell[0]
        #coords.y = cell[1]
        #print(coords.x)
        #print(coords)
        worldCoords = mapToWorld2(cell, world_origin, resolution)
        pose = PoseStamped()
        #print(pose)
        #https://answers.ros.org/question/354203/timestamp-message-ros2-python/
        #https://github.com/ros2/rclpy/blob/196669e539dd51bc56f9f4fdf6f0222d99480d0d/rclpy/rclpy/time.py#L138-L140
        pose.header.stamp = plan_time
        pose.header.frame_id = "map" #https://answers.ros.org/question/34684/header-frame_id/
        pose.pose.position.x = worldCoords[0]
        pose.pose.position.y = worldCoords[1]
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        posePlan.append(pose)
    return posePlan

def createRobotPose(cur_pos, cur_rot):
    #plan_time = ros_time().to_msg()
    pose = Pose()
    #pose.header.stamp = plan_time
    #pose.header.frame_id = "no idea" #https://answers.ros.org/question/34684/header-frame_id/
    pose.position.x = cur_pos.x
    pose.position.y = cur_pos.y
    pose.position.z = cur_pos.z
    pose.orientation.x = cur_rot.x
    pose.orientation.y = cur_rot.y
    pose.orientation.z = cur_rot.z
    pose.orientation.w = cur_rot.w
    return pose
    
def rotateOnTheSpot():
    drive_cmds = Pose()
    drive_cmds.position.x = 0.0
    drive_cmds.position.y = 0.0
    drive_cmds.orientation.w = 1.0
    drive_cmds.orientation.x = 0.0
    drive_cmds.orientation.y = 0.0
    drive_cmds.orientation.z = 1.0
    #cmd_vel = Twist()
    #cmd_vel.linear.x = drive_cmds.position.x
    #cmd_vel.linear.y = drive_cmds.position.y
    #cmd_vel.angular.z = getYaw(drive_cmds.orientation)
    return drive_cmds
    
def reverse():
    drive_cmds = Pose()
    drive_cmds.position.x = -0.22
    drive_cmds.position.y = 0.0
    drive_cmds.orientation.w = 0.0
    drive_cmds.orientation.x = 0.0
    drive_cmds.orientation.y = 0.0
    drive_cmds.orientation.z = 0.0
    
    
    """
    cmd_vel = Twist()
    cmd_vel.linear.x = -0.22
    cmd_vel.linear.y = 0.0
    cmd_vel.angular.z = 0.0
    """
    return drive_cmds
def forward():
    drive_cmds = Pose()
    drive_cmds.position.x = 0.22
    drive_cmds.position.y = 0.0
    drive_cmds.orientation.w = 0.0
    drive_cmds.orientation.x = 0.0
    drive_cmds.orientation.y = 0.0
    drive_cmds.orientation.z = 0.0
    """
    cmd_vel = Twist()
    cmd_vel.linear.x = 0.22
    cmd_vel.linear.y = 0.0
    cmd_vel.angular.z = 0.0
    """
    return drive_cmds
        
def cellXYtoROWCOL(coord):
    return (coord[1],coord[0])


def areCellsEqual(c1,c2):
    #print("Are equal? ",c1,c2)
    if(c1[0] == c2[0] and c1[1] == c2[1]): 
        return True
    return False

def main(args=None):
    print(hypotenuseLen(3,4))
    print(quaternion_from_euler(1.5707, 0, -1.5707))

if __name__ == '__main__':
    main()