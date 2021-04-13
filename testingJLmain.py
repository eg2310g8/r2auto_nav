#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jan 20 20:15:37 2021

@author: facestomperx
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math
import cmath
import time
import cv2
import random as rng
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from aStarPlanner import astar

import matplotlib.pyplot as plt
from PIL import Image

import scipy.stats





from GlobalPlan import GlobalPlan
from costmap2d import costmap2d
import utils
from getFrontierAndCentroids import getfrontier, binMap, widenBin 
import dwa_planner
from RobotMovement import OdomRobotMovement, TfRobotMovement

import copy

#from getFilter import getfrontier 

# constants
rotatechange = 0.1
speedchange = 0.05

occ_bins = [-1, 0, 50,100]
stop_distance = 0.25
front_angle = 30
front_angles = range(-front_angle,front_angle+1,1)
scanfile = 'lidar.txt'
mapfile = 'map.txt'

#bot footprint in terms of cells
footprint_spec = {
        "width":4,
        "length": 5,
    }

OCCUPIED = 1

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


class bokuNoNav(Node):
    def __init__(self):
        super().__init__('boku_no_nav')
        self.cmd_publisher_ = self.create_publisher(Twist,'cmd_vel',12)  #original 10
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        self.odom_subscription  # prevent unused variable warning
        # initialize variables
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.occ_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            qos_profile_sensor_data)
        self.occ_subscription  # prevent unused variable warning
        self.occdata = np.array([])
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning
        self.laser_range = np.array([])
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        
        self.tfTimeInterval = 0
        self.dwa_planner = dwa_planner.DWAPlanner()
        self.OdomRobotMovement = OdomRobotMovement()
        self.TfRobotMovement = TfRobotMovement()
        self.currentTfFrame = None
        self.currentOdom = None
        self.finalCmd = None
        
        
        self.RobotState = ["Mapping","Evaluating"]
        """might make this timers into another class"""
        self.localPlannerTime = 0
        self.windowed_plan = None
        
        #recovery
        self.isRobotRecovering = False
        self.lidarDistance = None
        self.forwardAngles = [-20,20]
        self.backwardAngles = [160,200]
        
        
        #self.tfTimeInterval = 0
        #trans = None
        #self.subscription  # prevent unused variable warning
        #self.tfBuffer = tf2_ros.Buffer()
        #self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        
    #callbacks
        
    def odom_callback(self, msg):
        #self.get_logger().info('In odom_callback')
        #print(msg)
        
        if (self.OdomRobotMovement.hasReference):
            #print(self.OdomRobotMovement.getVelocity())
            timeDiff = self.OdomRobotMovement.getTimeDiff(msg)
            if timeDiff != 0:
                #print("A future ODOM")
                self.tfTimeInterval = (self.tfTimeInterval + timeDiff) / 2
            #print("Average time: ", self.tfTimeInterval)
        
            
        
        
        self.OdomRobotMovement.updateFromOdomMsg(msg)
        
        #odom_position = msg.pose.pose.position
        #orientation_quat =  msg.pose.pose.orientation
        #self.get_logger().info('Odom Trans: %f, %f' % (odom_position .x, odom_position .y))
        #self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)
        #cell = utils.worldToMap(coords, world_origin, resolution)
        
        # got this hacky way
        try:
            trans = self.tfBuffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            #tf_vel = self.tfBuffer.lookupTwistFull()
            #https://answers.ros.org/question/256354/does-tftransformlistenerlookuptransform-return-quaternion-position-or-translation-and-rotation/
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().info('No transformation found')
            return
        #print(trans)
        self.currentTfFrame = trans
        return
        #self.TfRobotMovement.updateFrame(trans)
        cur_pos = trans.transform.translation
        cur_rot = trans.transform.rotation
        self.get_logger().info('Trans: %f, %f' % (cur_pos.x, cur_pos.y))
        # convert quaternion to Euler angles
        roll, pitch, yaw = euler_from_quaternion(cur_rot.x, cur_rot.y, cur_rot.z, cur_rot.w)
        self.get_logger().info('Rot-Yaw: Rad: %f Deg: %f' % (yaw, np.degrees(yaw)))
        
        
        
        return #everything below this line was an experiment using the Tf frames to detect speed, turns out odom might be better
        
        
        if (self.currentTfFrame == None):
            print("Init self.currentTfFrame")
            self.currentTfFrame = trans
            return
        #print(self.currentTfFrame)
        #print("Getting time lmao")
        #t2 = self.RobotMovement.getTimeFromFrame(self.currentTfFrame)
        #print("T2:", t2)
       # t1 = self.RobotMovement.getTimeFromFrame(trans)
        #print("t1: ", t1)
        self.currentTfFrame = trans
        cur_pos = trans.transform.translation
        cur_rot = trans.transform.rotation
        self.get_logger().info('Trans: %f, %f' % (cur_pos.x, cur_pos.y))
        # convert quaternion to Euler angles
        roll, pitch, yaw = euler_from_quaternion(cur_rot.x, cur_rot.y, cur_rot.z, cur_rot.w)
        self.get_logger().info('Rot-Yaw: Rad: %f Deg: %f' % (yaw, np.degrees(yaw)))
        #trans_copy = copy.deepcopy(trans)
        self.robot_vel = self.RobotMovement.myLookupTwist(trans)
        print("Robot twist: ", self.robot_vel)
        #print(self.RobotMovement.frameTimeDiff(trans_copy))
        if self.RobotMovement.isFutureFrame(trans):
            print("A future frame")
            self.tfTimeInterval = (self.tfTimeInterval + self.RobotMovement.frameTimeDiff(trans)) / 2
        print("Average time: ", self.tfTimeInterval)
        self.RobotMovement.updateFrame(trans)
        self.global_plan = None

    def occ_callback(self, msg):
        # find transform to obtain base_link coordinates in the map frame
        # lookup_transform(target_frame, source_frame, time)
        if(self.currentTfFrame == None):
            print("currentTfFrame still None")
            return 
        #binning grid
        grid_edges = binMap(msg)
        astar_grid_edges = widenBin(grid_edges,3) #for A star to not hug so tightly 
        costmap = costmap2d(msg, astar_grid_edges, footprint_spec) #grid_edges, fatter_edges
        self.currentCostmap = costmap
        self.dwa_planner.updateCostmap(costmap)
        
        if (not self.RobotState == ["Mapping","Evaluating"]):
            return
        #if (len(self.RobotState) == 2 and self.RobotState[0] == "Mapping" and self.RobotState[1] == "Travelling" or self.RobotState[1] == "LocalPlan"):
        #    return
        
        
        
        #we dont want the bot flying around while planning
        self.stopbot()
        time.sleep(0.5) # If a star computes fast rnough and got no chance to stop
        
        
        
        
        cur_pos = self.currentTfFrame.transform.translation
        cur_rot = self.currentTfFrame.transform.rotation
        roll, pitch, yaw = euler_from_quaternion(cur_rot.x, cur_rot.y, cur_rot.z, cur_rot.w)
        
        robot_pose = utils.createRobotPose(cur_pos,cur_rot)
        
        
        
        
        #trying to pass a fattened grid
        frontierImg, frontierCells, frontierWorldPts,mBotPos, grid_edges, fatter_edges = getfrontier(msg,cur_pos,grid_edges)
       # print("FRONTIERS")
       # print(frontierCells)
     #   print(len(frontierCells))
        
     #   print("Bot footprint: ", costmap.getMapBaseFootprint())   #should update my map
        aStarConfig = {
            "occupied": 1,
            "vacant": 0,
            "footprint":costmap.getMapBaseFootprint(),
            "footprintRadius": costmap.robotRadiusInCells 
        }
        
        binary_grid_edges = []
        for row in fatter_edges: #grid_edges, fatter_edges
            temp = []         
            for col in row:
                if col == 255:
                    temp.append(1)
                else:
                    temp.append(0)
            binary_grid_edges.append(temp)
        
        """
        print(frontierCells)
        for row in binary_grid_edges:
            print(row)
        """
        
        occdata = np.array(msg.data)
        #print(occdata.shape)
        #checkOcc = np.uint8(occdata.reshape(msg.info.height,msg.info.width))
        occ_counts, edges, binnum = scipy.stats.binned_statistic(occdata, np.nan, statistic='count', bins=occ_bins)
       # iwidth = msg.info.width
        #iheight = msg.info.height
        #total_bins = iwidth * iheight
        #odata = np.uint8(binnum.reshape(msg.info.height,msg.info.width))
        
        img = Image.fromarray(frontierImg) #grid_edges, frontierImg, fatter_edges
        # show the image using grayscale map
        plt.imshow(img, cmap='gray', origin='lower')
        plt.draw_all()
        plt.pause(0.00000000001)
        #print(frontierCells)
        #return
        
        if (len(frontierCells) == 0): #(frontierCells[0][0] == 0 and frontierCells[0][1] == 0) ):
            self.evaluateCompletion()
            return
        
        
        shortestPath = None
        shortestEnd = None
        
        
        """
        shortestPathIdx = None
        shortestDistance = float('inf')
        print(frontierWorldPts)
        
        for idx,frontier in enumerate(frontierWorldPts):
            dist = utils.getRoughEuclideanDistance(cur_pos, frontier)
            if dist < shortestDistance:
                shortestDistance = dist
                shortestPathIdx = idx
          """  
        
        
        
        
        #try to just do a star on this?
        for frontiers in frontierCells:
            startPos = mBotPos
            endPos = tuple(frontiers)
            #print("Cost: ",costmap.getCost(endPos[0], endPos[1]))
            """
            if (not costmap.isCellCoordValid(endPos) or costmap.getCost(endPos[0], endPos[1]) > 0):
                print("Sadly invalid path? Maybe can find somewhere nearer")
                continue
            """
            """
            endPose = utils.mapToWorld2(endPos, costmap.getMapOrigin(), costmap.getMapResolution())
            inc_x = endPose[0] - cur_pos.x
            inc_y = endPose[1] - cur_pos.y
            angleBetween = math.atan2(inc_y, inc_x) - yaw # this works
            print("Bot yaw: ", np.degrees(yaw))
            print("angle between points??"  , angleBetween, np.degrees(angleBetween))
            """
            #print("Forming path from ", startPos, endPos)    
            #print(costmap.getGridShape())
            path = astar(binary_grid_edges, startPos, endPos, aStarConfig)
            if(path == "invalid start"):
                shortestPath = "invalid start"
                break
            if (path == None):
                #print("A star invalid path")
                continue
            #print(mBotPos)
            #print(path)
            #print("Path length: ", len(path))
            if shortestPath == None or len(path) < len(shortestPath):
                shortestPath = path
                shortestEnd = endPos
        
        
        """
        startPos = mBotPos
        endPos = tuple(frontierCells[shortestPathIdx])
        path = astar(binary_grid_edges, startPos, endPos, aStarConfig)
        
        else:
            shortestPath = path
        
        
        if(shortestPath == None):
            print("Maybe all the A star stuff fked, so oops")
            self.recover()
            return
            #self.evaluateCompletion()
        """
        if(shortestPath == "invalid start" or shortestPath == None):
            print("A star invalid path")
            self.recover()
            return
            
        print("Shortest path is: ", shortestPath[0], shortestPath[len(shortestPath) - 1], len(shortestPath))
        print(shortestEnd)
        print(shortestPath)
        poses = utils.convertPlanToPoses(shortestPath, costmap.getMapOrigin(), costmap.getMapResolution())
        #print("Poses length: ", len(poses))
        global_plan = GlobalPlan(poses)
        self.global_plan = global_plan
        goal = poses[len(poses) - 1].pose.position
        nextPoint = poses[1].pose.position
        
        
        #endPose = utils.mapToWorld2(endPos, costmap.getMapOrigin(), costmap.getMapResolution())
        inc_x = nextPoint.x - cur_pos.x
        inc_y = nextPoint.y - cur_pos.y
        angleBetween = math.atan2(inc_y, inc_x) - yaw # this works
        print("Bot yaw: ", np.degrees(yaw))
        print("angle between points??"  , angleBetween, np.degrees(angleBetween))
        #return
        
        self.RobotState = ["Mapping","LocalPlan"]
        """
        img2 = Image.fromarray(np.array(frontierImg,dtype="uint8"))
        # show the image using grayscale map
        plt.imshow(img2, cmap='gray', origin='lower')
        
        
        plt.draw_all()
        plt.pause(0.00000000001)
        
        """

    def scan_callback(self, msg):
        # self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # print to file
        #np.savetxt(scanfile, self.laser_range)
        # replace 0's with nan
        self.laser_range[self.laser_range==0] = np.nan
        #print(self.laser_range)
        #print("Updating lidarRange")
        self.dwa_planner.lidarRange = self.laser_range
        
        if(self.currentTfFrame == None):
            print("currentTfFrame still None")
            return 
        
        
        

    """
     * Get consecutive unknown cells that are of turtlebot dimmension
     * @param[in] map 2d array representing map
     * @param[out] list of indexes in map that are unknown contours
    """
    def get_unknown_contours(self,map):
        print("get_unknown_contours")
        start_pos,map_edge = self.find_overall_edge(map)
       # print(start_pos)
        #print(map_edge.shape())
        #print(map_edge)
        map_edge_array = np.asarray(map_edge)
        return map

    """"
     * Either shrink and dilate (opencv), so called edge detection,
     * or for now just scan thru and find values other than unknown
     *
     *
    """
    def find_overall_edge(self,map):
        #map_edge = []
        start_pos = [0,0]
        for i,row in enumerate(map):
            #current_row = []
            #print(row)
            for j,col in enumerate(row):
                if col == 1 or col == 2:
                    #current_row.append(UNKNOWN_CELL)
                    map[i,j] = 1
                elif col == 3:
                    #current_row.append(KNOWN_CELL)
                    map[i,j] = 2
                    start_pos = [i,j]
                    
        return [start_pos,map]
    
    def evaluateCompletion(self):
        print("Yay we're done??")
        self.RobotState = ["Mapping","Complete"]
    def stopbot(self):
        self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # time.sleep(1)
        self.cmd_publisher_.publish(twist)
    
    def getRobotPose(self):
        cur_pos = self.currentTfFrame.transform.translation
        cur_rot = self.currentTfFrame.transform.rotation 
        return utils.createRobotPose(cur_pos,cur_rot)
    
    def executeLocalPlanning(self, currentTime):
        #print("Local plan now")
        #return
        self.localPlannerTime = currentTime
        robot_vel = self.OdomRobotMovement.getVelocity()
        cur_pos = self.currentTfFrame.transform.translation
        cur_rot = self.currentTfFrame.transform.rotation
        robot_pose = utils.createRobotPose(cur_pos,cur_rot)
        costmap = self.currentCostmap
        global_plan = self.global_plan
        
        #what i suspect is happening here is, the path havent done yet, then i clear everything
        windowed_plan = self.dwa_planner.transformGlobalPlan(robot_pose, global_plan, costmap, self.windowed_plan)
        
        
        if (len(windowed_plan.getAllPoses()) < 2):
            self.windowed_plan = None
            self.RobotState = ["Mapping","Evaluating"]
            return
        self.windowed_plan =  windowed_plan
        #print("ROBOT VELOCITY")
        #print(robot_vel)
        bestTraj, bestCmd = self.dwa_planner.findBestPath(robot_pose, robot_vel, windowed_plan)
        #print("BEST PATH")
        if (bestTraj == None):
            print("bestTraj is none, so just rotate and hope for the best? Alternatively, activate a recovery plan that moves away from closest point")
            print("Maybe backtrack and recompute")
            self.stopbot()
            self.recover()
        #else:
        #    print(vel_x, vel_y, np.degrees(vel_th)
            #vel_x, vel_y, vel_th = bestTraj.getVelocities()
        else:
            #print("Supposed goal to verify velocity makes sense")
                #print(windowed_plan.getFinalGoalPose())
            #print(utils.worldToMap(windowed_plan.getFinalGoalPose(), costmap.getMapOrigin(), costmap.getMapResolution()))
                #print(global_plan.getFinalGoalPose())
            #print(utils.worldToMap(global_plan.getFinalGoalPose(), costmap.getMapOrigin(), costmap.getMapResolution()))
                #print(bestPath)
            self.finalCmd = self.dwa_planner.convertTrajectoryToTwist(bestCmd)
            #self.dwa_planner.trajStackPush(self.finalCmd)
            
            
        #print(finalCmd)
        #print("Entire plan vs windowed length: ", len(self.global_plan.getAllPoses()), len(windowed_plan.getAllPoses()))
        #print(startPos,endPos)
        self.cmd_publisher_.publish(self.finalCmd)
        #self.RobotMovement.updateFrame(trans)
        self.RobotState = ["Mapping","Travelling"]
    
    def recover(self):
        #if (not self.isRobotRecovering):
        #if(self.laser_range == None):
        #    return
            
        self.isRobotRecovering = True
        #print(self.laser_range)
        
        bestCmd = self.dwa_planner.recover(self.laser_range, self.getRobotPose(), self.cmd_publisher_)
        #if (bestCmd == "isRecovered"):
        #    self.isRobotRecovering = False
        #    return
        print("Recovery pose")
        print(bestCmd)
        print("Current cmd")
        print(self.finalCmd)
        self.finalCmd = self.dwa_planner.convertTrajectoryToTwist(bestCmd)
        
        
        print("Recovery cmd: ")
        print(self.finalCmd)
        self.cmd_publisher_.publish(self.finalCmd)
    def mover(self):
        try:
            # initialize variable to write elapsed time to file
            # contourCheck = 1

            # find direction with the largest distance from the Lidar,
            # rotate to that direction, and start moving
            #self.pick_direction()

            while rclpy.ok():
                
                if (self.RobotState[0] == "Mapping" and (self.RobotState[1] == "Complete")):
                    break
                if self.laser_range.size != 0:
                    # check distances in front of TurtleBot and find values less
                    # than stop_distance
                    #lri = (self.laser_range[front_angles]<float(stop_distance)).nonzero()
                    shortestDistIdx = np.nanargmin(self.laser_range)
                    if (self.laser_range[shortestDistIdx] < 0.12):
                        
                        if((shortestDistIdx < 20 and shortestDistIdx > -20) or (shortestDistIdx > 160 and shortestDistIdx < 180) ):
                            print("Emergency! Front and back too near")
                            self.stopbot()
                            self.recover()
                        else:
                            self.isRobotRecovering = False
                    else:
                        self.isRobotRecovering = False
                    
                    #self.lidarDistances = lri
                    #print(self.laser_range)
                    #self.get_logger().info('Distances: %s' % str(lri))
                    #if (len(lri) > 0):
                    #    return
                    # if the list is not empty
                    """
                    if(len(lri[0])>0):
                        # stop moving
                        self.stopbot()
                        # find direction with the largest distance from the Lidar
                        # rotate to that direction
                        # start moving
                        self.pick_direction()
                    """
                
                               
                
                currentTime = utils.current_milli_time()
                #delayThreshold = 50
                
                
                if (len(self.RobotState) == 2 and self.RobotState[0] == "Mapping" and (self.RobotState[1] == "LocalPlan" or self.RobotState[1] == "Travelling") ):
                    #print("Mapping local plan")
                    #print("currentTime: ", currentTime)
                    cur_pos = self.currentTfFrame.transform.translation
                    cur_goal_pose = self.global_plan.getFinalGoalPose()
                    #print("Checking if reached goal")
                    if (self.dwa_planner.isGoalReached(cur_pos,None,cur_goal_pose)):
                        print("Goal has REACHED")
                        self.windowed_plan = None
                        self.RobotState = ["Mapping","Evaluating"]
                        continue
                    if (currentTime - self.localPlannerTime >=( 100 )): #125, 100, 75 can achive, 50 dies
                        #print("Interval: ", currentTime - self.localPlannerTime)
                        self.executeLocalPlanning(currentTime)
                        self.localPlannerTime = currentTime
                    #return
                
                # allow the callback functions to run
                rclpy.spin_once(self)

        except Exception as e:
            print(e)
        
        # Ctrl-c detected
        finally:
            # stop moving
            self.stopbot()




def main(args=None):
    rclpy.init(args=args)

    auto_nav = bokuNoNav()
    auto_nav.mover()

    # create matplotlib figure
    # plt.ion()
    # plt.show()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    auto_nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()