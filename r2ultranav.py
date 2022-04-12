# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from asyncio.format_helpers import _format_callback_source
from shutil import move
from this import d
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float64MultiArray, String, Bool, Float32MultiArray,Float32, Int8
import numpy as np
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import cv2
import math
import cmath
import time
import matplotlib.pyplot as plt
from PIL import Image 
from scipy.interpolate import griddata

# constants
rotatechange = 0.65
back_angles = range(150, 210 + 1, 1)

scanfile = 'lidar.txt'
mapfile = 'map.txt'
myoccdata = np.array([])
occ_bins = [-1, 0, 100, 101]
map_bg_color = 1


# code from https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/


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

    return roll_x, pitch_y, yaw_z  # in radians

def map_value(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def thermal_viz(msg):
    MINTEMP = 25.0
    MAXTEMP = 32.0
    for r in range(len(msg)):
        for p in range(len(msg[0])):
            msg[r][p] = map_value(msg[r][p], MINTEMP, MAXTEMP, 0, 255)

    #thermal_array = np.reshape(msg.data, (msg.layout.dim[0].size, msg.layout.dim[1].size))
    img = Image.fromarray(msg)
    plt.imshow(img, cmap='gray', origin='lower')
    plt.draw_all()
    plt.pause(0.00000000001)

class AutoNav(Node):

    def __init__(self):
        super().__init__('auto_nav')

        # create publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create a subscriber
        # This node subscribes to messages of type Float64MultiArray
        # over a topic named: /en613/state_est
        # The message represents the current estimated state:
        #   [x, y, yaw]
        # The callback function is called as soon as a message
        # is received.
        # The maximum number of queued messages is 10.

        # create subscription to track orientation
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        # self.get_logger().info('Created subscriber')
        self.odom_subscription  # prevent unused variable warning
        # initialize variables
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.locx = 0
        self.locy = 0
        self.movelist = []

        # create subscription to track occupancy
        self.occ_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            qos_profile_sensor_data)
        self.occ_subscription  # prevent unused variable warning
        self.occdata = np.array([])

        # create subscription to track lidar
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning
        self.laser_range = np.array([])
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.shoot_publisher = self.create_publisher(Int8, 'shoot', 10)

        self.nfc_subscription = self.create_subscription(
            Bool, 
            'nfc',
            self.nfc_callback,
            10
        )
        self.nfc_subscription

        self.thermal_subscription = self.create_subscription(
            Float32MultiArray,
            'thermal',
            self.thermal_callback,
            10
        )
        self.thermal_subscription

        self.ldr_subscription = self.create_subscription(
            Float32,
            'ldr',
            self.ldr_callback,
            10
        )

        self.target_subscription = self.create_subscription(
            Float32MultiArray,
            'target',
            self.target_callback,
            10
        )
        
        self.isTargetDetected = False
        self.isDoneShooting = False
        self.nfcDetected = False
        self.loaded = False

        # To change before starting test        

        # TODO upate
        self.d = 0.35
        self.forward_speed = 0.15
        self.k_diff = 300
        self.k_theta = 0.02
        self.follow = "Left" #"Left", "Right" CHECK CAPITALISATION

        self.thermal_points = [(math.floor(ix / 8), (ix % 8)) for ix in range(0, 64)]
        self.thermal_grid_x, self.thermal_grid_y = np.mgrid[0:7:32j, 0:7:32j]
        self.oldval = {"lf" : 0.3, "f": 0.5, "rf":0.3, "l": 0.21 ,"r": 0.21}
        self.front_dist = 0
        self.leftfront_dist = 0
        self.rightfront_dist = 0
        self.left_dist = 0
        self.right_dist = 0

        self.shot = False
        self.turn_left = False
        self.turn_right = False
        self.forward = False

    def target_callback(self, msg):

        if not self.movelist:
            for i in range(0,len(msg.data),2):
                print("MSG: ",msg.data[i],msg.data[i+1])
                if msg.data[i] < 0.0015:
                    self.movelist.append([0,msg.data[i+1]])
                elif msg.data[i] < 0.0025:
                    temp = [msg.data[i+1],0]
                    self.movelist.append(temp)
                #self.movelist.append(temp)
        print("Move List: ",self.movelist)


    def ldr_callback(self,msg):
        self.ldrval = msg.data
        self.get_logger().info("LDR Value: %s"%self.ldrval)
        if self.ldrval > 0.77:
            self.loaded = True

    def nfc_callback(self, msg):
        self.nfc = msg.data
        if self.nfc:
            self.get_logger().info('NFC tag found')
            self.nfcDetected = True

    def thermal_callback(self, msg):
        self.thermal_array = griddata(self.thermal_points, msg.data, (self.thermal_grid_x, self.thermal_grid_y), method="cubic") 
        self.thermal_array = np.reshape(self.thermal_array, (32, 32))
        #self.get_logger().info('Reading Thermal Camera')
        #thermal_viz(self.thermal_array)
        # if 15 percent of grid is heated
        #if np.count_nonzero(self.thermal_array > 30) > 160 and self.loaded:
        #    self.isTargetDetected = True
        #    self.get_logger().info("Heated Target Found")
        midpoint = [0, 0]
        heat_points = 0
        for row in range(len(self.thermal_array)):
            for col in range(len(self.thermal_array[0])):
                if self.thermal_array[row][col] > 33:
                    midpoint[0] += row
                    midpoint[1] += col
                    heat_points += 1
        if heat_points > 0:
            self.isTargetDetected = True
            midpoint[0] /= heat_points
            midpoint[1] /= heat_points
            if midpoint[1] > 16.5:
                self.turn_left = True
                self.turn_right = False
                self.forward = False
                self.get_logger().info("turn left")
            elif midpoint[1] < 15.5:
                self.turn_left = False
                self.turn_right = True
                self.forward = False
                self.get_logger().info("turn right")
            else:
                self.turn_left = False
                self.turn_right = False
                self.forward = True
                self.get_logger().info("forward")
        else:
            self.turn_right = False
            self.turn_left = False
            self.forward = False
            self.isTargetDetected = False
       
    def odom_callback(self, msg):
        self.get_logger().info('In odom_callback')
        orientation_quat = msg.pose.pose.orientation
        position = msg.pose.pose.position
        self.locx, self.locy = position.x, position.y
        self.roll, self.pitch, self.yaw = euler_from_quaternion(
            orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)

    def occ_callback(self, msg):
        global myoccdata
        self.get_logger().info('In occ_callback')
        # create numpy array
        msgdata = np.array(msg.data)
        # compute histogram to identify percent of bins with -1
        # occ_counts = np.histogram(msgdata,occ_bins)
        # calculate total number of bins
        # total_bins = msg.info.width * msg.info.height
        # log the info
        # self.get_logger().info('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i' % (occ_counts[0][0], occ_counts[0][1], occ_counts[0][2], total_bins))

        # make msgdata go from 0 instead of -1, reshape into 2D
        oc2 = msgdata + 1
        # reshape to 2D array using column order
        # self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width,order='F'))
        try:
            trans = self.tfBuffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().info('No transformation found')
            return
        self.occdata = np.uint8(oc2.reshape(msg.info.height, msg.info.width))
        myoccdata = np.uint8(oc2.reshape(msg.info.height, msg.info.width))
        odata = myoccdata
        np.savetxt(mapfile, self.occdata)

    def scan_callback(self, msg):
        # self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # print to file
        np.savetxt(scanfile, self.laser_range)

        # replace 0's with nan
        self.laser_range[self.laser_range == 0] = np.nan
        if self.front_dist == np.nan or self.front_dist == 100:
            self.front_dist = self.oldval["f"]
        if self.leftfront_dist == np.nan or self.leftfront_dist == 100:
            self.leftfront_dist = self.oldval["lf"]          
        if self.rightfront_dist == np.nan or self.rightfront_dist == 100:
            self.rightfront_dist = self.oldval["rf"]
        if self.left_dist == np.nan or self.left_dist == 100:
            self.left_dist = self.oldval["l"]           
        if self.right_dist == np.nan or self.right_dist == 100:
            self.right_dist = self.oldval["r"]




    # function to rotate the TurtleBot
    def rotatebot(self, rot_angle, x=0.0, angular=rotatechange):
       # self.get_logger().info('In rotatebot')
       # create Twist object
       twist = Twist()

       # get current yaw angle
       current_yaw = self.yaw
       # log the info
       self.get_logger().info('Current: %f' % math.degrees(current_yaw))
       # we are going to use complex numbers to avoid problems when the angles go from
       # 360 to 0, or from -180 to 180
       c_yaw = complex(math.cos(current_yaw), math.sin(current_yaw))
       # calculate desired yaw
       target_yaw = current_yaw + math.radians(rot_angle)
       # convert to complex notation
       c_target_yaw = complex(math.cos(target_yaw), math.sin(target_yaw))
       self.get_logger().info('Desired: %f' % math.degrees(cmath.phase(c_target_yaw)))
       # divide the two complex numbers to get the change in direction
       c_change = c_target_yaw / c_yaw
       # get the sign of the imaginary component to figure out which way we have to turn
       c_change_dir = np.sign(c_change.imag)
       # set linear speed to zero so the TurtleBot rotates on the spot
       twist.linear.x = x
       # set the direction to rotate
       twist.angular.z = c_change_dir * angular
       # start rotation
       self.publisher_.publish(twist)

       # we will use the c_dir_diff variable to see if we can stop rotating
       c_dir_diff = c_change_dir
       # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
       # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
       # becomes -1.0, and vice versa
       while(c_change_dir * c_dir_diff > 0):
           # allow the callback functions to run
           rclpy.spin_once(self)
           current_yaw = self.yaw
           # convert the current yaw to complex form
           c_yaw = complex(math.cos(current_yaw), math.sin(current_yaw))
           # self.get_logger().info('Current Yaw: %f' % math.degrees(current_yaw))
           # get difference in angle between current and target
           c_change = c_target_yaw / c_yaw
           # get the sign to see if we can stop
           c_dir_diff = np.sign(c_change.imag)
           # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))

       self.get_logger().info('End Yaw: %f' % math.degrees(current_yaw))
       # set the rotation speed to 0
       twist.angular.z = 0.0
       # stop the rotation
       self.publisher_.publish(twist)

    # Move in a desired angle
    def move_angle(self, angle, dist, speed=0.12):
        #distx = -distx
        dangle = math.degrees(angle)
        print("Angle and Dist: ", dangle, dist)
        
        #print("angle: ", angle)
        if dangle != 0:
            self.rotatebot(dangle)
        iyaw = complex(math.cos(self.yaw), math.sin(self.yaw))
        #dist = math.sqrt(distx**2 + disty**2)
        #print("Dist: ", dist)

        ix = self.locx
        iy = self.locy
        cdist = 0


        while cdist < dist:
            twist = Twist()
            twist.linear.x = speed
            cyaw = self.yaw
            cyaw = complex(math.cos(cyaw), math.sin(cyaw))
            c_change = iyaw/cyaw
            c_change_dir = np.sign(c_change.imag)
            twist.angular.z = c_change_dir * 0.05

            self.publisher_.publish(twist)
            rclpy.spin_once(self)
            cx = self.locx
            cy = self.locy
            cdist = math.sqrt((cx-ix)**2+(cy-iy)**2)
            print(cx, ix, cy, iy, cdist, dist)
            
        self.stopbot()
            
            


        
        

    # WALL TRACKING FUNCTION
    def pick_direction(self):
        self.get_logger().info('In pick direction:')

        # Getting distance
        self.front_dist = np.nan_to_num(self.laser_range[0], copy=False, nan=100)
        
        self.leftfront_dist = np.nan_to_num(self.laser_range[45], copy=False, nan=100)
        self.left_dist = np.nan_to_num(self.laser_range[90], copy=False, nan=100)

        self.rightfront_dist = np.nan_to_num(self.laser_range[315], copy=False, nan=100)
        self.right_dist = np.nan_to_num(self.laser_range[270], copy=False, nan=100)

        if self.follow == "Right":
            self.get_logger().info('Front Distance: %s' % str(self.front_dist))
            self.get_logger().info('Front Right Distance: %s' % str(self.rightfront_dist))
            self.get_logger().info('Right Distance: %s' % str(self.right_dist))
        else:
            self.get_logger().info('Front Distance: %s' % str(self.front_dist))
            self.get_logger().info('Front Left Distance: %s' % str(self.leftfront_dist))
            self.get_logger().info('Left Distance: %s' % str(self.left_dist))           

        # Set up twist message as msg
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        # ensure that there is no nan or 100
        if self.front_dist == np.nan or self.front_dist == 100:
            self.front_dist = self.oldval["f"]
        if self.leftfront_dist == np.nan or self.leftfront_dist == 100:
            self.leftfront_dist = self.oldval["lf"]
        if self.rightfront_dist == np.nan or self.rightfront_dist == 100:
            self.rightfront_dist = self.oldval["rf"]
        if self.left_dist == np.nan or self.left_dist == 100:
            self.left_dist = self.oldval["l"]
        if self.right_dist == np.nan or self.right_dist == 100:
            self.right_dist = self.oldval["r"]

        # selected d1 and d0 for angle track
        if self.follow == "Right":
            d1 = self.rightfront_dist
            d0 = self.right_dist
        else:
            d1 = self.leftfront_dist
            d0 = self.left_dist

        # if it sees nothing or the tracking side distance is very far, it will turn 45 degree
        if (self.front_dist > 0.5 and self.right_dist > 0.5 and self.left_dist > 0.5) or (self.follow == "Right" and self.right_dist > 0.7) or  (self.follow == "Left" and self.left_dist > 0.7):
            if self.follow == "Right" and (np.nanmin(self.laser_range[180:271]) < self.d):
                self.get_logger().info("Turning 45 Right")
                self.rotatebot(-45,0.03)
                return

            elif self.follow == "Left" and (np.nanmin(self.laser_range[90:181]) < self.d):
                self.get_logger().info("Turning 45 Left")
                self.rotatebot(45,0.03)
                return

        if np.nanmin(np.append(self.laser_range[0:45],self.laser_range[315:359])) < 0.2:
            if self.follow == "Left":
                self.get_logger().info("Turning Right and Reverse")
                self.rotatebot(-10,-0.05)
            else:
                self.get_logger().info("Turning Left and reverse")
                self.rotatebot(10,-0.05)
                      
        # calculate theta
        height = d1*math.sin(math.radians(45))
        proj = (height/math.tan(math.radians(45))) 
        width = proj - d0
        theta = math.degrees(math.atan2(width,height))
        if self.follow == "Right":
            doffset = -round(theta)
        else:
            doffset = round(theta)

        # if front very far then can ignore offset, it means it is tracking lone wall
        if d1 > 0.7:
            doffset = 0

        print("doffset: ",doffset)

        # offset the front lidar position so that the value we get
        # for front is the true front
        if doffset - 15 < 0:
            if doffset + 15 > 0:
                frontmind = np.nanmin(np.append(self.laser_range[0:15+doffset],self.laser_range[doffset-15+360:360]))
                print(0,15+doffset,doffset-15+360,360)
            else:
                frontmind = np.nanmin(self.laser_range[doffset-15+360:doffset+15+360])
                print(doffset-15+360,doffset+15+360)
        else:
            frontmind = np.nanmin(self.laser_range[doffset-15:doffset+15])
            print(doffset-15,doffset+15)
        
        print("frontmind: ",frontmind)

        # if obstacle at front
        if frontmind < 0.35:
            # front and left and right got obsacle
            # caclulate the angle of obstacle that is infront and turn accordingly
            if self.leftfront_dist < self.d and self.rightfront_dist < self.d:
                self.get_logger().info("Angled Turn (surrounded by obs)")
                frontdist = np.append(self.laser_range[20:-1:-1],self.laser_range[359:339:-1])
                minfront = np.nanargmin(frontdist)
                
                if self.follow == "Right":
                    print('right')
                    sidedist = self.laser_range[315:214:-1]
                    sidemin = np.nanargmin(sidedist)
                    print(sidemin)
                    angle = (40-minfront) + 25 + sidemin
                    print(type(angle))
                    self.get_logger().info('Angle: %i' % angle)
                    if angle > 90:
                        self.rotatebot((180-angle)*0.9,0.0,0.45)
                    else:
                        self.rotatebot(90,0.0) 

                else:
                    sidedist = self.laser_range[45:176]
                    sidemin = np.nanargmin(sidedist)
                    angle = minfront + 25 + sidemin
                    self.get_logger().info('Angle: %i' % angle)
                    if angle > 90:
                        self.rotatebot((angle-180)*0.9,0.0,0.45)
                    else:
                        self.rotatebot(-90,0.0)


            # if front and right got obstacle
            elif self.rightfront_dist < self.d:
                self.get_logger().info("Obstacle In Front and Right")
                if self.follow == "Right":
                    self.rotatebot(90,0.0)
                else:
                    self.rotatebot(-90,0.0)

            # if front and left got obstacle
            elif self.leftfront_dist < self.d:
                self.get_logger().info("Obstacle In Front and left")
                if self.follow == "Left":
                    self.rotatebot(-90,0.0)
                else:
                    self.rotatebot(90,0.0)

            # if front obstacle only
            else:
                self.get_logger().info("Obstacle In Front only")
                if self.follow == "Right":     
                    self.rotatebot(90,0.0)
                else:
                    self.rotatebot(-90,0.0)


            self.get_logger().info('Front Distance: %s' % str(self.front_dist))
            self.get_logger().info('Front Left Distance: %s' % str(self.leftfront_dist))
            self.get_logger().info('Front Right Distance: %s' % str(self.rightfront_dist))


        
            
        # if no obstacle in front
        else:
            self.get_logger().info("Wall Tracking")
            # get the minimum distance to the tracking side of the bot
            if self.follow == "Left":
                leftside = self.laser_range[45:91]
                mind = np.nanmin(leftside)
                
            else:
                rightside = self.laser_range[270:291]
                mind = np.nanmin(rightside)
                
            self.get_logger().info('Min Distance: %s' % str(mind))

            # turn according to angle of robot from the wall
            if self.follow =="Right":
                theta = -theta 
                msg.angular.z = (theta + (0.25-mind)*self.k_diff)*self.k_theta
            else:
                
                msg.angular.z = (theta + (mind-0.25)*self.k_diff)*self.k_theta 

            print("Theta: ",theta)
            print("Angular Z before Limit: ",msg.angular.z)        

            msg.linear.x = self.forward_speed

            # limit the max turning speed
            if msg.angular.z > 0.45:
                msg.angular.z = 0.45
            if msg.angular.z < -0.45:
                msg.angular.z = -0.45

        self.get_logger().info("x: %f" % msg.linear.x)
        self.get_logger().info("z: %f" % msg.angular.z)

        # update the old value to prevent death
        self.oldval["lf"] = self.leftfront_dist
        self.oldval["f"] = self.front_dist
        self.oldval["rf"] = self.rightfront_dist
        self.oldval["r"] = self.right_dist
        self.oldval["l"] = self.left_dist

        # Send velocity command to the robot
        self.publisher_.publish(msg)

    def stopbot(self):
        self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # time.sleep(1)
        self.publisher_.publish(twist)

    def mover(self):
        global myoccdata
        try:
            rclpy.spin_once(self)

            # ensure that we have a valid lidar data before we start wall follow logic
            while (self.laser_range.size == 0):
                print("Spin to get a valid lidar data")
                rclpy.spin_once(self)

            # initial move to find the appropriate wall to follow
            #self.move_angle(0.0,0.5,0.12)
            # start wall follow logic

            #self.move_angle(0.16, -0.14, 0.12)

            if self.movelist:
                movement = self.movelist.pop(0)
                self.move_angle(movement[0],movement[1],0.12)

            while rclpy.ok():
                if self.laser_range.size != 0:
                  
                    # if NFC Detected and not loaded
                    if self.loaded == False and self.nfcDetected:
                        self.stopbot()
                        self.get_logger().info("Stop bot")
                        # wait for button to be pressed
                        while (not self.loaded):
                            self.get_logger().info("Waiting for Button")
                            rclpy.spin_once(self)

                    # if AMG Detected Heat Signature
                    if self.isTargetDetected and self.loaded: # add check for loaded
                        #self.stopbot()
                        self.get_logger().info("Stop bot")
                                        # allow the callback functions to run

                        if not self.shot:
                            if self.turn_left:
                               self.rotatebot(float(1), 0.0, 0.1)
                            elif self.turn_right:
                                self.rotatebot(float(-1), 0.0, 0.1)
                            elif self.forward:
                                while self.laser_range[0] == 0 or self.laser_range[1] == np.nan:
                                    rclpy.spin_once(self)
                                sweep = np.append(self.laser_range[0:10], self.laser_range[350:360])
                                print(sweep)
                                if np.nanmax(sweep)>0.3:
                                    twist = Twist()
                                    twist.linear.x = 0.2
                                    twist.angular.z = 0.0
                                    self.publisher_.publish(twist)
                                else:
                                    spd = Int8()
                                    spd.data = 40
                                    self.rotatebot(110, 0.0, 0.25)
                                    self.shoot_dist = []
                                    #while rclpy.ok():
                                        #if len(self.shoot_dist)==0:
                                        #    if (self.laser_range[0] == 0 or self.laser_range[179] == 0):
                                        #        continue
                                        #    else:
                                        #       self.shoot_dist = [self.laser_range[0], self.laser_range[179]]
                                        #if (self.laser_range[0] != 0 and self.shoot_dist[0] - 0.07 > self.laser_range[0]) or (self.laser_range[179] != 0  and self.shoot_dist[1] - 0.07 > self.laser_range[179]):
                                        #    break
                                        #twist.linear.x = 0.05
                                        #twist.angular.z = 0.0
                                        #self.publisher_.publish(twist)
                                        #rclpy.spin_once(self)
                                    self.stopbot()
                                    self.shoot_publisher.publish(spd)
                                    self.shot = True

                    # while there is no target detected, keep picking direction (do wall follow)
                    elif self.shot:
                        self.stopbot()
                    else:
                        if self.movelist:
                            movement = self.movelist.pop(0)
                            self.move_angle(movement[0],movement[1],0.12)


                rclpy.spin_once(self)
        except KeyboardInterrupt:
            self.stopbot()
        
        except Exception as e:
            print(e)

        # Ctrl-c detected
        finally:
            # stop moving
            self.stopbot()


def main(args=None):
    rclpy.init(args=args)

    auto_nav = AutoNav()
    auto_nav.mover()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    auto_nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
