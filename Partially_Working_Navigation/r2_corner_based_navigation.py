# Copyright 2015 Open Source Robotics Foundation, Inc.
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

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from .astar import Astar
import numpy as np
import matplotlib.pyplot as plt
import math
import cmath
import time
import tf2_ros
import cv2
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import scipy.stats
from PIL import Image
from contextlib import suppress

# constants
rotatechange = 0.5
speedchange = 0.1
occ_bins = [-1, 0, 55, 100]
stop_distance = 0.35
front_angle = 15
front_angles = range(-front_angle, front_angle + 1, 1)
back_angles = range(150, 210 + 1, 1)
ninety_degrees_right_side_angles = range(255, 285 + 1, 1)
ninety_degrees_left_side_angles = range(75, 105 + 1, 1)
scanfile = 'lidar.txt'
mapfile = 'mapnew.txt'
map_bg_color = 1
occdata = np.array([])
calculated_distance_to_goal = math.inf
last_zero_degree_distance = math.inf
list_of_goals = []
current_zero_degree_distance = 0
required_angle_turn_in_degree = 0
turn_right = False
going_straight = False

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


def getListOfPath(currMap, start, end):
    mat = []
    innerMat = []

    for i in currMap:
        for j in i:
            if j == 1:
                # this means that this is unexplored
                innerMat.append(0)
            elif j == 2 or j == 0:
                # this means this is explored & unoccupied
                innerMat.append(0)
            elif j == 3:
                # this means this is an obstacle
                innerMat.append(None)
        mat.append(innerMat)
        innerMat = []

    astar = Astar(mat)
    result = astar.run(start, end)
    # print(result)
    return result


class AutoNav(Node):

    def __init__(self):
        super().__init__('auto_nav')

        # Create publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # self.get_logger().info('Created publisher')

        # Create subscription to track orientation
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)

        # self.get_logger().info('Created subscriber')
        self.odom_subscription  # Prevent unused variable warning

        # Initialize variables
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        # Create subscription to track occupancy
        self.occ_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            qos_profile_sensor_data)

        self.occ_subscription  # Prevent unused variable warning
        self.occdata = np.array([])
        self.map_res = 0.5
        self.startpoint = []
        self.endpoint = []
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)

        # Create subscription to track lidar
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription  # Prevent unused variable warning
        self.laser_range = np.array([])

    def odom_callback(self, msg):

        # self.get_logger().info('In odom_callback')
        orientation_quat = msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(
            orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)

    def occ_callback(self, msg):
        global occdata, required_angle_turn_in_degree, list_of_goals
        self.get_logger().info('In occ_callback')

        # Create numpy array
        # print("Doing Set Up")
        msgdata = np.array(msg.data)
        # Compute histogram to identify percent of bins with -1, values btw 1 and below 50
        # between 50 and `100.
        occ_counts, edges, binnum = scipy.stats.binned_statistic(
            msgdata, np.nan, statistic='count', bins=occ_bins)

        # occ_counts = np.histogram(msgdata,occ_bins)
        # calculate total number of bins
        # total_bins = msg.info.width * msg.info.height
        # log the info
        # self.get_logger().info('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i' % (occ_counts[0][0], occ_counts[0][1], occ_counts[0][2], total_bins))

        # find transform to obtain base_link coordinates in the map frame
        # lookup_transform(target_frame, source_frame, time)
        try:
            trans = self.tfBuffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().info('No transformation found')
            return

        # Get start location !
        cur_pos = trans.transform.translation
        cur_rot = trans.transform.rotation

        # self.get_logger().info('Trans: %f, %f' % (cur_pos.x, cur_pos.y))

        # Convert quaternion to Euler angles
        roll, pitch, yaw = euler_from_quaternion(
            cur_rot.x, cur_rot.y, cur_rot.z, cur_rot.w)
        # self.get_logger().info('Rot-Yaw: R: %f D: %f' % (yaw, np.degrees(yaw)))

        # Get map resolution (0.05m/cell)
        map_res = msg.info.resolution
        self.map_res = map_res
        # self.get_logger().info('Resolution: %f' % (map_res))

        # Get map origin struct has field of x,y, and z
        map_origin = msg.info.origin.position

        # Get map width and height
        iwidth = msg.info.width
        iheight = msg.info.height

        # print("Find Start Location on Original Map")

        # Get map grid positions for x,y position
        grid_x = round((cur_pos.x - map_origin.x) / map_res)
        grid_y = round((cur_pos.y - map_origin.y) / map_res)
        # self.get_logger().info('Grid Y: %i Grid X: %i' % (grid_y, grid_x))

        # binnum go from 1 to 3 so we can use uint8
        # Convert into 2D array using column order
        odata = np.uint8(binnum.reshape(msg.info.height, msg.info.width))
        # Assigning odata to the global occdata variable for closure function
        occdata = odata

        # print("Finding the Wall Locations on Original Map")

        result_wall = np.where(odata == 3)
        coordinates_wall = list(zip(result_wall[0], result_wall[1]))

        # print("Finding the Goal Location")
        # print("Finding All Explored Points")

        # Find goal location
        result_explored = np.where(odata == 2)
        coordinates_explored = list(
            zip(result_explored[0], result_explored[1]))

        def check_if_neighbour_unexplored(place):
            a = place[0]
            b = place[1]
            for i in [-1, 0, 1]:
                for j in [-1, 0, 1]:
                    with suppress(IndexError):
                        if odata[a+i, b+j] == 1:
                            return True
            return False

        correct_coordinates = []

        # print("Checking if Unexplored Points are next to Explored Points")
        for coord in coordinates_explored:
            if check_if_neighbour_unexplored(coord) == True:
                correct_coordinates.append(coord)

        def check_if_neighbour_wall(place):
            a = place[0]
            b = place[1]
            for i in range(-2, 2):
                for j in range(-2, 2):
                    with suppress(IndexError):
                        if odata[a+i, b+j] == 3:
                            return True
            return False

        correcter_coordinates = []

        # print("Checking if Explored-Unexplored Points are next to Walls")
        for coord in correct_coordinates:
            if check_if_neighbour_wall(coord) == False:
                correcter_coordinates.append(coord)

        correct_coordinates = correcter_coordinates
        '''
        def check_if_reachable(place):
            a = place[0]
            b = place[1]
            if getListOfPath(odata,(grid_x,grid_y),(a,b)) == None:
                print("False")
                return False

            else:
                print("True")
                return True
        
        correcter_coordinates = []
        
        for coord in correct_coordinates:
            if check_if_reachable(coord) == True:
                correcter_coordinates.append(coord)
        
        correct_coordinates = correcter_coordinates
        # print(correct_coordinates)
        '''
        def get_distance(place):
            res = (place[1]-grid_x)**2 + (place[0]-grid_y)**2
            return res

        # print("Find distances between start point and goal coordinates")
        distances_between = list(map(get_distance, correct_coordinates))
        # print(distances_between)
        # print("Finding min distance between start point and goal points")
        # min_dist = min(distances_between)
        min_dist = min(distances_between)
        # print("Find the index of the goal coordinate with min dist")
        goals = np.where(distances_between == min_dist)
        # print(goals)
        goal_index = goals[0][0]
        # print(goal_index)
        # print("Get the correct goal coordinate")
        goal_x, goal_y = correct_coordinates[goal_index]
        # self.get_logger().info('Goal_X: %i Goal_Y: %i' % (goal_x, goal_y))

        # print("Set start location to 0 on original map")
        # set current robot location to 0
        # print(grid_y, grid_x)
        for i in range(-2, 2):
            for j in range(-2, 2):
                with suppress(IndexError):
                    odata[grid_y+i, grid_x+j] = 0

        # print("Set goal location to 4 on original map")
        # set goal location to 4
        for i in range(-1, 1):
            for j in range(-1, 1):
                with suppress(IndexError):
                    odata[goal_x+i, goal_y+j] = 4

        # to see all possible goals, uncomment
        '''
        for i in range(-1, 1):
            for j in range(-1, 1):
                for goals in correct_coordinates:
                    with suppress(IndexError):
                        odata[goals[0]+i, goals[1]+j] = 4
                        '''

        # print("Transfer original array to image with shifting")
        # print("Goal edits done")
        # create image from 2D array using PIL
        img = Image.fromarray(odata)
        img_transformed = Image.new(img.mode, (iwidth, iheight), map_bg_color)
        img_transformed.paste(img, (0, 0))

        # print("Rotate the array as necessary")
        # rotate by 90 degrees so that the forward direction is at the top of the image
        rotated = img_transformed.rotate(np.degrees(
            yaw)-90, expand=True, fillcolor=map_bg_color)
        rotated_array = np.copy(np.asarray(rotated))
        self.occdata = rotated_array

        # print("Find where start is on new map")
        start = np.where(rotated_array == 0)
        # print("start is", start)
        start = list(zip(start[0], start[1]))
        start = start[0]
        self.startpoint = start
        print("Start")
        print(self.startpoint)

        # print("Find where end is on new map")
        end = np.where(rotated_array == 4)
        # print("end is", end)
        end = list(zip(end[0], end[1]))
        list_of_goals = end
        end = end[0]
        self.endpoint = end
        print("End")
        print(self.endpoint)

        # notify start and end locations
        # self.get_logger().info('Start Y: %i Start X: %i' %
        # (start[1], start[0]))
        # self.get_logger().info('Goal Y: %i Goal X: %i' % (end[1], end[0]))

        # create image from 2D array using PIL
        # rotated = Image.fromarray(rotated)
        # show the image using grayscale map
        # plt.imshow(img, cmap='gray', origin='lower')
        # plt.imshow(img_transformed, cmap='gray', origin='lower')
        plt.imshow(rotated, cmap='gray', origin='lower')
        plt.draw_all()
        # pause to make sure the plot gets created
        plt.pause(0.00000000001)

    def scan_callback(self, msg):
        # self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # print to file
        # np.savetxt(scanfile, self.laser_range)
        # replace 0's with nan
        self.laser_range[self.laser_range == 0] = np.nan

    # function to rotate the TurtleBot

    def rotatebot(self, rot_angle):
        # takes in a rot_angle which is in degree
        self.get_logger().info('In rotatebot')
        # create Twist object
        twist = Twist()

        # get current yaw angle
        current_yaw = self.yaw
        # log the info
        # self.get_logger().info('Current: %f' % math.degrees(current_yaw))
        # we are going to use complex numbers to avoid problems when the angles go from
        # 360 to 0, or from -180 to 180
        c_yaw = complex(math.cos(current_yaw), math.sin(current_yaw))
        # calculate desired yaw
        target_yaw = current_yaw + math.radians(rot_angle)
        # convert to complex notation
        c_target_yaw = complex(math.cos(target_yaw), math.sin(target_yaw))
        # self.get_logger().info('Desired: %f' % math.degrees(cmath.phase(c_target_yaw)))
        # divide the two complex numbers to get the change in direction
        c_change = c_target_yaw / c_yaw
        # get the sign of the imaginary component to figure out which way we have to turn
        c_change_dir = np.sign(c_change.imag)
        # set linear speed to zero so the TurtleBot rotates on the spot
        twist.linear.x = 0.0
        # set the direction to rotate
        twist.angular.z = c_change_dir * rotatechange
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
            # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' %
            # (c_change_dir, c_dir_diff))

        # self.get_logger().info('End Yaw: %f' % math.degrees(current_yaw))
        # set the rotation speed to 0
        twist.angular.z = 0.0
        # stop the rotation
        self.publisher_.publish(twist)

    def rotate_bot_to_face_goal(self):

        print("In rotate_bot_to_face_goal")
        global calculated_distance_to_goal, last_zero_degree_distance, turn_right, going_straight, list_of_goals
        # Ensure that laser_range.size is detecting something
        while (self.laser_range.size == 0 or self.startpoint == [] or self.endpoint == []):
            print("Spin to get a valid startpoint, endpoint and lidar data")
            rclpy.spin_once(self)

        rclpy.spin_once(self)
        # function to norm vectors

        def unit_vector(vector):
            """ Returns the unit vector of the vector.  """
            return vector / np.linalg.norm(vector)

        # function to find angle btw two vectors
        def angle_between(v1, v2):
            """ Returns the angle in radians between vectors 'v1' and 'v2'::

                    >>> angle_between((1, 0, 0), (0, 1, 0))
                    1.5707963267948966
                    >>> angle_between((1, 0, 0), (1, 0, 0))
                    0.0
                    >>> angle_between((1, 0, 0), (-1, 0, 0))
                    3.141592653589793
            """
            v1_u = unit_vector(v1)
            v2_u = unit_vector(v2)
            return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

        print("Calculated minimum distance to goal in metres")
        calculated_distance_to_goal = math.sqrt(
            ((self.endpoint[0] - self.startpoint[0]) * 0.05)**2 + ((self.endpoint[1] - self.startpoint[1]) * 0.05)**2)
        print(calculated_distance_to_goal)

        print("Find goal direction to go in")
        # find angle between direction vector and current direction
        # direction_vector_end_point_to_start_point = (self.endpoint[0]-self.startpoint[0], self.endpoint[1]-self.startpoint[1])
        direction_vector = (
            self.endpoint[0]-self.startpoint[0], self.endpoint[1]-self.startpoint[1])
        angle_i_want_in_radians = angle_between((1, 0), direction_vector)

        angle_i_want = np.degrees(angle_i_want_in_radians)

        '''
        # Quadrant 1
        if self.endpoint[0] - self.startpoint[0] > 0 and self.endpoint[1] - self.startpoint[1] > 0:
            print("I'm in first Quadrant, turn right")
            angle_i_want = -1 * np.degrees(angle_between((0, 1), direction_vector_end_point_to_start_point))
        
        if self.startpoint[0] - self.endpoint[0] > 0:
            # If goal point is to the right, rotate right
            angle_i_want = -1 * np.degrees(angle_i_want_in_radians)

        else:
            # If goal point is to the left, rotate left
            # If straight/backward then rotate accordingly
            angle_i_want = np.degrees(angle_i_want_in_radians)
        '''
        print("Angle_i_want")
        # print out next desired angle
        print(round(angle_i_want))
        breakNow = False
        for i in range(0, 361, 1):
            print("Spin-spin spinnnnn ~~~")
            if (breakNow):
                break
            self.rotatebot(1)
            self.stopbot()
            rclpy.spin_once(self)
            for j in list_of_goals:
                print("Calculated minimum distance to goal in metres")
                calculated_distance_to_goal = math.sqrt(
                    ((j[0] - self.startpoint[0]) * 0.05)**2 + ((j[1] - self.startpoint[1]) * 0.05)**2)
                print(calculated_distance_to_goal)
                if (abs(self.startpoint[0] - j[0]) <= (math.pi / 12 * calculated_distance_to_goal) and j[1] >= self.startpoint[1]):
                    breakNow = True
                    break
            rclpy.spin_once(self)

        # while abs(self.startpoint[0] - self.endpoint[0]) <= (math.pi/6 * calculated_distance_to_goal) or self.endpoint[1] >= self.startpoint[1]:
        #     print("Spin spin until aligned in the 0 degrees...will it ever terminate?")
        #     self.rotatebot(1)
        #     self.stopbot()
        #     rclpy.spin_once(self)
        rclpy.spin_once(self)
        # print("It terminated!")
        last_zero_degree_distance = calculated_distance_to_goal
        print("Rotating the bot towards the goal")
        # time.sleep(15)
        # self.rotatebot(round(angle_i_want))
        print("Rotation Finished")
        self.stopbot()
        # time.sleep(15)
        # offset_in_degrees = int(round(np.degrees(
        #     0.35 / calculated_distance_to_goal)))
        # if (self.laser_range[offset_in_degrees] >= self.laser_range[-1 * offset_in_degrees]):
        #     self.rotatebot(offset_in_degrees)
        # else:
        #     self.rotatebot(-offset_in_degrees)
        current_zero_degree_distance = self.laser_range[0]
        if (current_zero_degree_distance >= last_zero_degree_distance):
            print("Inside going-straight in rotate_bot_to_face_goal")
            turn_right = False
            going_straight = True
            twist = Twist()
            twist.linear.x = speedchange
            twist.angular.z = 0.0
            # not sure if this is really necessary, but things seem to work more
            # reliably with this
            time.sleep(1)
            self.publisher_.publish(twist)
        else:
            turn_right = False
            going_straight = False

    def move_backwards(self):
        global turn_right, going_straight
        print("Inside move backwards")
        right_degree_turn = -90
        left_degree_turn = 90
        turn_right = False
        going_straight = False
        while abs(right_degree_turn) == 90 and left_degree_turn == 90 and len((self.laser_range[back_angles] < float(stop_distance)).nonzero()[0]) <= 0:
            print("Struggling to get out of a sad corner")
            for right in range(0, -91, -1):
                if (self.laser_range[right] - self.laser_range[right - 1]) < -0.65:
                    right_degree_turn = right
                    break
            for left in range(0, 91, 1):
                if (self.laser_range[left] - self.laser_range[left + 1] < -0.65):
                    left_degree_turn = left
                    break
            twist = Twist()
            twist.linear.x = -speedchange
            twist.angular.z = 0.0
            # not sure if this is really necessary, but things seem to work more
            # reliably with this
            time.sleep(1)
            self.stopbot()
            self.publisher_.publish(twist)
        self.rotate_bot_to_face_goal()
        if (not going_straight):
            self.my_pick_direction()

    def move_back_for_2_second(self):
        twist = Twist()
        twist.linear.x = -speedchange
        twist.angular.z = 0.0
        # not sure if this is really necessary, but things seem to work more
        # reliably with this
        time.sleep(2)
        self.stopbot()
        self.publisher_.publish(twist)

    def my_pick_direction(self):
        global last_zero_degree_distance, current_zero_degree_distance, turn_right, going_straight
        right_degree_turn = -90
        left_degree_turn = 90
        print("Inside my_pick_direction")
        while current_zero_degree_distance < last_zero_degree_distance:
            print("Searching for the right direction inside while loop")

            for right in range(0, -91, -1):
                if (self.laser_range[right] - self.laser_range[right - 1]) < -0.50:
                    right_degree_turn = right
                    break
            for left in range(0, 91, 1):
                if (self.laser_range[left] - self.laser_range[left + 1] < -0.50):
                    left_degree_turn = left
                    break
            if abs(right_degree_turn) == 90 and left_degree_turn == 90:
                # 180 degrees in front is all a dead end
                print("Searching for largest distance without care for goal point")
                lr2i = np.nanargmax(self.laser_range)
                turn_right = False
                # self.move_backwards()
                going_straight = True
                self.rotatebot(float(lr2i))
                self.stopbot()

            elif abs(right_degree_turn) < left_degree_turn:
                # turning to the right side
                turn_right = True
                going_straight = False
                offset_in_degrees = np.degrees(
                    0.4 / self.laser_range[right_degree_turn])
                print("Going Right in Pick Direction")
                print(float(right_degree_turn - offset_in_degrees))
                self.rotatebot(float(right_degree_turn - offset_in_degrees))
                self.stopbot()
            else:
                turn_right = False
                going_straight = False
                offset_in_degrees = np.degrees(
                    0.4 / self.laser_range[left_degree_turn])
                print("Going Left in Pick Direction")
                print(float(left_degree_turn - offset_in_degrees))
                self.rotatebot(float(left_degree_turn + offset_in_degrees))
                self.stopbot()

            last_zero_degree_distance = current_zero_degree_distance
            print("Last zero degree distance")
            print(last_zero_degree_distance)
            rclpy.spin_once(self)
            current_zero_degree_distance = self.laser_range[0]
            print("current zero degree distance")
            print(current_zero_degree_distance)
        lrin = (self.laser_range[front_angles] <
                float(stop_distance)).nonzero()
        last_zero_degree_distance = math.inf
        current_zero_degree_distance = 0
        # if(len(lrin[0]) > 0):
        twist = Twist()
        twist.linear.x = speedchange
        twist.angular.z = 0.0
        # not sure if this is really necessary, but things seem to work more
        # reliably with this
        time.sleep(1)
        self.publisher_.publish(twist)

    def move_forward(self):
        self.get_logger().info('In initialmove')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = speedchange
        twist.angular.z = 0.0
        # time.sleep(2)
        self.publisher_.publish(twist)

    def trialmover(self):
        global going_straight

        try:
            rclpy.spin_once(self)
            # initialize variable to write elapsed time to file
            contourCheck = 1
            start_time = time.time()
            self.rotate_bot_to_face_goal()
            if (not going_straight):
                # print("It's not going straight")
                self.my_pick_direction()

            while rclpy.ok():
                print(going_straight)
                print(turn_right)
                if self.laser_range.size != 0:
                    if contourCheck and len(occdata) != 0:
                        print("Inside contourCheck:")
                        if self.closure():
                            print("Inside selfclosure contourcheck:")
                            # map is complete, so save current time into file
                            with open("maptime.txt", "w") as f:
                                f.write("Elapsed Time: " +
                                        str(time.time() - start_time))
                            contourCheck = 0
                            # save the map
                            cv2.imwrite('mazemap.png', occdata)
                            print("Map is complete!")
                            break
                    # check distances in front of TurtleBot and find values less
                    # than stop_distance
                    lri = (self.laser_range[front_angles]
                           < float(stop_distance)).nonzero()
                    self.get_logger().info('Distances: %s' % str(lri))
                    lrleft = (self.laser_range[ninety_degrees_left_side_angles] < float(
                        stop_distance)).nonzero()
                    lrright = (self.laser_range[ninety_degrees_right_side_angles] < float(
                        stop_distance)).nonzero()
                    # if the list is not empty
                    if(len(lri[0]) > 0):
                        # stop moving
                        print("Stop before hit wall")
                        self.stopbot()
                        print("Spin to find a way away from the obstacles")
                        # curr_lidar_distance = -math.inf
                        # selectedindex = 0
                        # for i in range(-90, 90, 1):
                        #     if (self.laser_range[i] > curr_lidar_distance):
                        #         curr_lidar_distance = self.laser_range[i]
                        #         selectedindex = i
                        # while (len(lri[0]) > 0):
                        #     print("Spin to find a way away from the obstacles")
                        #     self.rotatebot(1)
                        #     self.stopbot()
                        #     rclpy.spin_once(self)
                        #     lri = (self.laser_range[front_angles] < float(
                        #         stop_distance)).nonzero()
                        # self.rotatebot(selectedindex)
                        # self.move_forward()
                        # going_straight = False
                        # self.stopbot()
                        self.rotate_bot_to_face_goal()
                        if (not going_straight):
                            #     self.my_pick_direction()
                            self.my_pick_direction()

                    elif going_straight:
                        pass

                    elif turn_right:
                        print("Turn right in trialmover")
                        if (len(lrleft[0]) > 0 or len(lrright[0]) > 0):
                            self.stopbot()
                            self.rotate_bot_to_face_goal()
                            self.my_pick_direction()

                    elif not turn_right:
                        print("Turn left in trialmover")
                        if (len(lrright[0]) > 0 or len(lrleft[0]) > 0):
                            self.stopbot()
                            self.rotate_bot_to_face_goal()
                            if (not going_straight):
                                self.my_pick_direction()

                    # find direction with the largest distance from the Lidar
                    # rotate to that direction
                    # start moving
                    # self.pick_direction()

                # allow the callback functions to run
                rclpy.spin_once(self)

        except Exception as e:
            print(e)

        # Ctrl-c detected
        finally:
            # stop moving
            print("this is in finally")
            self.stopbot()

    def pick_direction(self):

        # function to norm vectors
        def unit_vector(vector):
            """ Returns the unit vector of the vector.  """
            return vector / np.linalg.norm(vector)

        # function to find angle btw two vectors
        def angle_between(v1, v2):
            """ Returns the angle in radians between vectors 'v1' and 'v2'::

                    >>> angle_between((1, 0, 0), (0, 1, 0))
                    1.5707963267948966
                    >>> angle_between((1, 0, 0), (1, 0, 0))
                    0.0
                    >>> angle_between((1, 0, 0), (-1, 0, 0))
                    3.141592653589793
            """
            v1_u = unit_vector(v1)
            v2_u = unit_vector(v2)
            return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

        print("Find direction to go in")
        # find angle between direction vector and current direction
        direction_vector = (end[0]-start[0], end[1]-start[1])
        angle_i_want_in_radians = angle_between((1, 0), direction_vector)

        print("angle_i_want")
        # print out next desired angle
        print(angle_i_want)

        self.rotatebot(angle_i_want)
        # start moving
        twist = Twist()
        twist.linear.x = speedchange
        twist.angular.z = 0.0
        # not sure if this is really necessary, but things seem to work more
        # reliably with this
        time.sleep(1)
        self.publisher_.publish(twist)

    def avoid_obstacle(self):
        angle_found = False
        try:
            while angle_found == False:
                rclpy.spin_once(self)
                # initialize variable to write elapsed time to file
                # contourCheck = 1
                # self.get_logger().info('In pick_direction')
                if self.laser_range.size != 0:
                    # use nanargmax as there are nan's in laser_range added to replace 0's
                    lr2i = np.nanargmax(self.laser_range[front_angles])
                    self.get_logger().info('Picked direction: %d %f m' %
                                           (lr2i, self.laser_range[lr2i]))
                    angle_found = True
                else:
                    lr2i = 0
                    self.get_logger().info('No data!')

        finally:
            self.get_logger().info('Obstacle being avoided!')

        # rotate to that direction
        self.get_logger().info("Doing initial rotation")
        self.rotatebot(float(lr2i))
        twist = Twist()
        twist.linear.x = speedchange
        twist.angular.z = 0.0
        time.sleep(1)
        self.publisher_.publish(twist)
        time.sleep(2)
        self.pick_direction()

    def stopbot(self):
        self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # time.sleep(1)
        self.publisher_.publish(twist)

    def closure(self):
        # This function checks if mapdata contains a closed contour. The function
        # assumes that the raw map data from SLAM has been modified so that
        # -1 (unmapped) is now 0, and 0 (unoccupied) is now 1, and the occupied
        # values go from 1 to 101.

        # According to: https://stackoverflow.com/questions/17479606/detect-closed-contours?rq=1
        # closed contours have larger areas than arc length, while open contours have larger
        # arc length than area. But in my experience, open contours can have areas larger than
        # the arc length, but closed contours tend to have areas much larger than the arc length
        # So, we will check for contour closure by checking if any of the contours
        # have areas that are more than 10 times larger than the arc length
        # This value may need to be adjusted with more testing.
        global occdata

        ALTHRESH = 10
        # We will slightly fill in the contours to make them easier to detect
        DILATE_PIXELS = 3
        mapdata = occdata
        # assumes mapdata is uint8 and consists of 0 (unmapped), 1 (unoccupied),
        # and other positive values up to 101 (occupied)
        # so we will apply a threshold of 2 to create a binary image with the
        # occupied pixels set to 255 and everything else is set to 0
        # we will use OpenCV's threshold function for this
        ret, img2 = cv2.threshold(mapdata, 2, 255, 0)
        # we will perform some erosion and dilation to fill out the contours a
        # little bit
        element = cv2.getStructuringElement(
            cv2.MORPH_CROSS, (DILATE_PIXELS, DILATE_PIXELS))
        # img3 = cv2.erode(img2,element)
        img4 = cv2.dilate(img2, element)
        # use OpenCV's findContours function to identify contours
        # OpenCV version 3 changed the number of return arguments, so we
        # need to check the version of OpenCV installed so we know which argument
        # to grab
        fc = cv2.findContours(img4, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        (major, minor, _) = cv2.__version__.split(".")
        if(major == '3'):
            contours = fc[1]
        else:
            contours = fc[0]
        # find number of contours returned
        lc = len(contours)
        # rospy.loginfo('# Contours: %s', str(lc))
        # create array to compute ratio of area to arc length
        cAL = np.zeros((lc, 2))
        for i in range(lc):
            cAL[i, 0] = cv2.contourArea(contours[i])
            cAL[i, 1] = cv2.arcLength(contours[i], True)

        # closed contours tend to have a much higher area to arc length ratio,
        # so if there are no contours with high ratios, we can safely say
        # there are no closed contours
        cALratio = cAL[:, 0]/cAL[:, 1]
        # rospy.loginfo('Closure: %s', str(cALratio))
        if np.any(cALratio > ALTHRESH):
            return True
        else:
            return False

    def mover(self):
        try:
            rclpy.spin_once(self)
            # initialize variable to write elapsed time to file
            # contourCheck = 1
            # self.get_logger().info('In pick_direction')
            if self.laser_range.size != 0:
                # use nanargmax as there are nan's in laser_range added to replace 0's
                lr2i = np.nanargmax(self.laser_range)
                self.get_logger().info('Picked direction: %d %f m' %
                                       (lr2i, self.laser_range[lr2i]))
            else:
                lr2i = 0
                self.get_logger().info('No data!')

            # rotate to that direction
            self.get_logger().info("Doing initial rotation")
            self.rotatebot(float(lr2i))
            twist = Twist()
            twist.linear.x = speedchange
            twist.angular.z = 0.0
            self.publisher_.publish(twist)

            while rclpy.ok():
                if self.laser_range.size != 0:
                    # check distances in front of TurtleBot and find values less
                    # than stop_distance

                    lri = (self.laser_range[front_angles]
                           < float(stop_distance)).nonzero()
                    # self.get_logger().info('Distances: %s' % str(lri))
                    # print("This is lri:")
                    # print(lri)
                    # if the list is not empty
                    if(len(lri[0]) > 0):
                        # stop moving
                        self.stopbot()
                        # find direction with the largest distance from the Lidar
                        # rotate to that direction
                        # start moving
                        self.avoid_obstacle()
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

    auto_nav = AutoNav()
    auto_nav.trialmover()

    # create matplotlib figure
    plt.ion()
    plt.show()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    auto_nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
