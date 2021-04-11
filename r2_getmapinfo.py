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
from geometry_msgs.msg import Twist, Pose
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String, Float64MultiArray
import numpy as np
import matplotlib.pyplot as plt
import math
import cmath
import time
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import scipy.stats
from PIL import Image
from contextlib import suppress

# constants
occ_bins = [-1, 0, 50, 100]
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


class Mapping(Node):

    def __init__(self):
        super().__init__('mapping')

        # create publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(
            Pose,
            '/goal',
            10)
        # self.get_logger().info('Created publisher')

        # Call the callback functions every 5 seconds
        timer_period = 5
        self.timer1 = self.create_timer(timer_period, self.give_goal_values)

        """ self.goal_requests = self.create_subscription(
            String,
            'goal_request',
            self.give_goal_values,
            10) """

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

        # create subscription to track occupancy
        self.occ_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            qos_profile_sensor_data)
        self.occ_subscription  # prevent unused variable warning
        self.occdata = np.array([])
        self.map_res = 0.5
        self.start = []
        self.end = []
        self.where_to_go = []
        self.angle_i_want = 5
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)

        self.est_state_subscription = self.create_subscription(
            Float64MultiArray,
            '/state_est',
            self.state_estimate_callback,
            10)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.goal_x_coordinates = False
        self.goal_y_coordinates = False
        

    def odom_callback(self, msg):
        # self.get_logger().info('In odom_callback')
        position = msg.pose.pose.position
        # self.get_logger().info("%s" % str(position))

        orientation_quat = msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(
            orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)

    def occ_callback(self, msg):
        self.get_logger().info('In occ_callback')
        # create numpy array
        #print("Doing Set Up")
        msgdata = np.array(msg.data)
        # compute histogram to identify percent of bins with -1, values btw 1 and below 50
        # btw 50 and `100.
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

        # get start location !
        cur_pos = trans.transform.translation
        cur_rot = trans.transform.rotation
        # self.get_logger().info('Trans: %f, %f' % (cur_pos.x, cur_pos.y))
        # convert quaternion to Euler angles
        roll, pitch, yaw = euler_from_quaternion(
            cur_rot.x, cur_rot.y, cur_rot.z, cur_rot.w)
        # self.get_logger().info('Rot-Yaw: R: %f D: %f' % (yaw, np.degrees(yaw)))

        # get map resolution (0.05m/cell)
        map_res = msg.info.resolution
        self.map_res = map_res
        #self.get_logger().info('Resolution: %f' % (map_res))
        # get map origin struct has field of x,y, and z
        map_origin = msg.info.origin.position

        # get map width and height
        iwidth = msg.info.width
        iheight = msg.info.height

        #print("Find Start Location on Original Map")
        # get map grid positions for x,y position
        grid_x = round((cur_pos.x - map_origin.x) / map_res)
        grid_y = round((cur_pos.y - map_origin.y) / map_res)
        # self.get_logger().info('Grid Y: %i Grid X: %i' % (grid_y, grid_x))

        # binnum go from 1 to 3 so we can use uint8
        # convert into 2D array using column order
        odata = np.uint8(binnum.reshape(msg.info.height, msg.info.width))

        #print("Finding the Wall Locations on Original Map")
        # for all the occupied points(=3), make the closest woah=1 cell(s) black also
        result_wall = np.where(odata == 3)
        coordinates_wall = list(zip(result_wall[0], result_wall[1]))

        #print("Finding the Goal Location")
        #print("Finding All Explored Points")
        # find goal location
        result_explored = np.where(odata == 2)
        coordinates_explored = list(
            zip(result_explored[0], result_explored[1]))
        # print(coordinates_explored)

        # def check_if_neighbour_unexplored(a,b):
        def check_if_neighbour_unexplored(place):
            a = place[0]
            b = place[1]
            for i in [-1, 0, 1]:
                for j in [-1, 0, 1]:
                    # print(i,j)
                    with suppress(IndexError):
                        # print("checking new_map[%i %i]" % (a+i,b+j))
                        if odata[a+i, b+j] == 1:
                            return True
            return False

        correct_coordinates = []
        
        #print("Checking if Unexplored Points are next to Explored Points")
        for coord in coordinates_explored:
            if check_if_neighbour_unexplored(coord) == True:
                correct_coordinates.append(coord)

        #print("checking if neighbours explored")
        # print(correct_coordinates)

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

        #print("Checking if Explored-Unexplored Points are next to Walls")
        for coord in correct_coordinates:
            if check_if_neighbour_wall(coord) == False:
                correcter_coordinates.append(coord)

        correct_coordinates = correcter_coordinates

        # print(correct_coordinates)

        def get_distance(place):
            # print(place)
            # print(grid_x,grid_y)
            res = (place[1]-grid_x)**2 + (place[0]-grid_y)**2
            return res

        #print("Find distances between start point and goal coordinates")
        distances_between = list(map(get_distance, correct_coordinates))
        # print(distances_between)
        #print("Finding min distance between start point and goal points")
        min_dist = min(distances_between)
        #print("Find the index of the goal coordinate with min dist")
        goals = np.where(distances_between == min_dist)
        # print(goals)
        goal_index = goals[0][0]
        # print(goal_index)
        #print("Get the correct goal coordinate")
        goal_x, goal_y = correct_coordinates[goal_index]
        # self.get_logger().info('Goal_X: %i Goal_Y: %i' % (goal_x, goal_y))

        self.unrotatedstart = (grid_y,grid_x)
        self.unrotatedgoal = (goal_x,goal_y)

        #self.get_logger().info("Start is %s" % (str(self.unrotatedstart)))
        #self.get_logger().info("Goal is %s" % (str(self.unrotatedgoal)))

        #print("set start location to 0 on original map")
        # set current robot location to 0
        #print(grid_y,grid_x)
        for i in range(-2, 2):
                for j in range(-2, 2):
                    with suppress(IndexError):
                        odata[grid_y+i, grid_x+j] = 0

        #print("set goal location to 4 on original map")
        # set goal location to 4
        for i in range(-1, 1):
                for j in range(-1, 1):
                    with suppress(IndexError):
                        odata[goal_x+i, goal_y+j] = 4
        '''
        # to see all possible goals, uncomment
        
        for i in range(-1, 1):
                for j in range(-1, 1):
                    for goals in correct_coordinates:
                        with suppress(IndexError):
                            odata[goals[0]+i, goals[1]+j] = 4
        '''
        

        #print("transfer original array to image with shifting")
        # print("Goal edits done")
        # create image from 2D array using PIL
        img = Image.fromarray(odata)
        img_transformed = Image.new(img.mode, (iwidth,iheight), map_bg_color)
        img_transformed.paste(img, (0,0))

        #print("Rotate the array as necessary")
        # rotate by 90 degrees so that the forward direction is at the top of the image
        rotated = img_transformed.rotate(np.degrees(yaw)-90, expand=True, fillcolor=map_bg_color)
        rotated_array = np.asarray(rotated)
        self.occdata = rotated_array
        self.unrotatedoccdata = np.asarray(img_transformed)
        self.unrotatedmsginfo = msg
        print(self.occdata)
        
        #print("Find where start is on new map")
        start = np.where(rotated_array == 0)
        #print("start is", start)
        start = list(zip(start[0], start[1]))
        start = start[0]
        #print(start)
        self.start = start

        #print("Find where end is on new map")
        end = np.where(rotated_array == 4)
        #print("end is", end)
        end = list(zip(end[0], end[1]))
        end = end[0]
        #print(end)
        self.end = end

        # notify start and end locations
        self.get_logger().info('Start Y: %i Start X: %i' %
                               (start[1], start[0]))
        self.get_logger().info('Goal Y: %i Goal X: %i' % (end[1], end[0]))

        
        # create image from 2D array using PIL
        # rotated = Image.fromarray(rotated)
        # show the image using grayscale map
        #plt.imshow(img, cmap='gray', origin='lower')
        #plt.imshow(img_transformed, cmap='gray', origin='lower')
        plt.imshow(rotated, cmap='gray', origin='lower')
        plt.draw_all()
        # pause to make sure the plot gets created
        plt.pause(0.00000000001)
        '''
        # create image from 2D array using PIL
        odataimg = Image.fromarray(odata)
        # show the image using grayscale map
        #plt.imshow(img, cmap='gray', origin='lower')
        #plt.imshow(img_transformed, cmap='gray', origin='lower')
        plt.imshow(odataimg, cmap='gray', origin='lower')
        plt.draw_all()
        # pause to make sure the plot gets created
        plt.pause(0.00000000001)
        '''


    def state_estimate_callback(self,msg):
        """
        Extract the position and orientation data. 
        This callback is called each time
        a new message is received on the '/en613/state_est' topic
        """
        # Update the current estimated state in the global reference frame
        curr_state = msg.data
        self.current_x = curr_state[0]
        self.current_y = curr_state[1]
        self.current_yaw = curr_state[2]

        # Print the pose of the robot
        # Used for testing
        """ self.get_logger().info('X:%f Y:%f YAW:%f' % (
        self.current_x,
        self.current_y,
        np.rad2deg(self.current_yaw)))  # Goes from -pi to pi  """
    
    def give_goal_values(self):
        pose = Pose()

        diff_x = self.end[1] - self.start[1]
        diff_y = self.end[0] - self.start[0]
        diff_x *= 0.5
        diff_y *= 0.5

        pose.position.x = self.current_x
        pose.position.y = self.current_y

        pose.position.x -= diff_x
        pose.position.y -= diff_y

        self.publisher_.publish(pose)
        self.get_logger().info("Goal Published!")
        


def main(args=None):
    rclpy.init(args=args)
    
    mapping = Mapping()
    rclpy.spin(mapping)
    # create matplotlib figure
    plt.ion()
    plt.show()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mapping.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
