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
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import scipy.stats
from PIL import Image
from contextlib import suppress

# constants
rotatechange = 0.5
speedchange = 0.1
occ_bins = [-1, 0, 50, 100]
stop_distance = 0.25
front_angle = 30
front_angles = range(-front_angle, front_angle+1, 1)
scanfile = 'lidar.txt'
mapfile = 'mapnew.txt'
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
    print(result)
    return result


class AutoNav(Node):

    def __init__(self):
        super().__init__('auto_nav')

        # create publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # self.get_logger().info('Created publisher')

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

        # create subscription to track lidar
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning
        self.laser_range = np.array([])

    def odom_callback(self, msg):
        # self.get_logger().info('In odom_callback')
        orientation_quat = msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(
            orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)

    def occ_callback(self, msg):
        self.get_logger().info('In occ_callback')
        # create numpy array
        print("Doing Set Up")
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

        print("Find Start Location on Original Map")
        # get map grid positions for x,y position
        grid_x = round((cur_pos.x - map_origin.x) / map_res)
        grid_y = round((cur_pos.y - map_origin.y) / map_res)
        # self.get_logger().info('Grid Y: %i Grid X: %i' % (grid_y, grid_x))

        # binnum go from 1 to 3 so we can use uint8
        # convert into 2D array using column order
        odata = np.uint8(binnum.reshape(msg.info.height, msg.info.width))

        print("Finding the Wall Locations on Original Map")
        # for all the occupied points(=3), make the closest woah=1 cell(s) black also
        result_wall = np.where(odata == 3)
        coordinates_wall = list(zip(result_wall[0], result_wall[1]))

        '''
        print("Making the Walls Larger on Original Map")
        woah = 1
        for wall_x, wall_y in [coord for coord in coordinates_wall]:
            for i in range(-woah, woah):
                for j in range(-woah, woah):
                    with suppress(IndexError):
                        odata[wall_x + i, wall_y + j] = 3
        '''

        print("Finding the Goal Location")
        print("Finding All Explored Points")
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
        
        print("Checking if Unexplored Points are next to Explored Points")
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

        print("Checking if Explored-Unexplored Points are next to Walls")
        for coord in correct_coordinates:
            if check_if_neighbour_wall(coord) == False:
                correcter_coordinates.append(coord)

        correct_coordinates = correcter_coordinates

        # print(correct_coordinates)

        def get_distance(place):
            return (place[0]-grid_x)**2 + (place[1]-grid_y)**2

        print("Find distances between start point and goal coordinates")
        distances_between = list(map(get_distance, correct_coordinates))
        # print(distances_between)
        print("Finding min distance between start point and goal points")
        min_dist = np.amin(distances_between)
        print("Find the index of the goal coordinate with min dist")
        goals = np.where(distances_between == min_dist)
        # print(goals)
        goal_index = goals[0][0]
        # print(goal_index)
        print("Get the correct goal coordinate")
        goal_x, goal_y = correct_coordinates[goal_index]
        # self.get_logger().info('Goal_X: %i Goal_Y: %i' % (goal_x, goal_y))

        print("set start location to 0 on original map")
        # set current robot location to 0
        print(grid_y,grid_x)
        for i in range(-1, 1):
                for j in range(-1, 1):
                    with suppress(IndexError):
                        odata[grid_y+i, grid_x+j] = 0

        print("set goal location to 4 on original map")
        # set goal location to 4
        for i in range(-1, 1):
                for j in range(-1, 1):
                    with suppress(IndexError):
                        odata[goal_x+i, goal_y+j] = 4

        print("transfer original array to image with shifting")
        # print("Goal edits done")
        # create image from 2D array using PIL
        img = Image.fromarray(odata)
        img_transformed = Image.new(img.mode, (iwidth,iheight), map_bg_color)
        img_transformed.paste(img, (0,0))

        print("Rotate the array as necessary")
        # rotate by 90 degrees so that the forward direction is at the top of the image
        rotated = img_transformed.rotate(np.degrees(yaw)-90, expand=True, fillcolor=map_bg_color)
        rotated_array = np.copy(np.asarray(rotated))
        
        print("Find where start is on new map")
        start = np.where(rotated_array == 0)
        print("start is", start)
        start = list(zip(start[0], start[1]))
        start = start[0]
        print(start)
        self.start = start

        print("Find where end is on new map")
        end = np.where(rotated_array == 4)
        print("end is", end)
        end = list(zip(end[0], end[1]))
        end = end[0]
        print(end)
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
        
        self.occdata = np.array([])

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
        # rotate_angle in radians
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
            self.get_logger().info('Current Yaw: %f' % math.degrees(current_yaw))
            # get difference in angle between current and target
            c_change = c_target_yaw / c_yaw
            # get the sign to see if we can stop
            c_dir_diff = np.sign(c_change.imag)
            self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))

        self.get_logger().info('End Yaw: %f' % math.degrees(current_yaw))
        # set the rotation speed to 0
        twist.angular.z = 0.0
        # stop the rotation
        self.publisher_.publish(twist)

    def pick_direction(self):
        
        where_to_go = self.where_to_go
        print("check if there is any instruction on where to go")
        if where_to_go != None:
            self.get_logger().info('starting pick direction')
            for i in range(len(where_to_go)-1):
                next_closest = where_to_go[i]

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

                print("find direction to go in")
                # find angle between direction vector and current direction
                direction_vector = (next_closest[0]-self.start[0], next_closest[1]-self.start[1])
                angle_i_want = angle_between((0, 1), direction_vector)
                angle_i_want = np.degrees(angle_i_want) // 1
                self.angle_i_want = -1 * angle_i_want

                print("print out next_closest and angle_i_want")
                # print out next closest location & desired angle
                print(next_closest)
                print(angle_i_want)

                # find the distance between both points
                length = np.linalg.norm(direction_vector)
                time_taken = length // speedchange

                self.rotatebot(float(angle_i_want))
                # start moving
                self.get_logger().info('Start moving[pick_direction]')
                twist = Twist()
                twist.linear.x = speedchange
                twist.angular.z = 0.0
                # not sure if this is really necessary, but things seem to work more
                # reliably with this
                time.sleep(1)
                self.publisher_.publish(twist)
                time.sleep(time_taken)
        

    def stopbot(self):
        self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # time.sleep(1)
        self.publisher_.publish(twist)

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
                        self.pick_direction()
                        
                self.pick_direction()
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
    auto_nav.mover()

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
