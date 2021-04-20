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
rotatechange = 1
speedchange = 0.22
occ_bins = [-1, 0, 50, 100]
stop_distance = 0.25
front_angle = 30
front_angles = range(-front_angle, front_angle+1, 1)
scanfile = 'lidar.txt'
mapfile = 'mapnew.txt'
map_bg_color = 1
occdata = np.array([])
margin_of_error = 5
current_position = ()
end_position = ()
list_of_paths = []
last_angle_turn = 0

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
            elif j == 2:
                # this means this is explored & unoccupied
                innerMat.append(0)
            elif j == 3:
                # this means this is an obstacle
                innerMat.append(None)
        mat.append(innerMat)
        innerMat = []

    astar = Astar(mat)
    result = astar.run(start, end)
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
        global occdata, current_position, list_of_paths, end_position

        self.get_logger().info('In occ_callback')
        # create numpy array
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
        #self.get_logger().info('Resolution: %f' % (map_res))
        # get map origin struct has field of x,y, and z
        map_origin = msg.info.origin.position

        # get map width and height
        iwidth = msg.info.width
        iheight = msg.info.height

        # get map grid positions for x,y position
        grid_x = round((cur_pos.x - map_origin.x) / map_res)
        grid_y = round((cur_pos.y - map_origin.y) / map_res)
        self.get_logger().info('Grid Y: %i Grid X: %i' % (grid_y, grid_x))

        # binnum go from 1 to 3 so we can use uint8
        # convert into 2D array using column order
        odata = np.uint8(binnum.reshape(msg.info.height, msg.info.width))
        occdata = odata

        # for all the occupied points(=3), make the closest 1 cell(s) black also
        result_wall = np.where(odata == 3)
        coordinates_wall = list(zip(result_wall[0], result_wall[1]))

        for wall_x, wall_y in [coord for coord in coordinates_wall]:
            for i in range(-1, 1):
                for j in range(-1, 1):
                    with suppress(IndexError):
                        odata[wall_x + i, wall_y + j] = 3

        # find goal location
        result_unexplored = np.where(odata == 1)
        coordinates_unexplored = list(
            zip(result_unexplored[0], result_unexplored[1]))
        # print(coordinates_unexplored)

        # def check_if_neighbour_explored(a,b):
        def check_if_neighbour_explored(place):
            a = place[0]
            b = place[1]
            for i in [-1, 0, 1]:
                for j in [-1, 0, 1]:
                    # print(i,j)
                    with suppress(IndexError):
                        # print("checking new_map[%i %i]" % (a+i,b+j))
                        if odata[a+i, b+j] == 2:
                            return True
            return False

        correct_coordinates = []

        for coord in coordinates_unexplored:
            if check_if_neighbour_explored(coord) == True:
                correct_coordinates.append(coord)

        #print("checking if neighbours explored")
        # print(correct_coordinates)

        def check_if_neighbour_wall(place):
            a = place[0]
            b = place[1]
            for i in range(-1, 1):
                for j in range(-1, 1):
                    with suppress(IndexError):
                        if odata[a+i, b+j] == 3:
                            return True
            return False

        correcter_coordinates = []
        for coord in correct_coordinates:
            if check_if_neighbour_wall(coord) == False:
                correcter_coordinates.append(coord)

        correct_coordinates = correcter_coordinates

        # print(correcter_coordinates)

        def get_distance(place):
            return (place[0]-grid_x)**2 + (place[1]-grid_y)**2

        distances_between = list(map(get_distance, correct_coordinates))
        # print(distances_between)
        min_dist = np.amin(distances_between)
        goals = np.where(distances_between == min_dist)
        # print(goals)
        goal_index = goals[0][0]
        # print(goal_index)
        goal_x, goal_y = correct_coordinates[goal_index]
        self.get_logger().info('Goal_X: %i Goal_Y: %i' % (goal_x, goal_y))

        # set current robot location to 0
        odata[grid_y][grid_x] = 0

        # set goal location to 4
        odata[goal_x][goal_y] = 4

        # print("Goal edits done")
        # create image from 2D array using PIL
        img = Image.fromarray(odata)
        # print("odata to img done")
        # find center of image
        i_centerx = iwidth/2
        i_centery = iheight/2
        # find how much to shift the image to move grid_x and grid_y to center of image
        shift_x = round(grid_x - i_centerx)
        shift_y = round(grid_y - i_centery)
        # self.get_logger().info('Shift Y: %i Shift X: %i' % (shift_y, shift_x))

        # pad image to move robot position to the center
        # adapted from https://note.nkmk.me/en/python-pillow-add-margin-expand-canvas/
        left = 0
        right = 0
        top = 0
        bottom = 0
        if shift_x > 0:
            # pad right margin
            right = 2 * shift_x
        else:
            # pad left margin
            left = 2 * (-shift_x)

        if shift_y > 0:
            # pad bottom margin
            bottom = 2 * shift_y
        else:
            # pad top margin
            top = 2 * (-shift_y)

        # create new image
        new_width = iwidth + right + left
        new_height = iheight + top + bottom
        img_transformed = Image.new(
            img.mode, (new_width, new_height), map_bg_color)
        img_transformed.paste(img, (left, top))

        # rotate by 90 degrees so that the forward direction is at the top of the image
        rotated = img_transformed.rotate(np.degrees(
            yaw)-90, expand=True, fillcolor=map_bg_color)
        rotated_array = np.copy(np.asarray(rotated))
        start = np.where(rotated_array == 0)

        # apparently this might cause problem...
        start = list(zip(start[0], start[1]))
        start = start[0]
        current_position = start
        print(start)

        end = np.where(rotated_array == 4)
        end = list(zip(end[0], end[1]))
        end = end[0]
        end_position = end
        print(end)

        # notify start and end locations
        self.get_logger().info('Start Y: %i Start X: %i' %
                               (start[1], start[0]))
        self.get_logger().info('Goal Y: %i Goal X: %i' % (end[1], end[0]))

        # find out how to get to chosen location
        where_to_go = getListOfPath(rotated_array, start, end)
        where_to_go.pop(0)
        if len(list_of_paths) > len(where_to_go) or len(list_of_paths) == 0:
            list_of_paths = where_to_go

        # create image from 2D array using PIL
        # rotated = Image.fromarray(rotated)
        # show the image using grayscale map
        #plt.imshow(img, cmap='gray', origin='lower')
        #plt.imshow(img_transformed, cmap='gray', origin='lower')
        plt.imshow(rotated, cmap='gray', origin='lower')
        plt.draw_all()
        # pause to make sure the plot gets created
        plt.pause(0.00000000001)

    def scan_callback(self, msg):
        # self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # print to file
        np.savetxt(scanfile, self.laser_range)
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

    def pick_direction(self):

        # self.rotatebot(angle_needed)
        # start moving
        self.get_logger().info('Start moving')
        twist = Twist()
        twist.linear.x = speedchange
        twist.angular.z = 0.0
        # not sure if this is really necessary, but things seem to work more
        # reliably with this
        time.sleep(1)
        self.publisher_.publish(twist)

    def my_move(self):
        global list_of_paths, last_angle_turn

        next_closest = list_of_paths.pop(0)

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

        # find angle between direction vector and current direction
        direction_vector = (
            next_closest[0]-current_position[0], next_closest[1]-current_position[1])
        angle_i_want = angle_between((0, 1), direction_vector)

        # print out next closest location & desired angle
        print("My current position")
        print(current_position)
        print("Next closest")
        print(next_closest)
        print("Angle I Want")
        print(list_of_paths)
        print(angle_i_want)
        if (abs(angle_i_want - last_angle_turn) >= 0.12):
            last_angle_turn = -1 * angle_i_want
            print("Last Angle Turn")
            print(last_angle_turn)
            self.rotatebot(last_angle_turn)
        else:
            self.rotatebot(0)

        self.get_logger().info('Try my_move')
        twist = Twist()
        twist.linear.x = speedchange
        twist.angular.z = 0.0

        time.sleep(1)
        self.publisher_.publish(twist)

    def stopbot(self):
        self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # time.sleep(1)
        self.publisher_.publish(twist)

    def initialmove(self):
        self.get_logger().info('In initialmove')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = speedchange
        twist.angular.z = 0.0
        time.sleep(2)
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
            # initialize variable to write elapsed time to file
            contourCheck = 1

            # start moving straight for the first 2 seconds
            self.initialmove()
            while rclpy.ok():
                if len(list_of_paths) != 0 and self.laser_range.size != 0:
                    # check distances in front of TurtleBot and find values less
                    # than stop_distance
                    lri = (self.laser_range[front_angles]
                           < float(stop_distance)).nonzero()
                    # self.get_logger().info('Distances: %s' % str(lri))

                    # if the list is not empty
                    if(len(lri[0]) > 0):
                        # stop moving
                        print("It's gonna crashhhhhh !!! I FUCKED UPP !")
                        self.stopbot()
                        break
                    # find direction with the largest distance from the Lidar
                    # rotate to that direction
                    # start moving
                    # self.pick_direction()
                    self.my_move()
                    # check if SLAM map is complete
                    if contourCheck and len(occdata) != 0:
                        if self.closure():
                            # map is complete, so save current time into file
                            with open("maptime.txt", "w") as f:
                                f.write("Elapsed Time: " +
                                        str(time.time() - start_time))
                            contourCheck = 0
                            # play a sound
                            soundhandle = SoundClient()
                            rospy.sleep(1)
                            soundhandle.stopAll()
                            soundhandle.play(SoundRequest.NEEDS_UNPLUGGING)
                            rospy.sleep(2)
                            # save the map
                            cv2.imwrite('mazemap.png', occdata)
                            break

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
    
    # create matplotlib figure
    plt.ion()
    plt.show()
    
    auto_nav.mover()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    auto_nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
