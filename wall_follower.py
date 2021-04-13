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

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float64MultiArray, String
import numpy as np
import matplotlib.pyplot as plt
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import cv2
import math
import cmath
import time
from PIL import Image

# constants
rotatechange = 0.5
speedchange = 0.15
occ_bins = [-1, 0, 100, 101]
stop_distance = 0.40
map_bg_color = 1

front_angles = range(-10, 10 + 1, 1)
frontleft_angles = range(44, 46 + 1, 1)
ninety_degrees_left_side_angles = range(85, 95 + 1, 1)
back_angles = range(150, 210 + 1, 1)
ninety_degrees_right_side_angles = range(265, 275 + 1, 1)
frontright_angles = range(310, 320+1, 1)

scanfile = 'lidar.txt'
mapfile = 'map.txt'
myoccdata = np.array([])
current_lrleft = 0
previous_lrleft = 0
isTargetDetected = False
isDoneShooting = False

# To change before starting test
stopping_time_in_seconds = 540 # 9 minutes
initial_direction = "Front" # "Front", "Left", "Right", "Back"

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


class AutoNav(Node):

    def __init__(self):
        super().__init__('auto_nav')

        # create publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # self.get_logger().info('Created publisher')

        # Create a subscriber
        # Node subscribes to messages from the targeting node
        self.targeting_subscription = self.create_subscription(
            String,
            'targeting_status',
            self.target_callback,
            10)
        self.targeting_subscription  # prevent unused variable warning

        # Create a subscriber
        # This node subscribes to messages of type Float64MultiArray
        # over a topic named: /en613/state_est
        # The message represents the current estimated state:
        #   [x, y, yaw]
        # The callback function is called as soon as a message
        # is received.
        # The maximum number of queued messages is 10.
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/state_est',
            self.state_estimate_callback,
            10)
        self.subscription  # prevent unused variable warning

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

    def state_estimate_callback(self, msg):
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

    def target_callback(self, msg):
        global isTargetDetected, isDoneShooting
        self.get_logger().info('In target_callback')
        self.get_logger().info('I heard: "%s"' % msg.data)
        if (msg.data == 'Detected'):
            print('Target Detected')
            isTargetDetected = True
            isDoneShooting = False
        elif (msg.data == 'Done'):
            print('Is Done shooting')
            isDoneShooting = True
            isTargetDetected = False
        else:
            print('No Target Detected')
            isTargetDetected = False

    def odom_callback(self, msg):
        self.get_logger().info('In odom_callback')
        orientation_quat = msg.pose.pose.orientation
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
        print('In occ callback:')
        self.occdata = np.uint8(oc2.reshape(msg.info.height, msg.info.width))
        myoccdata = np.uint8(oc2.reshape(msg.info.height, msg.info.width))
        odata = myoccdata
        """ cur_pos = trans.transform.translation
        cur_rot = trans.transform.rotation
        myroll, mypitch, myyaw = euler_from_quaternion(
            cur_rot.x, cur_rot.y, cur_rot.z, cur_rot.w)
        map_res = msg.info.resolution
        # Get map origin struct has field of x,y, and z
        map_origin = msg.info.origin.position
        # Get map grid positions for x,y position
        # Get map width and height
        iwidth = msg.info.width
        iheight = msg.info.height
        grid_x = round((cur_pos.x - map_origin.x) / map_res)
        grid_y = round((cur_pos.y - map_origin.y) / map_res)
        odata[grid_y][grid_x] = 0
        img = Image.fromarray(odata)
        img_transformed = Image.new(img.mode, (iwidth, iheight), map_bg_color)
        img_transformed.paste(img, (0, 0))
        rotated = img_transformed.rotate(np.degrees(
            myyaw) - 90, expand=True, fillcolor=map_bg_color) """

        # For some reason, plotting will cause the remaining wall
        # follower to work weirdly
        # plt.imshow(odata, cmap='gray', origin='lower')
        # plt.draw_all()
        # pause to make sure the plot gets created
        # plt.pause(0.00000000001)
        # print to file
        np.savetxt(mapfile, self.occdata)

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

        # lrfront = (self.laser_range[front_angles] < float(stop_distance)).nonzero()
        # lrfrontleft = (self.laser_range[frontleft_angles] < float(stop_distance)).nonzero()
        # lrleft = (self.laser_range[ninety_degrees_left_side_angles] < float(stop_distance)).nonzero()
        # lrright = (self.laser_range[ninety_degrees_right_side_angles] < float(stop_distance)).nonzero()
        # lrfrontright = (self.laser_range[frontright_angles] < float(stop_distance)).nonzero()
        # self.get_logger().info('Distances: %s' % str(lrfront))
        self.front_dist = np.nan_to_num(
            self.laser_range[0], copy=False, nan=100)
        self.leftfront_dist = np.nan_to_num(self.laser_range[45], copy=False, nan = 100)
        self.rightfront_dist = np.nan_to_num(self.laser_range[315], copy=False, nan = 100)

        self.get_logger().info('Front Distance: %s' % str(self.front_dist))
        self.get_logger().info('Front Left Distance: %s' % str(self.leftfront_dist))
        self.get_logger().info('Front Right Distance: %s' % str(self.rightfront_dist))
        # Logic for following the wall
        # >d means no wall detected by that laser beam
        # <d means an wall was detected by that laser beam
        d = 0.28
        # Set turning speeds (to the left) in rad/s
        # These values were determined by trial and error.
        self.turning_speed_wf_fast = 0.75  # Fast turn ideal = 1.0
        self.turning_speed_wf_slow = 0.40  # Slow turn = 0.50
        # Set movement speed
        self.forward_speed = speedchange
        # Set up twist message as msg
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        if self.leftfront_dist > d and self.front_dist > d and self.rightfront_dist > d:
            self.wall_following_state = "search for wall"
            msg.linear.x = self.forward_speed
            msg.angular.z = -self.turning_speed_wf_slow  # turn right to find wall

        elif self.leftfront_dist > d and self.front_dist < d and self.rightfront_dist > d:
            self.wall_following_state = "turn left"
            msg.angular.z = self.turning_speed_wf_fast

        elif (self.leftfront_dist > d and self.front_dist > d and self.rightfront_dist < d):
            if (self.rightfront_dist < 0.25):
                # Getting too close to the wall
                self.wall_following_state = "turn left"
                msg.linear.x = self.forward_speed
                msg.angular.z = self.turning_speed_wf_fast
            else:
                # Go straight ahead
                self.wall_following_state = "follow wall"
                msg.linear.x = self.forward_speed

        elif self.leftfront_dist < d and self.front_dist > d and self.rightfront_dist > d:
            self.wall_following_state = "search for wall"
            msg.linear.x = self.forward_speed
            msg.angular.z = -self.turning_speed_wf_slow  # turn right to find wall

        elif self.leftfront_dist > d and self.front_dist < d and self.rightfront_dist < d:
            self.wall_following_state = "turn left"
            msg.angular.z = self.turning_speed_wf_fast

        elif self.leftfront_dist < d and self.front_dist < d and self.rightfront_dist > d:
            self.wall_following_state = "turn left"
            msg.angular.z = self.turning_speed_wf_fast

        elif self.leftfront_dist < d and self.front_dist < d and self.rightfront_dist < d:
            self.wall_following_state = "turn left"
            msg.angular.z = self.turning_speed_wf_fast

        elif self.leftfront_dist < d and self.front_dist > d and self.rightfront_dist < d:
            self.wall_following_state = "search for wall"
            msg.linear.x = self.forward_speed
            msg.angular.z = -self.turning_speed_wf_slow  # turn right to find wall

        else:
            pass

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

    def initialmove(self):
        self.get_logger().info('In initialmove, move backwards')
        # publish to cmd_vel to move TurtleBot
        if initial_direction == "Back":
            self.get_logger().info("Going back")
        elif initial_direction == "Right":
            self.get_logger().info("Going right")
            self.rotatebot(90)
        elif initial_direction == "Left":
            self.get_logger().info("Going right")
            self.rotatebot(-90)
        elif initial_direction == "Front":
            self.get_logger().info("Going Forward")
            self.rotatebot(180)
        twist = Twist()
        twist.linear.x = -speedchange
        twist.angular.z = 0.0
        lrback = (self.laser_range[back_angles] < float(
            0.40)).nonzero()
        self.publisher_.publish(twist)
        while len(lrback[0]) <= 0:
            time.sleep(1)
            twist.linear.x = -speedchange
            twist.angular.z = 0.0
            rclpy.spin_once(self)
            lrback = (self.laser_range[back_angles] < float(
                0.40)).nonzero()
            self.publisher_.publish(twist)
        self.stopbot()
        self.rotatebot(-90)
        self.stopbot()

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
        global myoccdata
        ALTHRESH = 10
        # We will slightly fill in the contours to make them easier to detect
        DILATE_PIXELS = 3
        mapdata = myoccdata
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
        global myoccdata, current_lrleft, previous_lrleft, isTargetDetected, isDoneShooting
        try:
            # initialize variable to write elapsed time to file
            # contourCheck = 1
            rclpy.spin_once(self)
            while (self.laser_range.size == 0):
                print("Spin to get a valid startpoint, endpoint and lidar data")
                rclpy.spin_once(self)
            # initialize variable to write elapsed time to file
            contourCheck = 1
            start_time = time.time()
            # find direction with the largest distance from the Lidar,
            # rotate to that direction, and start moving
            self.initialmove()
            self.pick_direction()

            while rclpy.ok():
                if self.laser_range.size != 0:
                    """ if contourCheck and len(myoccdata) != 0:
                        print("Inside contourCheck:")
                        if self.closure():
                            self.stopbot()
                            print("Inside selfclosure contourcheck:")
                            # map is complete, so save current time into file
                            with open("maptime.txt", "w") as f:
                                f.write("Elapsed Time: " +
                                        str(time.time() - start_time))
                            contourCheck = 0
                            # save the map
                            cv2.imwrite('mazemap.png', myoccdata)
                            print("Map is complete!")
                            if isDoneShooting:
                                print("I'm done shooting and My map is complete")
                                break """
                    elapsed_time = time.time() - start_time
                    if elapsed_time > stopping_time_in_seconds:
                        print("5 minutes have passed")
                        break
                    # check distances in front of TurtleBot and find values less
                    # than stop_distance
                    lrfront = (self.laser_range[front_angles]
                               < float(stop_distance)).nonzero()
                    lrfrontleft = (self.laser_range[45] < float(
                        stop_distance)).nonzero()
                    lrleft = (self.laser_range[ninety_degrees_left_side_angles] > float(
                        stop_distance)).nonzero()
                    # current_lrleft = np.sum(
                    # self.laser_range[ninety_degrees_left_side_angles])
                    current_lrleft = self.laser_range[90]
                    self.get_logger().info('Distances front angles: %s' % str(lrfront))
                    print(current_lrleft)
                    # self.get_logger().info('Distances left angles: %s' % str(lrleft))
                    if not isTargetDetected:
                        self.pick_direction()
                    else:
                        self.stopbot()
                        while (not isDoneShooting):
                            print('In mover, target detected.')
                            rclpy.spin_once(self)
                        isTargetDetected = False
                    # # if the list is not empty
                    # # if current_lrleft > 0.25:
                    # if len(lrleft[0]) > 0:
                    #     self.stopbot()
                    #     self.get_logger().info("Now turning left")
                    #     self.turn_left()
                    #     # self.pick_direction()

                    # elif (len(lrfront[0]) > 0):
                    #     # stop moving
                    #     self.stopbot()
                    #     # find direction with the largest distance from the Lidar
                    #     # rotate to that direction
                    #     # start moving
                    #     self.pick_direction()

                # allow the callback functions to run
                # previous_lrleft = current_lrleft
                rclpy.spin_once(self)

        except Exception as e:
            print(e)

        # Ctrl-c detected
        finally:
            # stop moving
            self.stopbot()
            cv2.imwrite('mazemapfinally.png', myoccdata)


def main(args=None):
    rclpy.init(args=args)

    auto_nav = AutoNav()
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
