
import rclpy
from rclpy.node import Node
import cv2
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile 

from std_msgs.msg import Bool, MultiArrayDimension, Float32MultiArray, Float32
from geometry_msgs.msg import Twist 
from sensor_msgs.msg import LaserScan, BatteryState
from nav_msgs.msg import Odometry, OccupancyGrid

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import cmath
import math
import numpy as np
from .astar import *

def euler_from_quartenion(x, y, z, w):
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
    
    return roll_x, pitch_y, yaw_z

    

qos = QoSProfile(depth=10)

class Robot(Node):

    def __init__(self):
        super().__init__('control')
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            qos)
        self.odom_subscription
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription
        self.occ_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            qos_profile_sensor_data)
        self.occ_subscription
        
        self.spd_publisher = self.create_publisher(Twist, 'cmd_vel', qos)
        self.coor_publisher = self.create_publisher(Float32MultiArray, 'target', qos)
        self.complete_publisher = self.create_publisher(Bool, 'map_complete', qos)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.occdata = np.nan
        self.confirmed = []
        self.wall = []
        self.unexplored = []
        self.kernel = np.ones((3, 3), np.uint8)
        self.map_frame_origin = [0, 0]
        self.process = []
    
    def occ_callback(self, msg):
        self.reso = msg.info.resolution
        msgdata = np.array(msg.data)
        self.map_frame_origin[0], self.map_frame_origin[1] = int(-msg.info.origin.position.x/self.reso), int(-msg.info.origin.position.y/self.reso)
        self.grey = np.full((msg.info.height, msg.info.width), 125, np.uint8)
        self.occdata = np.uint8(msgdata.reshape(msg.info.height,msg.info.width))

    def odom_callback(self, msg):
        orientation_quat  = msg.pose.pose.orientation
        relative_pos = msg.pose.pose.position
        self.curr_x, self.curr_y = relative_pos.x, relative_pos.y
        self.roll, self.pitch, self.yaw = euler_from_quartenion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)
        #self.get_logger().info('Orientation (x, y, z, w): %f, %f, %f, %f' % (orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w))
        #self.get_logger().info('Orientation (roll, pitch, yaw): %f, %f, %f' % (self.roll, self.pitch, self.yaw))
    
    def scan_callback(self, msg):
        self.laser_range = np.array(msg.ranges)
        # replace 0's with nan
        self.laser_range[self.laser_range==0] = np.nan
    
    def run(self):
        #print(self.occdata)
        while type(self.occdata) == float or len(self.occdata)==0:
            print(self.occdata)
            rclpy.spin_once(self)
        print(self.occdata)
        start = self.get_clock().now().to_msg()
        while rclpy.ok():
            #create bitmap of wall
            self.wall = cv2.inRange(self.occdata, 55, 100)
            
            #create bitmap of explored region 
            self.confirmed = cv2.inRange(self.occdata, 0, 40)
            
            #Close any smalls gaps in the wall
            self.wall = cv2.dilate(self.wall, self.kernel)
            self.wall = cv2.erode(self.wall, self.kernel)
            self.wall = cv2.dilate(self.wall, self.kernel, iterations=1)

            inv_wall = cv2.bitwise_not(self.wall)
            
            #find the edges of the ex[plored region
            self.unexplored = cv2.bitwise_or(self.grey, self.confirmed)
            edges = cv2.Canny(self.unexplored, 0, 255)
            self.unexplored = cv2.bitwise_or(self.unexplored, self.unexplored, mask = inv_wall)
            
            #obtain frontier by identifying edges of explored regions that are not bordered by a wall
            self.frontier = cv2.bitwise_and(cv2.bitwise_not(self.wall), edges)
            
            #thicken frontiers 
            self.frontier = cv2.dilate(self.frontier, np.ones((2, 2), np.uint8))

            #check if map has been completed
            wall_contours, hierarchy = cv2.findContours(self.wall, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            lc = len(wall_contours)

            cAL = np.zeros((lc, 2))
            for i in range(lc):
                cAL[i, 0] = cv2.contourArea(wall_contours[i])
                cAL[i, 1] = cv2.arcLength(wall_contours[i], True)

            # closed contours tend to have a much higher area to arc length ratio,
            # so if there are no contours with high ratios, we can safely say
            # there are no closed contours
            cALratio = cAL[:, 0]/cAL[:, 1]
            # rospy.loginfo('Closure: %s', str(cALratio))
            now = self.get_clock().now().to_msg()
            if np.any(cALratio > 8) or now.sec - start.sec > 270:
                print('True')
                msg = Bool()
                msg.data = True
                self.complete_publisher.publish(msg)
                try:
                    cv2.imwrite('occupancy_grid.jpg', self.occdata)
                except:
                    np.savetxt('grid_complete', self.occdata)

            #find frontier
            contours, hierarchy = cv2.findContours(self.frontier, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            all_pts = []
            if len(contours) > 0:
                for i in range(len(contours)):
                    cnt = contours[i]
                    if (cv2.contourArea(cnt) <5):
                        continue
                    M = cv2.moments(cnt)
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    self.unexplored = cv2.circle(self.unexplored, (cx, cy), 5, 150, 2)
                    #xr = (self.map_frame_origin[0]-cx)*self.reso
                    #yr = (self.map_frame_origin[1]-cy)*self.reso
                    pt=[np.array([cx,cy])]
                    if len(all_pts)>0:
                        all_pts=np.vstack([all_pts,pt])
                    else:       
                        all_pts=pt
            try:
                now = rclpy.time.Time()
                trans = self.tf_buffer.lookup_transform('map', 'base_link', now)
                self.curr_x, self.curr_y = self.map_frame_origin[0]+int(trans.transform.translation.x/self.reso), self.map_frame_origin[1]+int(trans.transform.translation.y/self.reso)
                if np.abs(trans.transform.translation.x) > 20 or np.abs(trans.transform.translation.y) > 20:
                    raise (ConnectivityException)
                x, y, relative_yaw = euler_from_quartenion(trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w)
                # point_start = [self.curr_x, self.curr_y]
                # init_points = [point_start.copy()]
                # while np.any([(val == 0 or val == 125) for val in self.unexplored[point_start[1]-8:point_start[1]+9:1, point_start[0]-8:point_start[0]+9:1].flatten().tolist()]):
                #     ave_x = 0
                #     ave_y = 0
                #     for n, val in enumerate(self.unexplored[point_start[1]-8:point_start[1]+9:1, point_start[0]-8:point_start[0]+9:1].flatten().tolist()):
                #         val_x = n%17
                #         val_y = n//17
                #         if val==0 or val == 125:
                #             ave_x += val_x - 8
                #             ave_y += val_y - 8
                #     if ave_x == 0 or ave_y == 0:
                #         break
                #     if np.abs(ave_x) > np.abs(ave_y):
                #         move_dir = (-int(ave_x/np.abs(ave_x)), -int(ave_y/(np.abs(ave_x))))
                #     else:
                #         move_dir = (-int(ave_x/np.abs(ave_y)), -int(ave_y/(np.abs(ave_y))))
                #     point_start[0] += move_dir[0]
                #     point_start[1] += move_dir[1]
                #     print(point_start)
                #     if [point_start] not in init_points:
                #         init_points += [point_start.copy()]
                #     else:
                #         break
                # if len(init_points) > 2:
                #     msg = Float32MultiArray()
                #     #print(init_points)
                #     msg.data, line = route_parser(init_points, -relative_yaw)
                #     #print(line)
                #     self.coor_publisher.publish(msg)
                #     for i in range(2, len(line), 2):
                #         self.unexplored = cv2.line(self.unexplored, (line[i-2], line[i-1]), (line[i], line[i+1]), 10, 1)
                #     print(msg.data)
                #if len(contours) > 0:
                    # all_pts = sorted(all_pts, key=lambda x: np.sqrt((self.curr_x-x[0])**2+ (self.curr_y-x[1])**2), reverse=False)
                    # path = Astar(np.array(self.unexplored))
                    # for pts in all_pts:
                    #     points = path.run([self.curr_x, self.curr_y], [80, 120])
                    #     if points != None:
                    #         msg = Float32MultiArray()
                    #         msg.data, line = route_parser(points, -relative_yaw)
                    #         self.coor_publisher.publish(msg)
                    #         for i in range(2, len(line), 2):
                    #             self.unexplored = cv2.line(self.unexplored, (line[i-2], line[i-1]), (line[i], line[i+1]), 10, 1)
                    #         print(msg.data)
                    #         break
                self.unexplored = cv2.arrowedLine(self.unexplored, (self.curr_x, self.curr_y), (self.curr_x+int(10*math.cos(relative_yaw)), self.curr_y + int(10*math.sin(relative_yaw))), 175, 2, tipLength = 0.2)

            except (LookupException, ExtrapolationException):
                self.get_logger().info('Exception transform')
                self.process = self.unexplored.copy()
            except (ConnectivityException):
                self.get_logger().info('Too big')
                

            if len(self.wall) != 0:
                cv2.imshow('unexplored', self.unexplored)
            key = cv2.waitKey(1)
            if key == ord('q'):
                break
            rclpy.spin_once(self)
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    robot = Robot()
    robot.run()
    robot.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
