'''
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
                        self.pick_direction()
                # allow the callback functions to run
                rclpy.spin_once(self)

        except Exception as e:
            print(e)

        # Ctrl-c detected
        finally:
            # stop moving
            self.stopbot()
        '''

        #uncomment this if next option doesn't work
        '''
        print("find direction to go in")
        # find angle between direction vector and current direction
        direction_vector = (end[0]-start[0], end[1]-start[1])
        angle_i_want = angle_between((1, 0), direction_vector) 
        angle_i_want = np.degrees(angle_i_want) // 1
        self.angle_i_want = angle_i_want 

        print("angle_i_want")
        # print out next desired angle
        print(angle_i_want)

        # find the time i should travel
        local_dist = ((end[1]-start[1])**2 + (end[1]-start[1])**2)**0.5)*0.5
        time_to_take = local_dist / speedchange

        self.rotatebot(angle_i_want)
        # start moving
        twist = Twist()
        twist.linear.x = speedchange
        twist.angular.z = 0.0
        # not sure if this is really necessary, but things seem to work more
        # reliably with this
        time.sleep(1)
        self.publisher_.publish(twist)
        time.sleep(time_to_take)
        self.stopbot()
        '''

        '''
            if case == 3:
                start = self.unrotatedstart
                end = self.unrotatedgoal
                occdata = self.unrotatedoccdata
                msginfo = self.unrotatedmsginfo
                
                self.get_logger().info("Getting List of Paths")
                path = PyPiPathFinding(np.ndarray.tolist(occdata),start,end)

                def MapToWorld:
                    pass

                print(path)
                self.get_logger().info("Getting index 5 from List of Paths")
                end = path[5]

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

                pass
        '''