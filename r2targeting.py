# TO DO
# Set up the servo motor code ... copy from rpi
# Integrate it so that auto_nav stops when this code is running and resumes when code stops running

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8, Bool, String
import time
import busio
import board
import adafruit_amg88xx
import RPi.GPIO as GPIO

# constants
rotatechange = 0.1
speedchange = 0.05
detecting_threshold = 32.0
firing_threshold = 35.0

message_sent = 'Not Detected'

# Set up Thermal Camera
i2c = busio.I2C(board.SCL, board.SDA)
amg = adafruit_amg88xx.AMG88XX(i2c)


class ThermalCamera(Node):
    def __init__(self):
        super().__init__('thermalcamera')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # Set up publisher 'targeting_status' to communicate with wallfollower
        self.publisher_targeting = self.create_publisher(
            String, 'targeting_status', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    # targeting_status callback function to stop wallfollower logic when target is detected
    def timer_callback(self):
        global message_sent
        msg = String()
        msg.data = message_sent
        self.publisher_targeting.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

    def stopbot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

    def find_target(self):

        # See if target found
        global message_sent
        target_found = False

        # -------------------------------------------------------------------------- #
        # While target has not been found, run this code to check if target is found #
        # -------------------------------------------------------------------------- #
        while not target_found:
            detected = False

            for row in amg.pixels:
                print('[', end=" ")
                for temp in row:
                    if temp > detecting_threshold:
                        detected = True
                        target_found = True
                    print("{0:.1f}".format(temp), end=" ")
                print("]")
                print("\n")

            if detected == True:
                # Communicate with wallfollower to stop working
                message_sent = 'Detected'
                self.timer_callback()
                print(" ")
                print("DETECTED!!")
                print("]")
                print("\n")
                time.sleep(1)

        # If target is found, stop movement
        self.stopbot()

        return True

    def centre_target(self):
        # ----------------------------------------------------------- #
        # Adjust the servo and robot until high temp is in the centre #
        # ----------------------------------------------------------- #

        # Centre the target in the robot's vision
        GPIO.setmode(GPIO.BCM)
        horizontally_centered = False
        vertically_centered = False
        centered = False

        while not centered:
            screen = amg.pixels
            max_row = 0
            max_column = 0
            max_value = 0.0
            for row in range(len(screen)):
                for column in range(len(screen[row])):
                    current_value = screen[row][column]
                    if current_value > max_value:
                        max_row = row
                        max_column = column
                        max_value = current_value

            if not horizontally_centered:
                # centre max value between row 3 and 4
                if max_column < 3:
                    # spin it anti-clockwise
                    twist = Twist()
                    twist.linear.x = 0.0
                    twist.angular.z = rotatechange
                    time.sleep(1)
                    self.publisher_.publish(twist)
                    time.sleep(1)
                elif max_column > 4:
                    # spin it clockwise
                    twist = Twist()
                    twist.linear.x = 0.0
                    twist.angular.z = -1 * rotatechange
                    time.sleep(1)
                    self.publisher_.publish(twist)
                    time.sleep(1)
                else:
                    horizontally_centered = True

                self.stopbot()

            if horizontally_centered and not vertically_centered:
                # centre max value between row 3 and 4
                if max_row < 3:
                    # shift the servo up by 5 degrees (limit:0)
                    try:
                        servo_pin = 4

                        GPIO.setup(servo_pin, GPIO.OUT)

                        p = GPIO.PWM(servo_pin, 50)

                        p.start(2.5)

                        degree = 0

                        servo_value = degree/90 * 5 + 2.5
                        p.ChangeDutyCle(servo_value)
                        time.sleep(1)
                    except:
                        p.stop()
                        GPIO.cleanup()
                    finally:
                        p.stop()
                        GPIO.cleanup()

                elif max_row > 4:
                    # shift the servo down by 5 degrees (limit: 20)
                    try:
                        servo_pin = 4

                        GPIO.setup(servo_pin, GPIO.OUT)

                        p = GPIO.PWM(servo_pin, 50)

                        p.start(2.5)

                        degree = 20

                        servo_value = degree/90 * 5 + 2.5
                        p.ChangeDutyCle(servo_value)
                        time.sleep(1)
                    except:
                        p.stop()
                        GPIO.cleanup()
                    finally:
                        p.stop()
                        GPIO.cleanup()

                else:
                    vertically_centered = True

            if horizontally_centered and vertically_centered:
                centered = True

            return True

    def move_to_target(self):
        # move to the object in increments
        twist = Twist()
        twist.linear.x = speedchange
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        time.sleep(1)
        self.stopbot()

    def firing_time(self):
        screen = amg.pixels
        for row in [3, 4]:
            for column in [3, 4]:
                if screen[row][column] > firing_threshold:
                    return True

    def targetting(self):
        global message_sent

        # find the target
        self.find_target()

        # centre the target
        self.centre_target()

        # --------------------------------------------------- #
        # Now it is centered, start moving towards the target #
        # --------------------------------------------------- #

        while not self.firing_time():
            self.move_to_target()
            self.centre_target()

        # ----------------------------- #
        # Now the bot can fire the ball #
        # ----------------------------- #

        GPIO.setmode(GPIO.BCM)

        # 11 , 13 Input 1,2
        GPIO.setup(17, GPIO.OUT)
        GPIO.setup(27, GPIO.OUT)
        self.get_logger().info("Setup the DC")

        # 12 Enable
        GPIO.setup(18, GPIO.OUT)
        pwm = GPIO.PWM(18, 100)
        pwm.start(0)
        time.sleep(5)

        # Spin Backwards Continuously
        GPIO.output(17, True)
        GPIO.output(27, False)
        pwm.ChangeDutyCycle(75)
        GPIO.output(18, True)
        self.get_logger().info("Start the DC")

        # Start the Stepper Motor 2 seconds later

        # Wait for 2 seconds
        time.sleep(5)

        # Set up the Stepper Pins
        control_pins = [26, 19, 13, 6]
        for pin in control_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, 0)

        halfstep_seq = [
            [1, 0, 0, 0],
            [1, 1, 0, 0],
            [0, 1, 0, 0],
            [0, 1, 1, 0],
            [0, 0, 1, 0],
            [0, 0, 1, 1],
            [0, 0, 0, 1],
            [1, 0, 0, 1]]

        self.get_logger().info("Started the Stepper")
        # Start Spinning the Stepper
        for i in range(512):
            for halfstep in range(8):
                for pin in range(4):
                    GPIO.output(control_pins[pin], halfstep_seq[halfstep][pin])
                time.sleep(0.001)

        # -------------- #
        # Do the cleanup #
        # -------------- #
        # Send message that the target has finished shooting
        message_sent = 'Done'
        self.timer_callback()

        # Stop the DC Motor
        GPIO.output(18, False)
        pwm.stop()
        self.get_logger().info("Stopped the DC Motor")

        # Cleanup all GPIO
        GPIO.cleanup()
        self.get_logger().info("Cleaned up GPIO")


def main(args=None):
    rclpy.init(args=args)

    thermalcamera = ThermalCamera()
    thermalcamera.targetting()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    thermalcamera.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
