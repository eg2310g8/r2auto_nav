# TO DO 
# Set up the servo motor code ... copy from rpi
# Integrate it so that auto_nav stops when this code is running and resumes when code stops running

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8, Bool
import time
import busio
import board
import adafruit_amg88xx
import RPi.GPIO as GPIO

# constants
rotatechange = 0.1
speedchange = 0.05
detecting_threshold = 30.0
firing_threshold = 40.0
correct_servo_angle = 20

# Set up Thermal Camera
i2c = busio.I2C(board.SCL, board.SDA)
amg = adafruit_amg88xx.AMG88XX(i2c)

# Reads thermal camera info, and tells Servo, Motor, and Stepper what to do

class ThermalCamera(Node):
    def __init__(self):
        super().__init__('thermalcamera')
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
        #self.publisher_servoangle = self.create_publisher(Int8,'servo_angle',10)
        #self.publisher_startmotor = self.create_publisher(Bool,'start_motor',10)
        #self.publisher_startstepper = self.create_publisher(Bool,'start_stepper',10)

    def stopbot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

    def find_target(self):
        # See if target found
        target_found = False

        # -------------------------------------------------------------------------- #
        # While target has not been found, run this code to check if target is found #
        # -------------------------------------------------------------------------- #
        while not target_found:
            detected = False

            for row in amg.pixels:
                print('[', end = " ")
                for temp in row:
                    if temp > detecting_threshold:
                        detected = True
                        target_found = True
                    print("{0:.1f}".format(temp), end = " ")
                print("]")

                '''
                # Pad to 1 decimal place
                print(["{0:.1f}".format(temp) for temp in row])
                print("")
                '''
            print("\n")
            if detected == True:
                print(" ")
                print("DETECTED!!")
                print("]")
            print("\n")
            time.sleep(1)

        # Tell AutoNav to stop working

        # If target is found, stop movement
        self.stopbot()

        return True

    def centre_target(self):
        # ----------------------------------------------------------- #
        # Adjust the servo and robot until high temp is in the centre #
        # ----------------------------------------------------------- #
        
        # Centre the target in the robot's vision
        GPIO.setmode(GPIO.BCM)
        
        centered = False
        horizontally_centered = False
        vertically_centered = False

        while not centered:
            screen = amg.pixels
            max_row = 0
            max_column = 0
            max_value = 0.0
            for row in range(len(screen)):
                for column in range(len(row)):
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
                    pass

                elif max_row > 4:
                    # shift the servo down by 5 degrees (limit: 20)
                    pass
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
        for row in [3,4]:
            for column in [3,4]:
                if screen[row][column] > firing_threshold:
                    return True


    
    def targetting(self):

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

        # Start the DC Motor

        GPIO.setmode(GPIO.BOARD)

        # 11 , 13 Input 1,2
        GPIO.setup(11, GPIO.OUT)
        GPIO.setup(13, GPIO.OUT)

        # 12 Enable
        GPIO.setup(12, GPIO.OUT)
        pwm = GPIO.PWM(12,100)
        pwm.start(0)
        sleep(1)

        # Spin Backwards Continuously
        GPIO.output(11,False)
        GPIO.output(13,True)
        pwm.ChangeDutyCycle(100)
        GPIO.output(12,True)

        # Start the Stepper Motor 2 seconds later

        # Wait for 2 seconds
        time.sleep(2)

        # Set up the Stepper Pins
        control_pins = [37,35,33,31]
        for pin in control_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, 0)

        halfstep_seq = [
                [1,0,0,0],
                [1,1,0,0],
                [0,1,0,0],
                [0,1,1,0],
                [0,0,1,0],
                [0,0,1,1],
                [0,0,0,1],
                [1,0,0,1]]

        # Start Spinning the Stepper
        for i in range(512):
            for halfstep in range(8):
                for pin in range(4):
                    GPIO.output(control_pins[pin], halfstep_seq[halfstep][pin])
                time.sleep(0.001)
        
        # -------------- #
        # Do the cleanup #
        # -------------- #

        # Stop the DC Motor
        GPIO.output(12,False)
        pwm.stop()

        # Cleanup all GPIO
        GPIO.cleanup()




            
            

        
        
        	


def main(args=None):
    rclpy.init(args=args)

    thermalcamera = ThermalCamera()
    thermalcamera.targetting()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mover.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()