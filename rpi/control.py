import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile 
from turtlebot3_msgs.msg._sensor_state import SensorState
from std_msgs.msg import Bool, Float32MultiArray, MultiArrayDimension, Int8, Float32
from gpiozero import LightSensor, LED
import time

import numpy as np
import busio
import board
import adafruit_amg88xx

from . import i2c
from .flywheel import fire

qos = QoSProfile(depth=10)

class Sensors(Node):

    def __init__(self):
        super().__init__('sensor_publisher')
        self.fired = False
        self.pn532 = i2c.PN532_I2C(debug=False, reset=20, req=16)
        ic, ver, rev, support = self.pn532.get_firmware_version()
        print('Found PN532 with firmware version: {0}.{1}'.format(ver, rev))
        self.pn532.SAM_configuration()

        i2c_port = busio.I2C(board.SCL, board.SDA)
        self.amg = adafruit_amg88xx.AMG88XX(i2c_port)

        self.ldr_sensor = LightSensor(18)
        self.led = LED(16)
        self.led.on

        self.moving_ave = np.zeros(10, np.single)

        self.nfc_publisher_ = self.create_publisher(Bool, 'nfc' , qos)
        self.thermal_publisher_ = self.create_publisher(Float32MultiArray, 'thermal', qos)
        self.ldr_publisher = self.create_publisher(Float32, 'ldr', qos)
        self.button_publisher = self.create_publisher(Bool, 'button', qos)
        self.nfc_timer = self.create_timer(0.05, self.nfc_timer_callback)
        self.thermal_timer = self.create_timer(0.1, self.thermal_timer_callback)
        self.ldr_timer = self.create_timer(0.05, self.ldr_timer_callback)

        self.button_subsciption = self.create_subscription(
            SensorState, 
            'sensor_state',
            self.sensor_callback,
            qos_profile_sensor_data
        )
        self.button_subsciption

        self.shoot_subscription = self.create_subscription(
            Int8,
            'shoot',
            self.shoot_callback,
            qos
        )
        self.shoot_subscription
    
    def nfc_timer_callback(self):
        msg=Bool()
        uid = self.pn532.read_passive_target(timeout=0.05)
        if uid is None:
            msg.data=False
            #self.get_logger().info('Publishing: "%s"' % "False")
        else:
            msg.data=True
            #self.get_logger().info('Publishing: "%s"' % "True")
        self.nfc_publisher_.publish(msg)

    def thermal_timer_callback(self):
        try:
            msg = Float32MultiArray()
            #self.get_logger().info('Publishing thermal')
            msg.data = np.reshape(self.amg.pixels, 64).tolist()
            # This is almost always zero there is no empty padding at the start of your data
            msg.layout.data_offset = 0 

            # create two dimensions in the dim array
            msg.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]

            # dim[0] is the vertical dimension of your matrix
            msg.layout.dim[0].label = "y"
            msg.layout.dim[0].size = 8
            msg.layout.dim[0].stride = 64
            # dim[1] is the horizontal dimension of your matrix
            msg.layout.dim[1].label = "x"
            msg.layout.dim[1].size = 8
            msg.layout.dim[1].stride = 8
            self.thermal_publisher_.publish(msg)
        except Exception as e:
            self.get_logger().error('%s' % e)

    def ldr_timer_callback(self):
        msg = Float32()
        self.moving_ave = np.append(self.moving_ave, [self.ldr_sensor.value])
        self.moving_ave = np.delete(self.moving_ave, 0)
        ave = np.mean(self.moving_ave)
        msg.data = ave
        self.ldr_publisher.publish(msg)

    def sensor_callback(self, msg):
        self.button_press = msg.button
        if self.button_press != 0:
            send = Bool()
            send.data = True
            self.button_publisher.publish(send)

    def shoot_callback(self, msg):
        self.flywheel_spd = msg.data
        if self.flywheel_spd <= 100 and self.flywheel_spd > 0:
            fire(3, self.flywheel_spd)
            self.fired = False

def main(args=None):
    rclpy.init(args=args)
    
    sensor = Sensors()
    rclpy.spin(sensor)

    sensor.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
