#!/usr/bin/env python3

from can.interfaces import seeedstudio
from subprocess import call
from .pid import PID
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Bool
import os
import can
import smbus2


class SteerAngle(Node):
    def __init__(self):
        super().__init__('steer_angle')
        self.get_logger().info('ROS2 steer_angle Node Started')
        self.setParam()

        self.cm = CAN_MESSAGES_PARAMETERS()
        self.angle = None
        self.wheel_angle_setpoint = 0
        self.wheel_angle = 0
        self.total_pos = 0
        self.prevout = 0
        self.auto_steer = False
        
        self.sample_time = 0.05

        pid_limit = (self.pid_limit * -1 , self.pid_limit)
        integral_limit = (self.pid_limit * -1 , self.pid_limit)
        self.pid = PID(Kp=self.kp,Kd=self.kd,Ki=self.ki,sample_time=self.sample_time, 
                       output_limits=pid_limit ,integral_limits=integral_limit, differential_on_measurement=False)


        self.timer = self.create_timer(0.05, self.update) # update time 0.05 second

        self.sub_cam_angle = self.create_subscription(Int32, 'angle', self.subs_callback_angle, 1)
        self.sub_auto_steer= self.create_subscription(Bool, 'auto_steer', self.subs_callback_auto_steer, 1)

        self.ads = ADS1115()
        self.initialize_canbus()
        self.motor_reset_position()
        self.get_logger().info(f'Motor init: {self.motor_com_succes}')
        self.subs_callback_auto_steer(Bool(data=False))

    def subs_callback_auto_steer(self, msg):
        self.auto_steer = bool(msg.data)
        if(self.auto_steer == False):
            self.pid.reset()
            self.total_pos = 0
            self.prevout = 0
            self.motor_pos = 0
            self.wheel_average_status = False
            self.kill_motor()
        else:
            self.motor_reset_position()
            self.active_motor()

    def subs_callback_angle(self, msg):
        self.angle = int(msg.data)

    def update(self):
        self.kp = self.get_parameter('kp').get_parameter_value().integer_value
        self.ki = self.get_parameter('ki').get_parameter_value().integer_value
        self.kd = self.get_parameter('kd').get_parameter_value().integer_value
        self.pid_limit = self.get_parameter('pid_limit').get_parameter_value().integer_value

        self.pid.Kp = self.kp
        self.pid.Ki = self.ki
        self.pid.Kd = self.kd
        self.pid.sample_time = self.sample_time
        self.pid.output_limits = (self.pid_limit * -1 , self.pid_limit)
        self.pid.integral_limits = (self.pid_limit * -1 , self.pid_limit)

        wheel_p = 1
        self.wheel_angle = self.ads.read_channel()
        self.wheel_angle = self.wheel_angle - self.wheel_angle_center
        self.wheel_angle = round(self.wheel_angle * wheel_p)
        
        if(self.auto_steer == False):
            self.get_logger().info(f'AutoSteer: {self.auto_steer} , wheel_angle: {self.wheel_angle}')
            return -1
        
        if(self.angle == None):
            return -1

        self.wheel_angle_setpoint = int(self.angle)
        
        self.calculate_motor_speed()

    def calculate_motor_speed(self):
        self.pid.setpoint = self.wheel_angle_setpoint

        limit = 20
        if self.pid.setpoint > limit:
            self.pid.setpoint = limit
        elif self.pid.setpoint < -limit:
            self.pid.setpoint = -limit

        pid_output = int(self.pid.__call__(input_=self.wheel_angle, dt=self.sample_time))

        #50ms süre olup olmadığına göre mesaj gönderiyoruz
        if(pid_output != None):
            self.prevout = pid_output
            self.motor_pos = pid_output - self.total_pos
            self.send_speed()
            self.total_pos += self.motor_pos
            #if not (self.motor_pos > 10 or self.motor_pos < -10):
            err = self.wheel_angle_setpoint - self.wheel_angle
            str = f'pid_error= {err}, pid_output= {pid_output}, motor_pos= {self.motor_pos}, total_pos= {self.total_pos}'
            # self.get_logger().info(str)

    def setParam(self):
        self.declare_parameter("kp", 1500)
        self.declare_parameter("ki", 500)
        self.declare_parameter("kd", 0)
        self.declare_parameter("pid_limit", 30000)
        self.declare_parameter("wheel_angle_limit_right", 200)
        self.declare_parameter("wheel_angle_limit_left", 250)
        self.declare_parameter("wheel_angle_center", 225)

        self.kp = self.get_parameter('kp').get_parameter_value().integer_value
        self.ki = self.get_parameter('ki').get_parameter_value().integer_value
        self.kd = self.get_parameter('kd').get_parameter_value().integer_value
        self.pid_limit = self.get_parameter('pid_limit').get_parameter_value().integer_value

        self.wheel_angle_limit_right = self.get_parameter('wheel_angle_limit_right').get_parameter_value().integer_value
        self.wheel_angle_limit_left = self.get_parameter('wheel_angle_limit_left').get_parameter_value().integer_value
        self.wheel_angle_center = self.get_parameter('wheel_angle_center').get_parameter_value().integer_value


    def motor_reset_position(self):
        reset_pos_message = can.Message(arbitration_id=0x06000001, is_extended_id=True, 
                                        data=[0x23, 0x0C, 0x20, 0x09, 0x00, 0x00, 0x00, 0x00])
        self.usb_can_bus.send(reset_pos_message)
        activate_message = can.Message(arbitration_id=self.cm.CAN_ID, is_extended_id=True,data=self.cm.MOTOR_ACTIVATE_MESSAGE)
        self.usb_can_bus.send(activate_message)
        print("motor reset position")

    def send_speed(self):
        if self.motor_pos == 0:
            return -1
        if self.motor_pos >= 0:
            #direksiyon sola dönüyor
            self.Rotation = False
        else:
            #direksiyon sağa dönüyor
            self.Rotation = True

        message1, message2, message3, message4 = self.header_message()
        message5, message6 = self.main_message()
        message7, message8 = self.end_message()

        messages = message1 + message2 + message3 + message4 + message5 + message6 + message7 + message8
        finally_message = can.Message(arbitration_id=self.cm.CAN_ID, is_extended_id=True, data=messages)

        if self.motor_com_succes:
            self.usb_can_bus.send(finally_message)

       

    def motor_zero_message(self):
        self.motor_pos = 0
        self.send_speed()

    def header_message(self):
        header_1 = "23"
        header_2 = "02"
        header_3 = "20"
        header_4 = "01"
        message1 = bytes.fromhex(header_1)
        message2 = bytes.fromhex(header_2)
        message3 = bytes.fromhex(header_3)
        message4 = bytes.fromhex(header_4)

        return message1, message2, message3, message4

    def main_message(self):
        parse_data = divmod(abs(self.motor_pos), 0x100)
        high_data, low_data = bytes(parse_data)
        high_data = hex(high_data)
        low_data = hex(low_data)

        if high_data[:2] == '0x' or low_data[:2] == '0x':
            high_data = high_data[2:]
            low_data = low_data[2:]
        else:
            pass

        if self.motor_pos < 0:
            high_data = int("FF", base=16) - int(high_data, base=16)
            low_data = int("FF", base=16) - int(low_data, base=16)
            high_data = hex(high_data)
            low_data = hex(low_data)

            if high_data[:2] == '0x' or low_data[:2] == '0x':
                high_data = high_data[2:]
                low_data = low_data[2:]
            else:
                pass

        elif self.motor_pos >= 0:
            pass

        if int(high_data, base=16) <= int("F", base=16):
            high_data = "0" + high_data
        else:
            pass

        if int(low_data, base=16) <= int("F", base=16):
            low_data = "0" + low_data
        else:
            pass

        message5 = bytes.fromhex(high_data)
        message6 = bytes.fromhex(low_data)

        return message5, message6

    def end_message(self):
        if  not self.Rotation:
            header_7 = "00"
            header_8 = "00"
            message7 = bytes.fromhex(header_7)
            message8 = bytes.fromhex(header_8)
        elif self.Rotation:
            header_7_ff = "FF"
            header_8_ff = "FF"
            message7 = bytes.fromhex(header_7_ff)
            message8 = bytes.fromhex(header_8_ff)
        else:
            header_7 = "00"
            header_8 = "00"
            message7 = bytes.fromhex(header_7)
            message8 = bytes.fromhex(header_8)
        return message7, message8

    def initialize_canbus(self):
        try:
            if os.path.exists('/dev/ttyUSB0'):
                self.channel = '/dev/ttyUSB0'

            self.usb_can_bus = seeedstudio.SeeedBus(channel=self.channel, baudrate=self.cm.baudrate, timeout=self.cm.time_out,
                                                    frame_type=self.cm.frame_type, operation_mode=self.cm.operation_mode,
                                                    bitrate=self.cm.bitrate)
            self.received_message = self.usb_can_bus.recv()
            #activate_message = can.Message(arbitration_id=self.cm.CAN_ID, is_extended_id=True,
            #                               data=self.cm.MOTOR_ACTIVATE_MESSAGE)
            #self.usb_can_bus.send(activate_message)
            self.motor_com_succes = True
            #print(self.usb_can_bus)
        except:
            self.motor_com_succes = False

        return self.motor_com_succes

    def succes_check(self):
        return self.motor_com_succes

    def kill_motor(self):
        self.usb_can_bus.send(
            can.Message(arbitration_id=self.cm.CAN_ID, is_extended_id=True, data=self.cm.MOTOR_DEACTIVE_MESSAGE))

    def active_motor(self):
        self.usb_can_bus.send(
            can.Message(arbitration_id=self.cm.CAN_ID, is_extended_id=True, data=self.cm.MOTOR_ACTIVATE_MESSAGE))
        

class ADS1115:
    def __init__(self):
        self.bus = smbus2.SMBus(1)
        self.ads_address = 0x48
        self.set_data = [0xD0, 0x83]    # gain ve sps buradan ayarlanacak
        self.setup()

    def setup(self):
        self.bus.write_i2c_block_data(self.ads_address, 0x01, self.set_data)

    def read_channel(self):
        data = self.bus.read_i2c_block_data(self.ads_address, 0x00, 2)
        raw_adc = data[0] * 256 + data[1]
        scaled_sensor_val = int((raw_adc/32767)*720)
        return scaled_sensor_val

class CAN_MESSAGES_PARAMETERS:
    HEARTBEAT_ID = 0x07000001
    CAN_ID =   0x06000001
    QUERY_ID = 0x05800001
    PORT_0 = '/dev/ttyUSB0'
    PORT_1 = '/dev/ttyUSB1'
    baudrate = 2000000
    bitrate = 250000
    time_out = 0.1
    frame_type = 'STD'
    operation_mode = 'normal'
    MOTOR_ACTIVATE_MESSAGE = b'\x23\x0D\x20\x01\x00\x00\x00\x00'
    MOTOR_DEACTIVE_MESSAGE = b'\x23\x0C\x20\x01\x00\x00\x00\x00'
    MOTOR_ZERO_POINT = b'\x23\x02\x20\x01\x00\x00\x00\x00'
    ENCODER_QUERY = b'\x40\x04\x21\x01\x00\x00\x00\x00'


def main(args=None):

    if os.path.exists("/dev/ttyUSB0"):
        call("echo " + str("moveon") + " | sudo -S chmod 666 /dev/ttyUSB0", shell=True)
        # cmd = "sudo chmod 666 /dev/ttyUSB1"
        # subprocess.Popen(cmd.split(), stdout=subprocess.PIPE)
        print("Port 0 opened.")
    elif os.path.exists("/dev/ttyUSB1"):
        call("echo " + str("moveon") + " | sudo -S chmod 666 /dev/ttyUSB1", shell=True)
        # cmd = "sudo chmod 666 /dev/ttyUSB1"
        # subprocess.Popen(cmd.split(), stdout=subprocess.PIPE)
        print("Port 1 opened.")

    rclpy.init(args=args)

    steer_angle = SteerAngle()

    rclpy.spin(steer_angle)

    rclpy.shutdown()

if __name__ == '__main__':
    main()