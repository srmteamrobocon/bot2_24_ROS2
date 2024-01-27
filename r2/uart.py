import rclpy
import serial
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
import math
from threading import Lock

class Uart(Node):
    def __init__(self):
        super().__init__('uart_node')

        self.ROBOT_WIDTH = 0.229
        self.ROBOT_LENGTH = 0.550
        self.WHEEL_RADIUS = 0.127
        self.WHEEL_DIS_FROM_CENTER = 0.229
        self.WHEEL_GEOMETRY = 0.550 + 0.229  # Height and width in meters

        self.pico_pwm = 254
        self.max_pwm = 20 / 100 * self.pico_pwm

        self.loop_rate=0.020 # miliseconds
        self.MotorPWM:[int,int,int,int]=[0,0,0,0] 


        self.mutex = Lock()
        self.ser = serial.Serial('/dev/ttyACM0', 115200) 
        self.uart_timer=self.create_timer(self.loop_rate,self.uart_timer_callback)
        self.subscription = self.create_subscription(Twist,'piRobot/cmd_vel',self.twist_callback,10)
        # self.encoder=self.create_publis++++++++++++++++++++++++++++++++++++++++her(Motor_encoder,'/drive/raw_encoder',10)

        
    def twist_callback(self, msg):
        Vx = msg.linear.x
        # Vy = msg.linear.y
        Vy = msg.angular.z
        # omega = msg.angular.z
        omega = msg.linear.y

        self.MotorPWM[0] = (Vx - Vy - self.WHEEL_GEOMETRY * omega) / self.WHEEL_RADIUS

        self.MotorPWM[1] = (Vx + Vy + self.WHEEL_GEOMETRY * omega) / self.WHEEL_RADIUS

        self.MotorPWM[2] =  (Vx + Vy - self.WHEEL_GEOMETRY * omega) / self.WHEEL_RADIUS

        self.MotorPWM[3] = (Vx - Vy + self.WHEEL_GEOMETRY * omega) / self.WHEEL_RADIUS


        # self.MotorPWM[0] = (Vx - Vy - self.WHEEL_DIS_FROM_CENTER * omega) / self.WHEEL_RADIUS  # Forward left

        # self.MotorPWM[1] = (Vx + Vy + self.WHEEL_DIS_FROM_CENTER * omega) / self.WHEEL_RADIUS # Forward right
        
        # self.MotorPWM[2] = (Vx + Vy - self.WHEEL_DIS_FROM_CENTER * omega) / self.WHEEL_RADIUS # backward left
        
        # self.MotorPWM[3] = (Vx - Vy + self.WHEEL_DIS_FROM_CENTER * omega) / self.WHEEL_RADIUS  # backward right

        self.MotorPWM[0] = -int(self.MotorPWM[0] / 7.87 * self.max_pwm)
        self.MotorPWM[1] = -int(self.MotorPWM[1] / 7.87 * self.max_pwm)
        self.MotorPWM[2] = int(self.MotorPWM[2] / 7.87 * self.max_pwm)
        self.MotorPWM[3] = int(self.MotorPWM[3] / 7.87 * self.max_pwm)
        
        self.get_logger().info('Motor PWM values: Motor1={}, Motor2={}, Motor3={}, Motor4={}'.format(self.MotorPWM[0], self.MotorPWM[1], self.MotorPWM[2], self.MotorPWM[3]))

    def uart_timer_callback(self):
        
        self.mutex.acquire()

        try:

            dataToSend = f"{self.MotorPWM[0]},{self.MotorPWM[1]},{self.MotorPWM[2]},{self.MotorPWM[3]},\n".encode('utf-8')
            self.ser.write('m'.encode('utf-8'))
            self.ser.write(dataToSend)

            # response = self.ser.readline().strip()
            # self.get_logger().info(f"sent {dataToSend} rec {response}")
            # response=0

            self.ser.write('e'.encode('utf-8'))
            # response = self.ser.readline().strip()  # read(3) will read 3 bytes
            # print(f"revived {response}")

            # todo: publish encoder data here 
            # encoder_data:[int,int,int]=[int(num) for num in response.split()] 

        finally:
            self.mutex.release()

def main(args=None):
    rclpy.init(args=args)
    node = Uart()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()