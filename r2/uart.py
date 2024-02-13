import rclpy
import serial
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
import math
from threading import Lock

colors = {
    'red': '\033[91m',
    'green': '\033[92m',
    'yellow': '\033[93m',
    'blue': '\033[94m',
    'magenta': '\033[95m',
    'cyan': '\033[96m',
    'white': '\033[97m',
    'reset': '\033[0m'
}

class Uart(Node):
    def __init__(self):
        super().__init__('uart_node')

        self.pico_pwm = 254
        self.max_pwm = 20 / 100 * self.pico_pwm / 16

        self.loop_rate=0.020 # miliseconds
        self.MotorPWM:[int,int,int,int]=[0,0,0,0] 

        self.matrix_4x3 = np.array([[15.75, 0, -5.66909078166105],
                            [0, 15.75, 5.66909078166105],
                            [-15.75, 0, 5.66909078166105],
                            [0, -15.75,-5.66909078166105]])

        self.mutex = Lock()
        #self.ser = serial.Serial('/dev/ttyACM0', 115200) 
        self.uart_timer=self.create_timer(self.loop_rate,self.uart_timer_callback)
        self.subscription = self.create_subscription(Twist,'piRobot/cmd_vel',self.twist_callback,10)
        # self.encoder=self.create_publisher(Motor_encoder,'/drive/raw_encoder',10)
        self.get_logger().info(f"{colors['green']} Uart Node started ")

        
    def twist_callback(self, msg):

        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z

        # Define a Bot Velocity ~ 3x1 matrix
        matrix_3x1 = np.array([[linear_x],
                            [linear_y],
                            [angular_z]])
        # Perform matrix multiplication
        result_matrix = np.dot(self.matrix_4x3, matrix_3x1)

        self.MotorPWM[0] = int(result_matrix[0]  * self.max_pwm)
        self.MotorPWM[1] = int(result_matrix[1]  * self.max_pwm)
        self.MotorPWM[2] = int(result_matrix[2]  * self.max_pwm)
        self.MotorPWM[3] = int(result_matrix[3]  * self.max_pwm)
        
        self.get_logger().info('Motor PWM values: {} Motor1={}, Motor2={}, Motor3={}, Motor4={}'.format(colors['yellow'],self.MotorPWM[0], self.MotorPWM[1], self.MotorPWM[2], self.MotorPWM[3]))

    def uart_timer_callback(self):
        
        self.mutex.acquire()

        try:
            dataToSend = f"{self.MotorPWM[0]},{self.MotorPWM[1]},{self.MotorPWM[2]},{self.MotorPWM[3]},\n".encode('utf-8')
            self.ser.write('m'.encode('utf-8'))
            self.ser.write(dataToSend)

            self.ser.write('e'.encode('utf-8'))

        finally:
            self.mutex.release()

def main(args=None):
    rclpy.init(args=args)
    node = Uart()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    
# Archived Lines

# Vx = msg.linear.x
# # Vy = msg.linear.y
# Vy = msg.angular.z
# # omega = msg.angular.z
# omega = msg.linear.y

# self.MotorPWM[0] = (Vx - Vy - self.WHEEL_GEOMETRY * omega) / self.WHEEL_RADIUS
# self.MotorPWM[1] = (Vx + Vy + self.WHEEL_GEOMETRY * omega) / self.WHEEL_RADIUS
# self.MotorPWM[2] =  (Vx + Vy - self.WHEEL_GEOMETRY * omega) / self.WHEEL_RADIUS
# self.MotorPWM[3] = (Vx - Vy + self.WHEEL_GEOMETRY * omega) / self.WHEEL_RADIUS

'''
self.ROBOT_WIDTH = 0.229
self.ROBOT_LENGTH = 0.550
self.WHEEL_RADIUS = 0.127
self.WHEEL_DIS_FROM_CENTER = 0.229
self.WHEEL_GEOMETRY = 0.550 + 0.229  # Height and width in meters
'''

# response = self.ser.readline().strip()
# self.get_logger().info(f"sent {dataToSend} rec {response}")
# response=0

            
# response = self.ser.readline().strip()  # read(3) will read 3 bytes
# print(f"revived {response}")

# todo: publish encoder data here 
# encoder_data:[int,int,int]=[int(num) for num in response.split()] 