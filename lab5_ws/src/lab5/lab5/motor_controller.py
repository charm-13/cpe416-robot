import rclpy
from rclpy.node import Node
import rospy
from geometry_msgs.msg import Twist
import Jetson.GPIO as GPIO
import time

# Define GPIO pins for motor control
MOTOR_LEFT = # Set to the GPIO pin for the left motor (32, 33, 15??)
MOTOR_RIGHT = # Set to the GPIO pin for the right motor (32, 33, 15??)

class MotorController(Node):

    def __init__(self):
        super().__init__('bump_and_go')
        
        self.publisher = self.create_subscription(Twist, 'diff_drive/cmd_vel', self.drive, 10) 
        self.twist = Twist()
        
        # Initialize GPIO pins for PWM
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(MOTOR_LEFT, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(MOTOR_RIGHT, GPIO.OUT, initial=GPIO.HIGH)
        
    def drive(self):
        PWMstat_left = GPIO.PWM(MOTOR_LEFT, 100)
        PWMstat_right = GPIO.PWM(MOTOR_RIGHT, 100)

        # Set PWM duty cycle
        PWMstat_left.ChangeDutyCycle(50)
        PWMstat_right.ChangeDutyCycle(50)

        PWMstat_left.start(0)
        PWMstat_right.start(0)        

def main(args=None):
    rclpy.init(args=args)
    robot_node = MotorController()

    rclpy.spin(robot_node)
    
    robot_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
 