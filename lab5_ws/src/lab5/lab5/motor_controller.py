import rclpy
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
        
        self.publisher = self.create_publisher(Twist, 'diff_drive/cmd_vel', 10) # Idk if we need this
        self.twist = Twist()
        
        ### The block of code below is meant to be similar to the C++ code in the Jetson GPIO PWM example

        # Initialize GPIO pins for PWM
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(MOTOR_LEFT, GPIO.OUT)
        GPIO.setup(MOTOR_RIGHT, GPIO.OUT)

        # Tried to use code similar to the Jetson GPIO PWM C++ example
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
 