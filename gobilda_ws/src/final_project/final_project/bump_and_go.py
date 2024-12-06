import rclpy
import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32

import tf2_ros
import tf2_geometry_msgs
from tf_transformations import translation_matrix, quaternion_matrix, quaternion_from_matrix
from tf2_ros import TransformException

from enum import Enum, auto

class State(Enum):
    forward = auto()
    backward = auto()
    turning = auto()
    stop = auto()

class BumpAndGo(Node):

    def __init__(self):
        super().__init__('bump_and_go')
        
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.twist = Twist()
        self.linear_velocity = 0.5
        self.angular_velocity = 0.35
        
        self.status = State.forward
        
        self.laser_sub = self.create_subscription(
            Bool, "laser_status", self.laser_status, 10
        )
        
        self.distance = 'inf'
        self.distance_sub = self.create_subscription(
            Float32, "distance", self.curr_distance, 10
        )

        self.timer = self.create_timer(0.1, self.process_scan) # Period in seconds (float) | callback (callable)
        
        self.get_logger().info(f"Starting... going forward...")
        
    def laser_status(self, msg):
        if msg.data == True:
            self.status = State.stop
            
    def curr_distance(self, msg):
        self.distance = msg.data

    def process_scan(self):
        match self.status:
            case State.forward:               
                if self.distance != 'inf' and self.distance <= 5.0:
                    self.status = State.backward
                    self.get_logger().info(f"Backward!")
                
                # go forward
                self.twist.linear.x = self.linear_velocity
                self.twist.angular.z = 0.0
                self.publisher.publish(self.twist)
                
            case State.backward:
                if self.distance != 'inf' and self.distance > 5.0:
                    self.status = State.turning
                    self.get_logger().info(f"Turning!")
                    
                # go backward at 1/2 speed of forward
                self.twist.linear.x = -1 * (self.linear_velocity / 2)
                self.twist.angular.z = 0.0
                self.publisher.publish(self.twist)
                
            case State.turning:
                if self.distance > 5.0:   # if no obstacle -> forward
                    self.status = State.forward
                    self.get_logger().info(f"Forward!")
                    
                # turn
                self.twist.linear.x = 0.0
                self.twist.angular.z = self.angular_velocity
                self.publisher.publish(self.twist)
                
            case _: # State.stop or some error
                self.get_logger().info(f"Shutting off...")
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.publisher.publish(self.twist)
                self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    robot_node = BumpAndGo()

    rclpy.spin(robot_node)
    
    robot_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
 