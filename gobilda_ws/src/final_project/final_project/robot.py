import rclpy
import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32

import tf2_ros
import tf2_geometry_msgs

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

        self.timer = self.create_timer(0.1, self.process_scan) # Period in seconds (float) | callback (callable)
        self.time = self.get_clock().now()
        self.statetime = self.get_clock().now()
        
        self.get_logger().info(f"Starting... \nGoing forward...")

    def process_scan(self):
        self.time = self.get_clock().now()
        match self.status:
            case State.forward:            
                if (self.statetime - self.time).to_msg().sec >= 5.0:
                    self.status = State.backward
                    self.get_logger().info(f"Backward!")
                    self.statetime = self.get_clock().now()
                
                # go forward
                self.twist.linear.x = self.linear_velocity
                self.twist.angular.z = 0.0
                self.publisher.publish(self.twist)
                
            case State.backward:
                if (self.statetime - self.time).to_msg().sec >= 5.0:
                    self.status = State.turning
                    self.get_logger().info(f"Turning!")
                    self.statetime = self.get_clock().now()
                    
                # go backward at 1/2 speed of forward
                self.twist.linear.x = -1 * (self.linear_velocity / 2)
                self.twist.angular.z = 0.0
                self.publisher.publish(self.twist)
                
            case State.turning:
                if (self.statetime - self.time).to_msg().sec >= 5.0:   # if no obstacle -> forward
                    self.status = State.forward
                    self.get_logger().info(f"Forward!")
                    self.statetime = self.get_clock().now()
                    
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
 