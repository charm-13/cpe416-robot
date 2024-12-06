import rclpy
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool, Float32

import tf2_ros
import tf2_geometry_msgs
from tf_transformations import translation_matrix, quaternion_matrix, quaternion_from_matrix
from tf2_ros import TransformException

import numpy as np

''' For this lab we are going to use the TF2 library provided by ROS
to create an obstable detector node. In the end we want the following transformation:

--- odom2object

The functions "transform_to_matrix" and "multiply_transforms" are provided
as utility functions so that we can multiply two geometry_msgs/msg/TransformStamped
data types.

Your code should have one call the "multiply_transforms" function. That is all that
is needed for this assignment. The idea is we calculate the laser2object transform and
then get the odom2laser transfrom from the /tf topic (Note: this is NOT done through
subscriptions). Then we can multiply to get the following:
odom2object = odom2laser * laser2object (Note: that the laser parts of the transforms "cancel out")

Check the assignment notes for more information.
'''

class ObstacleDetectorImprovedNode(Node):

    def __init__(self):
        super().__init__('detector')
        
        self.scan_sub = self.create_subscription(
            LaserScan, "diff_drive/scan", self.scan_callback, 10
        )

        # We will use these variables to listen to the ROS2 transformation tree
        # and to broadcast our own transformation from Odom -> Object
        self.tf_buffer = tf2_ros.Buffer()
        # this object lets us listen to the /tf topic
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # this object lets us publish the transform to the /tf topic
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        
        self.distance = 'inf'
        self.status_publisher = self.create_publisher(Bool, "laser_status", 10)
        self.dist_publisher = self.create_publisher(Float32, "distance", 10)

        # Arguments are:
        # Period in seconds (float) | callback (callable)
        self.timer = self.create_timer(1.0, self.process_scan)
        self.timer = self.create_timer(1.0, self.laser_status)

        self.latest_laser = LaserScan()
        
        # To check the last time a scan was received
        self.last_scan_time = self.get_clock().now()

    def transform_to_matrix(self, transform):
        # Convert a Transform message to a 4x4 transformation matrix
        translation = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        ])
        rotation = np.array(quaternion_matrix((
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        ))[:3, :3])  # Extract the rotation part

        # Create the full transformation matrix
        matrix = np.eye(4)
        matrix[:3, :3] = rotation
        matrix[:3, 3] = translation
        return matrix

    def multiply_transforms(self, transform_a, transform_b):
        # Multiply two Transform messages and return the resulting Transform
        # Convert transforms to matrices
        matrix_a = self.transform_to_matrix(transform_a)
        matrix_b = self.transform_to_matrix(transform_b)

        # Perform matrix multiplication
        result_matrix = np.dot(matrix_a, matrix_b)

        # Extract translation and rotation from the result matrix
        result_transform = TransformStamped()
        result_transform.transform.translation.x = result_matrix[0, 3]
        result_transform.transform.translation.y = result_matrix[1, 3]
        result_transform.transform.translation.z = result_matrix[2, 3]
        
        # Convert the rotation part back to a quaternion
        result_rotation = quaternion_from_matrix(result_matrix)
        result_transform.transform.rotation.x = result_rotation[0]
        result_transform.transform.rotation.y = result_rotation[1]
        result_transform.transform.rotation.z = result_rotation[2]
        result_transform.transform.rotation.w = result_rotation[3]

        return result_transform
    
    def scan_callback(self, msg):
        # Copy the data from the laser scanner into member variable
        self.latest_laser = msg
        self.last_scan_time = self.get_clock().now()

    def process_scan(self):
        # Do nothing if there is no data from the laser yet
        if not self.latest_laser.ranges:
            return

        # Access the laser scan that is aligned with x-axis of the robot
        l_ranges = len(self.latest_laser.ranges)
        self.distance = self.latest_laser.ranges[l_ranges // 2]   # x -> midway between angle_min and angle_max
        
        self.dist_publisher.publish(Float32(data=self.distance))

        if self.distance != 'inf':

            laser2object_msg = TransformStamped()
            
            laser2object_msg.transform.translation.x = self.distance
            laser2object_msg.transform.translation.y = 0.0
            laser2object_msg.transform.translation.z = 0.0

            laser2object_msg.header.frame_id = self.latest_laser.header.frame_id # diff_drive/lidar_link
            laser2object_msg.child_frame_id = "detected_obstacle"

            try:
                # to: diff_drive/lidar_link, from: diff_drive/odom
                odom2laser_msg = self.tf_buffer.lookup_transform(
                    "diff_drive/odom", self.latest_laser.header.frame_id, rclpy.time.Time()
                )

            except TransformException as ex:
                self.get_logger().warn(f'Obstacle transform not found: {ex}')
                return
            
            odom2object_msg = self.multiply_transforms(odom2laser_msg, laser2object_msg)

            odom2object_msg.header.frame_id = "diff_drive/odom"
            odom2object_msg.child_frame_id = "detected_obstacle"

            # Call the broadcaster
            self.tf_broadcaster.sendTransform(odom2object_msg)
    
    def laser_status(self):
        time_since_last_scan = self.get_clock().now() - self.last_scan_time
        
        if time_since_last_scan.nanoseconds > 2e9:  # 2 second timeout
            self.get_logger().warn("No laser scan received in over 2 seconds.")
            self.status_publisher.publish(Bool(data=True))
        
        self.status_publisher.publish(Bool(data=False))
    
def main(args=None):
    rclpy.init(args=args)
    obstacle_node = ObstacleDetectorImprovedNode()

    rclpy.spin(obstacle_node)
    
    obstacle_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
