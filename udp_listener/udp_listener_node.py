import rclpy
from rclpy.node import Node
import socket
from geometry_msgs.msg import Quaternion
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped
import tf2_ros
import numpy as np
import cv2

class UDPListener(Node):
    def __init__(self):
        super().__init__('udp_listener')
        self.get_logger().info('UDP Listener Node started')

        # Define the UDP IP and Port
        self.udp_ip = "192.168.0.122"  # Listen on all available interfaces
        self.udp_port = 5005      # Example port, change as needed

        # Create a UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))

        # ROS2 Publishers
        self.model_pub = self.create_publisher(Marker, 'satellite_marker', 10)

        # Create a Transform Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Create a timer to check for incoming messages
        self.timer = self.create_timer(0.1, self.listen_to_udp_port)

    def listen_to_udp_port(self):
        try:
            data, addr = self.sock.recvfrom(1024)  # Buffer size of 1024 bytes
            self.get_logger().info(f"Received message: {data.decode()} from {addr}")

            # Parse the incoming Rodrigues vector
            rod_vect = list(map(float, data.decode().split(',')))

            if len(rod_vect) == 3:
                # Convert Rodrigues vector to quaternion
                quaternion_msg = self.rodrigues_to_quaternion(rod_vect)

                # Publish the marker with the received orientation
                self.publish_model(quaternion_msg)

                # Broadcast the transform for the 'virtual_sat' frame
                self.broadcast_transform(quaternion_msg)
        except socket.error as e:
            self.get_logger().error(f"Socket error: {e}")

    def rodrigues_to_quaternion(self, rod_vect):
        rod_vect = np.array(rod_vect, dtype=np.float64)
        rotation_matrix, _ = cv2.Rodrigues(rod_vect)
        quat = np.zeros(4)
        quat[0] = np.sqrt(1.0 + rotation_matrix[0, 0] + rotation_matrix[1, 1] + rotation_matrix[2, 2]) / 2.0
        quat[1] = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / (4.0 * quat[0])
        quat[2] = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / (4.0 * quat[0])
        quat[3] = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / (4.0 * quat[0])
        return Quaternion(x=quat[1], y=quat[2], z=quat[3], w=quat[0])

    def publish_model(self, quaternion_msg):
        # Publish the Satellite Marker
        mesh = Marker()
        mesh.header.frame_id = "virtual_sat"
        mesh.header.stamp = self.get_clock().now().to_msg()
        mesh.type = mesh.MESH_RESOURCE
        mesh.mesh_resource = "package://udp_listener/models/VirtualSatv8.stl"
        mesh.pose.orientation = quaternion_msg
        mesh.pose.position.x = 0.0
        mesh.pose.position.y = 0.0
        mesh.pose.position.z = 0.0
        mesh.scale.x = 1.0 / 100  # Adjust scale as needed
        mesh.scale.y = 1.0 / 100
        mesh.scale.z = 1.0 / 100
        mesh.color.a = 1.0  # Alpha must be non-zero
        mesh.color.r = 1.0  # Red color
        mesh.color.g = 0.0
        mesh.color.b = 0.0

        self.model_pub.publish(mesh)
        self.get_logger().info("Published satellite marker with updated orientation")

    def broadcast_transform(self, quaternion_msg):
        # Create a TransformStamped message
        t = TransformStamped()

        # Set the header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"  # You may change this to your desired fixed frame
        t.child_frame_id = "virtual_sat"

        # Set the translation (position)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # Set the rotation (orientation)
        t.transform.rotation = quaternion_msg

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = UDPListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
