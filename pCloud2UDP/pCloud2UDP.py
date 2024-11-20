import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import socket
import struct
import json
import math
from ament_index_python.packages import get_package_share_directory

package_name = 'pCloud2UDP'
topic_name = '/unilidar/cloud'
queue_size = 10
max_package_size = 50_000 # bytes -> 50kb (max udp package size ~ 65 kb)
DEBUG = False

def load_config():
    config_path = get_package_share_directory(package_name) + '/config/config.json'
    with open(config_path, 'r') as file:
        return json.load(file)

class pCloud2UDP(Node):
    
    def __init__(self):
        super().__init__(package_name)
        config = load_config()
        
        # Define UDP parameters here
        self.declare_parameter('udp_target_ip', config["ip"])
        self.declare_parameter('udp_target_port', config["port"])
        
        self.udp_target_ip = self.get_parameter('udp_target_ip').value
        self.udp_target_port = self.get_parameter('udp_target_port').value

        # Initialize UDP socket
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.get_logger().info(f"UDP socket initialized to {self.udp_target_ip}:{self.udp_target_port}")        
        
        # Subscription to topic
        self.subscription = self.create_subscription(
            PointCloud2,
            topic_name,
            self.pCloud_callback,
            queue_size)
        self.get_logger().info("Subscribed to /unilidar/cloud")
        
    def msg_as_byte_array(self, msg):
        header = struct.pack(
            "II",
            msg.header.stamp.sec,
            msg.header.stamp.nanosec)
        
        metadata = struct.pack(
            "IIII",
            msg.height,
            msg.width,
            msg.point_step,
            msg.row_step
            )
            
        identifier = bytes(bytearray([255,255,255,255]))
        end_identifier = bytes(bytearray([181,181,181,181]))
        msg_converted = identifier + header + metadata + msg.data + end_identifier
        
        return msg_converted
        
    def pCloud_callback(self, msg):
        if DEBUG:
            self.udp_socket.sendto(bytes(bytearray([3])), (self.udp_target_ip, self.udp_target_port))
            self.get_logger().info('PointCloud2 message sent via UDP.')
        else:
            # Default case
            try:
                # Serialize PointCloud2
                serialized_msg = self.msg_as_byte_array(msg)
                print(f"\nLength of message serialized: {len(serialized_msg)}") # Not sure if that actually works as intended
                
                # Split up the package if necessary
                num_packages_to_send = math.ceil(len(serialized_msg) / max_package_size)
                if num_packages_to_send > 1:
                    self.get_logger().info(f"Package larger than set package size. Splitting into {num_packages_to_send} Packages")
                packages = [serialized_msg[i:i+max_package_size] for i in range(0, len(serialized_msg), max_package_size)]
                if len(packages) > num_packages_to_send:
                    packages[-2] += packages[-1]
                    packages = packages[:-1]
                    
                # Send all packages
                for package in packages:
                    self.udp_socket.sendto(package, (self.udp_target_ip, self.udp_target_port))
                    self.get_logger().info('PointCloud2 message sent via UDP.')
            except Exception as e:
                self.get_logger().error(f"Failed to send PointCloud2 data: {e}")
    
        
def main(args=None):
    rclpy.init(args=args)
    node = pCloud2UDP()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().inf('Shutting down pCloud2UDP node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()
      
if __name__ == '__main__':
    main()
    