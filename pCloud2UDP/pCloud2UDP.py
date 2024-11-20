import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import socket
import struct
import json
from ament_index_python.packages import get_package_share_directory

package_name = 'pCloud2UDP'
topic_name = '/unilidar/cloud'
queue_size = 10
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
        msg_converted = identifier + header + metadata + msg.data
        
        return msg_converted
        
    def pCloud_callback(self, msg):
        if DEBUG:
            self.udp_socket.sendto(bytes(bytearray([3])), (self.udp_target_ip, self.udp_target_port))
            self.get_logger().info('PointCloud2 message sent via UDP.')
        else:
            try:
                self.udp_socket.sendto(self.msg_as_byte_array(msg), (self.udp_target_ip, self.udp_target_port))
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
    