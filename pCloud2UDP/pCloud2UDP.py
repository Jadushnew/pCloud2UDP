import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import socket
import struct
import csv
import json
import math
from ament_index_python.packages import get_package_share_directory

package_name = 'pCloud2UDP'
topic_name = '/unilidar/cloud'
queue_size = 10
max_package_size = 60_000 # bytes -> 60kb (max udp package size ~ 65 kb)
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
        
        self.msg_counter = 0
        
    def msg_as_byte_array(self, msg):
        
        metadata = struct.pack(
            "IIII",
            msg.height,
            msg.width,
            msg.point_step,
            msg.row_step
            )
        
        # msg.data contains not needed data like intensity, ring, etc. We only need x,y, and z, so we only need the first 12 bytes of every point
        # The rest of the information (the other 16 bytes) will be ignored
        point_offset = 24 # Ohne 4-er Loch
        wanted_bytes = 12
        cut_data = [];
        for i in range(0, len(msg.data), point_offset):
            cut_data.extend(msg.data[i:i+wanted_bytes])
            
        identifier = bytes(bytearray([255,255,255,255]))
        end_identifier = bytes(bytearray([181,181,181,181]))
        msg_converted = identifier + metadata + bytes(cut_data) + end_identifier
        
        return msg_converted
    
    def calculate_message_size(self, msg):
        header_size = 12 # Standard
        metadata_size = len(bytes(msg.height)) + len(bytes(msg.width)) + len(bytes(msg.point_step)) + len(bytes(msg.row_step))
        data_size = len(msg.data)
        misc_size = 2 # is_dense and is_bigendian
        
        return header_size + metadata_size + data_size + misc_size
        
        
    def pCloud_callback(self, msg):
        try:
            if self.msg_counter < 1:
                try:
                    point_offset = 28
                    wanted_bytes = 4
                    for i in range(0, 112):
                       curr_4byte = msg.data[i]
                       with open('doubledata.csv', mode='a', newline='') as file:
                           writer = csv.writer(file)
                           writer.writerow([curr_4byte])
                        
                    # for i in range(0, len(msg.data), point_offset):
                    #     curr_x = msg.data[i:i+wanted_bytes]
                    #     with open('x_output.csv', mode='a', newline='') as file:
                    #         writer = csv.writer(file)
                    #         writer.writerow([struct.unpack('f', bytes(curr_x))[0]])
                    # for i in range(4, len(msg.data), point_offset):
                    #     curr_x = msg.data[i:i+wanted_bytes]
                    #     with open('y_output.csv', mode='a', newline='') as file:
                    #         writer = csv.writer(file)
                    #         writer.writerow([struct.unpack('f', bytes(curr_x))[0]])
                    # for i in range(8, len(msg.data), point_offset):
                    #     curr_x = msg.data[i:i+wanted_bytes]
                    #     with open('z_output.csv', mode='a', newline='') as file:
                    #         writer = csv.writer(file)
                    #         writer.writerow([struct.unpack('f', bytes(curr_x))[0]])
                except Exception as e:
                    self.get_logger().error(f"Writing failed: {e}")
                
                self.msg_counter +=1
            self.get_logger().info(f"Received PointCloud2 message. Size: {self.calculate_message_size(msg)} bytes.")
            # Serialize PointCloud2
            serialized_msg = self.msg_as_byte_array(msg)
            
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
                self.get_logger().info(f"Message sent via UDP to {self.udp_target_ip}. Size: {len(package)} bytes.")
        except Exception as e:
            self.get_logger().error(f"Failed to send PointCloud2 data: {e}")
        self.get_logger().info("")
    
        
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
    