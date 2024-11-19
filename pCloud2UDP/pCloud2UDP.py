import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import socket
import struct

topic_name = '/unilidar/cloud'
queue_size = 10
DEBUG = False

class pCloud2UDP(Node):
    
    def __init__(self):
        super().__init__('pCloud2UDP')
        
        # Define UDP parameters here
        self.declare_parameter('udp_target_ip', '255.255.255.255')
        self.declare_parameter('udp_target_port', 60811)
        
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
            "IIQ",
            msg.header.stamp.sec,
            msg.header.stamp.nanosec,
            len(msg.header.frame_id)
            ) + msg.header.frame_id.encode('utf-8')
        
        metadata = struct.pack(
            "II??II",
            msg.height,
            msg.width,
            msg.is_bigendian,
            msg.is_dense,
            msg.point_step,
            msg.row_step
            )
        
        fields = struct.pack("I", len(msg.fields))
        for field in msg.fields:
            fields += struct.pack(
                "IIBI",
                len(field.name),
                field.offset,
                field.datatype,
                field.count
                ) + field.name.encode('utf-8')
            
        msg_converted = header + metadata + fields + msg.data
        
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
    