import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


topic_name = '/unilidar/cloud'
queue_size = 10

class pCloud2UDP(Node):
    
    def __init__(self):
        super().__init__('pCloud2UDP')
        
        # Define UDP parameters here
        
        # Subscription to topic
        self.subscription = self.create_subscription(
            PointCloud2,
            topic_name,
            self.pCloud_callback,
            queue_size)
        self.get_logger().info("Subscribed to /unilidar/cloud")
        
    def pCloud_callback(self, msg):
        print('message received. Callback activated.')
        
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
    