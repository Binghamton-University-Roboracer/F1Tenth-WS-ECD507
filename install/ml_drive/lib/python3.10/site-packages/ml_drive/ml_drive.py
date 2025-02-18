import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        
        # Create a subscriber to the /image_raw topic
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10  # QoS depth
        )
        self.subscription  # Prevent unused variable warning

        # Initialize CvBridge for ROS <-> OpenCV conversion
        self.bridge = CvBridge()

    def image_callback(self, msg):
        """Callback function to process the received image."""
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Display the image
            cv2.imshow("Camera Feed", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
