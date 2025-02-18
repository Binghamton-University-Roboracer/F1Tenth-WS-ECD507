import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
import cv2
import csv
import os
import time
from cv_bridge import CvBridge

class DataCollector(Node):
    def __init__(self):
        super().__init__('data_collector')

        # Subscribe to camera and odometry topics
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback,10)

        # Image conversion bridge
        self.bridge = CvBridge()

        # Initialize steering and throttle variables
        self.latest_steering = 0.0
        self.latest_throttle = 0.0
        self.latest_lidar = None


        # Create directory for images
        self.image_dir = os.path.expanduser("~/ecd507-training/data")
        os.makedirs(self.image_dir, exist_ok=True)

        # Open CSV file for logging
        self.log_file = open(os.path.join(self.image_dir, "driving_log.csv"), "w", newline='')
        self.writer = csv.writer(self.log_file)
        self.writer.writerow(["timestamp", "image_path","lidar_list", "steering", "throttle"])
        

    def lidar_callback(self,msg):
        self.latest_lidar = list(msg.ranges)
        

    def image_callback(self, msg):
        # Convert ROS 2 Image to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Generate timestamped image path
        timestamp = time.time()
        img_path = os.path.join(self.image_dir, f"{timestamp}.jpg")

        # Save image
        cv2.imwrite(img_path, frame)

        # Save data to CSV
        self.writer.writerow([timestamp, img_path, self.latest_lidar, self.latest_steering, self.latest_throttle])
        # self.get_logger().info(f"Saved {img_path}, Steering: {self.latest_steering}, Throttle: {self.latest_throttle}")

    def odom_callback(self, msg):
        # Extract throttle from linear velocity (assuming x-direction motion)
        self.latest_throttle = msg.twist.twist.linear.x  

        # Extract steering from angular velocity (assuming z-axis turning)
        self.latest_steering = msg.twist.twist.angular.z  

    def cleanup(self):
        self.log_file.close()
        self.get_logger().info("Data logging complete.")

def main():
    rclpy.init()
    node = DataCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cleanup()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
