import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped
import torch.nn as nn
import cv2
from cv_bridge import CvBridge
import torch  # Use TensorFlow if your model is in TF
import numpy as np
import os

# Define CNN model
class CNNModel(nn.Module):
    def __init__(self):
        super(CNNModel, self).__init__()
        self.conv_layers = nn.Sequential(
            nn.Conv2d(3, 24, kernel_size=5, stride=2),
            nn.ReLU(),
            nn.Conv2d(24, 36, kernel_size=5, stride=2),
            nn.ReLU(),
            nn.Conv2d(36, 48, kernel_size=5, stride=2),
            nn.ReLU(),
            nn.Conv2d(48, 64, kernel_size=3),
            nn.ReLU()
        )
        self.fc_layers = nn.Sequential(
            nn.Flatten(),
            nn.Linear(3840, 100),
            nn.ReLU(),
            nn.Linear(100, 50),
            nn.ReLU(),
            nn.Linear(50, 10),
            nn.ReLU(),
            nn.Linear(10, 2)  # Output: Steering angle and Throttle
        )
    
    def forward(self, x):
        x = self.conv_layers(x)
        x = self.fc_layers(x)
        return x

class AutonomousDriver(Node):
    def __init__(self):
        super().__init__('autonomous_driver')
        self.bridge = CvBridge()

        # Load the trained model (assuming PyTorch for now)
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = CNNModel().to(self.device)
        checkpoint = torch.load(os.path.expanduser('~/training/model.pth'))
        
        self.model.load_state_dict(checkpoint)
        self.get_logger().info(f"Model Loaded Successfully")

        #,map_location=torch.device(self.device) - extra argument to previous line
        self.model.eval()  # Set model to inference mode

        # Subscribers and Publishers
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.control_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        
    def image_callback(self, msg):
        # Convert ROS2 Image to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Preprocess image (resize, normalize, convert to tensor)
        processed_frame = self.preprocess_image(frame)

        # Get steering prediction from the model
        control_values = self.predict_control(processed_frame)[0]
        
        
        
        
               
        self.get_logger().info(f"{control_values}")

        # Publish the control command
        self.publish_control(control_values)

    def preprocess_image(self, frame):
        """Resize, normalize, and convert image to tensor."""
        frame = cv2.resize(frame, (200, 66))  # Resize to match training input
        frame = frame / 255.0  # Normalize
        frame = np.transpose(frame, (2, 0, 1))  # Change shape to (C, H, W)
        frame = torch.tensor(frame, dtype=torch.float32).unsqueeze(0).to(self.device)  # Add batch dimension
        return frame

    def predict_control(self, frame):
        """Use the ML model to predict steering angle."""
        with torch.no_grad():
            output = self.model(frame)  # Run inference
        return output.tolist() # Convert tensor output to float

    def publish_control(self, control_values):
        """Publish the predicted steering command."""
        steering_angle, throttle = control_values[0], control_values[1]
        msg = AckermannDriveStamped()
        msg.drive.steering_angle = steering_angle
        msg.drive.speed = throttle  # Adjust speed as needed
        self.control_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
