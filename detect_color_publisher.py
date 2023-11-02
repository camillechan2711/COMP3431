import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorPublisher(Node):
    """
    Create an ImagePublisher class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        super().__init__('color_publisher')
        
        # Create the publisher for detected colors
        self.publisher_ = self.create_publisher(String, 'detected_colors_topic', 10)
        
        # Subscribe to the TurtleBot's camera topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

    def image_callback(self, msg):
        """
        Callback function.
        This function gets called upon receiving an image message from the TurtleBot's camera.
        """
        # Convert ROS Image message to OpenCV image
        cv_image = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Convert to HSV
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define color ranges
        lower_pink = np.array([0, 100, 124])
        upper_pink = np.array([163, 234, 226])
        lower_yellow = np.array([0, 81, 62])
        upper_yellow = np.array([103, 237, 128])
        lower_green = np.array([0, 93, 62])
        upper_green = np.array([81, 235, 255])

        # Create masks for each color
        mask_pink = cv2.inRange(hsv_image, lower_pink, upper_pink)
        mask_yellow = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
        mask_green = cv2.inRange(hsv_image, lower_green, upper_green)

        # Check the number of non-zero pixels for each mask
        non_zero_pink = cv2.countNonZero(mask_pink)
        non_zero_yellow = cv2.countNonZero(mask_yellow)
        non_zero_green = cv2.countNonZero(mask_green)

        threshold = 150
        detected_colors = []

        # Determine the detected colors
        if non_zero_pink > threshold:
            detected_colors.append('Pink')
        if non_zero_yellow > threshold:
            detected_colors.append('Yellow')
        if non_zero_green > threshold:
            detected_colors.append('Green')

        # Publish the detected colors
        msg = String()
        msg.data = ', '.join(detected_colors) if detected_colors else 'No colors detected'
        self.publisher_.publish(msg)
        
        # Display the message on the console
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    
    color_publisher = ColorPublisher()
    
    rclpy.spin(color_publisher)
    
    color_publisher.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
