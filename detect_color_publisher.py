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
        super().__init__('color_publisher')
        
        self.publisher_ = self.create_publisher(String, 'detected_colors_topic', 10)
        self.position_publisher_ = self.create_publisher(String, 'detected_colors_positions', 10)
        
        self.subscription = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.image_callback,
            10)
        self.subscription
        
        self.br = CvBridge()

    def find_color_positions(self, mask, color_name, image):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            max_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(max_contour)
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(image, color_name, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            return f"{color_name}: x={x}, y={y}, w={w}, h={h}"
        return None

    def image_callback(self, msg):
        cv_image = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # ... [rest of the color detection code]

        pink_position = self.find_color_positions(mask_pink, 'Pink', cv_image)
        green_position = self.find_color_positions(mask_green, 'Green', cv_image)

        # Publish the positions if detected
        if pink_position:
            self.position_publisher_.publish(pink_position)
        if green_position:
            self.position_publisher_.publish(green_position)

        cv2.imshow('Result', cv_image)
        cv2.waitKey(1)

        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    
    color_publisher = ColorPublisher()
    
    rclpy.spin(color_publisher)
    
    color_publisher.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
