import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from cv_bridge import CvBridge
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from visualization_msgs.msg import Marker, MarkerArray
import cv2
import numpy as np
import math


#TODO implement math

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
        
        self.object = {}

        self.marker_list = MarkerArray()
        self.marker_list.markers = []

        # laser listener
        self.scan_data = {}
        self.laser_scan = self.create_subscription(LaserScan, "/scan",
                                                   callback=self.laser_callback, 
                                                   10)
        # marker publisher
        self.marker_publisher = self.create_publisher(MarkerArray, "visualization_marker_array", 10)

        #transform 
        self.tf2_buff = Buffer()
        self.tf2_listen = TransformListener(self.tf2_buff, self) 

        self.hfov = 62
        self.vfov = 48
        self.cam_width = 680
        self.cam_height = 480
        self.br = CvBridge()


    def laser_callback(self, data):
        for i in range (-30, 30):
            self.scan_data[abs(i)] = data.ranges[i]

    def image_callback(self, msg):
        cv_image = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # set color range
        lower_pink = np.array([0, 100, 124])
        upper_pink = np.array([163, 234, 226])
        lower_yellow = np.array([0, 93, 88])
        upper_yellow = np.array([100, 252, 205])
        # lower_green = np.array([0, 93, 62])
        # upper_green = np.array([81, 235, 255])
        lower_green = np.array([36, 179, 59])
        upper_green = np.array([60, 246, 128])
        lower_blue = np.array([87, 167, 116])
        upper_blue = np.array([101, 255, 231])

        #  create mask for every color
        mask_pink = cv2.inRange(hsv_image, lower_pink, upper_pink)
        mask_yellow = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
        mask_green = cv2.inRange(hsv_image,lower_green, upper_green)
        mask_blue = cv2.inRange(hsv_image,lower_blue, upper_blue)

        #  check every color's mask number of non-zero pix
        non_zero_pink = cv2.countNonZero(mask_pink)
        non_zero_yellow = cv2.countNonZero(mask_yellow)
        non_zero_green = cv2.countNonZero(mask_green)
        non_zero_blue = cv2.countNonZero(mask_blue)

        # set a Threshold 
        threshold = 150

        # result_pink = cv2.bitwise_and(image, image, mask=mask_pink)
        # result_yellow = cv2.bitwise_and(image, image, mask=mask_yellow)
        # result_green = cv2.bitwise_and(image, image, mask=mask_green)

        if non_zero_pink > threshold:
            self.find_color_positions(mask_pink, 'Pink')

        if non_zero_yellow > threshold:
            self.find_color_positions(mask_yellow,'yellow')

        if non_zero_green > threshold:
            self.find_color_positions(mask_green, 'Green')

        if non_zero_blue > threshold:
            self.find_color_positions(mask_blue,'blue')

        cv2.imshow('Result', image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        # ... [rest of the color detection code]

        pink_position = self.find_color_positions(mask_pink, 'Pink', cv_image)
        green_position = self.find_color_positions(mask_green, 'Green', cv_image)
        yellow_position = self.find_color_positions(mask_yellow, 'yellow', cv_image)
        blue_position = self.find_color_positions(mask_blue, 'blue', cv_image)

        if pink_position:
            self.appendObject(pink_position, "pink")
        if green_position:
            self.appendObject(pink_position, "green")
        if yellow_position:
            self.appendObject(pink_position, "yellow")
        if blue_position:
            self.appendObject(pink_position, "blue")
        cv2.imshow('Result', cv_image)
        cv2.waitKey(1)

        for obj in self.object:
            if obj["centered"] and not self.check_seen((obj["x"], obj["y"])):
                cam_frame_pos = self.calc_real_camframe(obj)
                map_frame_pos = self.transformToMap(cam_frame_pos)
                self.createMarker(map_frame_pos, obj["color"])

        self.object = []

    def find_angle_to_obj(self, obj_center):
        hor_angle = ((obj_center[0] - self.cam_width)/self.cam_width)*self.hfov
        ver_angle = ((obj_center[1] - self.cam_height)/self.cam_height)*self.vfov
        return hor_angle, ver_angle
    
    def check_centered(self, x, w):
        if x+w/2 == self.cam_width:
            return False
        elif x-w/2 == 0:
            return False
        return True
    
    def calc_real_camframe(self, obj):
        hor_angle, ver_angle = self.find_angle_to_obj(obj["x"], obj["y"])
        dist = self.scan_data[hor_angle]
        coords = []
        coords[0] = dist*math.cos(hor_angle*(math.pi/180))
        coords[1] = dist*math.sin(hor_angle*(math.pi/180))
        coords[2] = dist*math.tan(ver_angle*(math.pi/180))
        return coords


    def find_color_positions(self, mask, color_name, image):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            max_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(max_contour)
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(image, color_name, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            return x, y, w, h
        return None

        
    def appendObject(self, position, color):
        x, y, w, h = position
        centered = self.check_centered(x, w)
        self.objects.append({"x": x, "y": y, "color": color, "centered": centered})

    def check_seen(self, coord):
        for marker in self.marker_list.markers:
            if math.sqrt((marker.pose.positionf.x**2 - coord[0]**2) + (marker.pose.position.y**2 - coord[1]**2) <= 0.5):
                return True
            return False


    def transformToMap(self, coordinate):
        transform = self.tf2_buff.lookup_transform(target_frame="map", source="camera_link", time=rclpy.time.Time()).transform
        coordinate[0] = coordinate[0] + transform[0]
        coordinate[1] = coordinate[1] + transform[1]
        coordinate[2] = coordinate[2] + transform[2]
        return coordinate

    def createMarker(self, coord, color):
        marker = Marker()

        marker.header.frame_id = "/map"
        marker.header.stamp = rclpy.time.Time()

        marker.type = 3
        marker.id = len(self.marker_list.markers + 1)

        # Set the scale of the marker
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        if color == "pink":
            marker.color.r = 1
            marker.color.g = 0.75
            marker.color.b = 0.79
        elif color == "blue":
            marker.color.r = 0
            marker.color.g = 0
            marker.color.b = 1
        elif color == "yellow":
            marker.color.r = 1
            marker.color.g = 1
            marker.color.b = 0
        elif color == "green":
            marker.color.r = 0
            marker.color.g = 1
            marker.color.b = 0

        # Set the pose of the marker
        marker.pose.position.x = coord[0]
        marker.pose.position.y = coord[1]
        marker.pose.position.z = coord[2]
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        self.marker_list.markers.append(marker)
        self.marker_publisher.publish(marker)
        return

def main(args=None):
    rclpy.init(args=args)
    
    color_publisher = ColorPublisher()
    
    rclpy.spin(color_publisher)
    
    color_publisher.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
