import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from cv_bridge import CvBridge
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from visualization_msgs.msg import Marker, MarkerArray
from message_filters import TimeSynchronizer, Subscriber
import cv2
import numpy as np
import math


class ColorPublisher(Node):
    """
    Create an ImagePublisher class, which is a subclass of the Node class.
    """
    def __init__(self):
        super().__init__('color_publisher')
        
        # image data subscription
        self.subscription = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.image_callback, 10)
        self.subscription
        # object data structure. For storing detected marker objects before creating a marker. Resets each image_callback loop. 
        # Each entry has keys "x": int, x position in image, "y": int, y position in image, "color": string, "centered": bool 
        self.object = {}

        # marker data structure. marker_list.markers stores created markers.
        # ros standard marker structure stored in list.
        self.marker_list = MarkerArray()
        self.marker_list.markers = []

        # laser listener
        self.scan_data = {}
        self.laser_scan = self.create_subscription(LaserScan, "/scan",
                                                   self.laser_callback, 
                                                   10)
        # marker publisher
        self.marker_publisher = self.create_publisher(MarkerArray, "visualization_marker_array", 10)

        #transform listener.
        self.tf2_buff = Buffer()
        self.tf2_listen = TransformListener(self.tf2_buff, self) 

        # camera details for angle+distance math
        self.hfov = 18
        self.vfov = 14
        self.cam_width = 160
        self.cam_height = 120
        self.br = CvBridge()

    ### callback functions ###
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

        # issue: what if two markers are in frame at the same time with the same color? Should maybe have some sort of while loop. 
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

        # loop through object data structure creating markers for objects detected that are centered and have not had a marker created already
        for obj in self.object:
            if obj["centered"]:
                cam_frame_pos = self.calc_real_camframe(obj)
                if not cam_frame_pos: continue
                map_frame_pos = self.transformToMap(cam_frame_pos)
                if self.check_seen(map_frame_pos[0], map_frame_pos[1]): continue
                self.createMarker(map_frame_pos, obj["color"])

        self.object = []

    ### helper functions ###

    # find horizontal and vertical angle in degrees to obj from pixel measurements given the vertical and horizontal fov of the camera
    # returns tuple[2] of type double, first index representing horizontal angle, second 
    def find_angle_to_obj(self, obj_center):
        hor_angle = ((obj_center[0] - self.cam_width)/self.cam_width)*self.hfov
        ver_angle = ((obj_center[1] - self.cam_height)/self.cam_height)*self.vfov
        return hor_angle, ver_angle
    
    # check if object is too far to left or right. 
    # If too far to left or right the center of the object will be a "fake" center as we do not see the entire object.
    # returns True or False, if centered True. 
    def check_centered(self, x, w):
        if x+w/2 == self.cam_width:
            return False
        elif x-w/2 == 0:
            return False
        return True
    
    # function finds position of object in camera frame using angle and data from laser scan.
    # returns None or double[3] coordinates. 
    def calc_real_camframe(self, obj):
        hor_angle, ver_angle = self.find_angle_to_obj(obj["x"], obj["y"])
        # issue: scans and image callback may not be synchronous causing for this distance from laser scan to be inaccurate to the angle gotten from image data
        # meaning failure. 
        # TODO: implement message_filter synchronization with Subscriber and TimeSynchronizer
        # https://docs.ros.org/en/melodic/api/message_filters/html/python/index.html
        dist = self.scan_data[hor_angle]
        # check for if object is too close. too close will cause for incorrect z pos, stops marker creation in image_callback
        if (dist < 0.5):
            return None
        coords = []
        coords[0] = dist*math.cos(hor_angle*(math.pi/180))
        coords[1] = dist*math.sin(hor_angle*(math.pi/180))
        coords[2] = dist*math.tan(ver_angle*(math.pi/180))
        return coords

    # finds center of color blobs.
    # returns center x, y and bounding box dimensions w: width, h: height
    def find_color_positions(self, mask, color_name, image):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            max_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(max_contour)
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(image, color_name, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            return (x+(w/2)), (y+(h/2)), w, h
        return None

    # append object to object data structure with x and y coords in image, color and centered status.
    # void return, edits object data structure.
    def appendObject(self, position, color):
        x, y, w = position
        centered = self.check_centered(x, w)
        self.objects.append({"x": x, "y": y, "color": color, "centered": centered})

    # check if object has had a marker created already. 
    # returns bool, True if marker for object found. 
    def check_seen(self, coord):
        for marker in self.marker_list.markers:
            # checks dist of markers to object. if dist is less than 0.5 likely marker is for the object. 
            if math.sqrt((marker.pose.positionf.x**2 - coord[0]**2) + (marker.pose.position.y**2 - coord[1]**2) <= 0.5):
                return True
            return False

    # function to convert coordinate from camera frame to map frame
    # returns double[3] coordinates.
    def transformToMap(self, coordinate):
        transform = self.tf2_buff.lookup_transform(target_frame="map", source="camera_link", time=rclpy.time.Time()).transform
        coordinate[0] = coordinate[0] + transform[0]
        coordinate[1] = coordinate[1] + transform[1]
        coordinate[2] = coordinate[2] + transform[2]
        return coordinate


    # function to create marker given map frame coords and color. Appends marker to marker list structure. 
    # returns nothing. edits marker data structure and publishes to topic marker. 
    def createMarker(self, coord, color):
        marker = Marker()

        marker.header.frame_id = "/map"
        marker.header.stamp = rclpy.time.Time()

        marker.type = marker.CYLINDER
        marker.id = len(self.marker_list.markers + 1)

        # Set the scale of the marker
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 0.5
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
