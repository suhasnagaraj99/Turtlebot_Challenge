import cv2 
from matplotlib import pyplot as plt 
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from ultralytics import YOLO
from std_msgs.msg import Bool

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        
        # self.subscription = self.create_subscription(Image,'/image_raw',self.camera_callback,qos_profile_sensor_data)
        self.subscription = self.create_subscription(Image,'/camera/image_raw',self.camera_callback,qos_profile_sensor_data)
        self.subscription
        self.publisher = self.create_publisher(Image,'/count',qos_profile_sensor_data)


    def camera_callback(self, msg):
        pub_msg=Image()
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) 
        _, thresh = cv2.threshold(img_gray, 245, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 1000: 
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    cv2.circle(img, (cX, cY), 5, (0, 255, 0), -1)
        pub_img = bridge.cv2_to_imgmsg(img, encoding="rgb8")
        self.publisher.publish(pub_img)
        
        
            
def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)
    
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

