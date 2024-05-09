import cv2 
from matplotlib import pyplot as plt 
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from std_msgs.msg import Bool
from std_msgs.msg import Int64MultiArray
import os

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.buffer = []
        # filename = "Turtlebot_Challenge/src/group8/group8/stop_data.xml"
        # cwd = os.getcwd()
        # path = os.path.join(cwd,filename)
        # self.stop_data = cv2.CascadeClassifier(path)    
        self.stop_data = cv2.CascadeClassifier('/home/suhas99/ENPM673/new_ws/src/group8/group8/stop_data.xml')      
        self.subscription = self.create_subscription(CompressedImage,'camera/image_raw/compressed',self.camera_callback,qos_profile_sensor_data)
        self.subscription
        self.publisher_stop = self.create_publisher(Bool,'/stop2',qos_profile_sensor_data)
        self.publisher_box = self.create_publisher(Int64MultiArray,'/box_stop',qos_profile_sensor_data)
        self._bridge = CvBridge()


    def camera_callback(self, msg):
        pub_msg_stop = Bool()
        pub_msg_box = Int64MultiArray()
        
        # img = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        img = self._bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) 

        found = self.stop_data.detectMultiScale(img_gray, minSize=(20, 20)) 
        if len(found) > 0:
            self.buffer.append(True)
        else:
            self.buffer.append(False)

        while len(self.buffer) > 5:
            self.buffer.pop(0)

        if self.buffer.count(True) >= 3:
            for (x, y, w, h) in found:
                x1,y1,x2,y2=x,y,x+w,y+h
                pub_msg_box.data=[int(x1),int(y1),int(x2),int(y2)]
                self.publisher_box.publish(pub_msg_box)
                cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)
            pub_msg_stop.data=True
            self.publisher_stop.publish(pub_msg_stop)
        else:
            pub_msg_stop.data=False
            self.publisher_stop.publish(pub_msg_stop)
            
def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)
    
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

