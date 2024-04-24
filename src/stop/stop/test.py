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
        
        ## YOLO ##
        yolo_model_path = '/home/suhas99/ENPM673/final_project/src/stop/stop/last.pt'
        self.model = YOLO(yolo_model_path)
        self.threshold = 0.5
        
        ## Haar Cascade ##
        self.buffer = []
        self.stop_data = cv2.CascadeClassifier('/home/suhas99/ENPM673/final_project/src/stop/stop/stop_data.xml') 
        
        # self.subscription = self.create_subscription(Image,'/image_raw',self.camera_callback,qos_profile_sensor_data)
        self.subscription = self.create_subscription(Image,'/camera/image_raw',self.camera_callback,qos_profile_sensor_data)
        self.subscription
        self.publisher = self.create_publisher(Bool,'/stop',qos_profile_sensor_data)


    def camera_callback(self, msg):
        pub_msg = Bool()
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) 
        
        
        results = self.model(img)[0]

        for result in results.boxes.data.tolist():
            x1, y1, x2, y2, score, class_id = result

            if score >= self.threshold:
                pub_msg.data=True
                self.publisher.publish(pub_msg)
            else:
                pub_msg.data=False
                self.publisher.publish(pub_msg)
            
        # ## Haar Cascade ##
        # found = self.stop_data.detectMultiScale(img_gray, minSize=(20, 20)) 
        # if len(found) > 0:
        #     self.buffer.append(True)
        # else:
        #     self.buffer.append(False)

        # while len(self.buffer) > 5:
        #     self.buffer.pop(0)

        # # if all(value == True for value in self.buffer):
        # if self.buffer.count(True) >= 3:
        #     print("Yes")
        # else:
        #     print("No")
            
def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)
    
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

