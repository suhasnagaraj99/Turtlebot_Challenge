import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from std_msgs.msg import Int64MultiArray

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        
        self.index=0
        self.prev_img_gray=None
        self.img_gray=None
        self.image_subscription = self.create_subscription(CompressedImage,'camera/image_raw/compressed',self.camera_callback,qos_profile_sensor_data) 
        self.subscription2 = self.create_subscription(Int32,'/horizon_level',self.horizon_callback,qos_profile_sensor_data)
        # self.image_subscription = self.create_subscription(Image,'/image_raw',self.camera_callback,qos_profile_sensor_data)           
        # self.image_subscription = self.create_subscription(Image,'/camera/image_raw',self.camera_callback,qos_profile_sensor_data)
        # self.image_subscription = self.create_subscription(CompressedImage,'/camera/image_raw/compressed',self.camera_callback,qos_profile_sensor_data)
        self.publisher_stop = self.create_publisher(Bool,'/stop_dynamic',qos_profile_sensor_data)
        self.publisher_obs = self.create_publisher(Bool,'/obs',qos_profile_sensor_data)
        self.publisher_box = self.create_publisher(Int64MultiArray,'/box_dynamic',qos_profile_sensor_data)
        self.horiz=260
        self.counter = 0
        self.movement_detected = False
        self.horizon_detected = False
        
            
    def camera_callback(self, msg):
        pub_msg_stop = Bool()
        pub_msg_obs = Bool()
        pub_box=Int64MultiArray()
        bridge = CvBridge()
        step = 16
        threshold = 15
        # img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if self.index == 0:
            self.prev_img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            self.index += 1
        else:
            self.img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            flow = cv2.calcOpticalFlowFarneback(self.prev_img_gray, self.img_gray, None, 0.5, 5, 20, 5, 10, 1.5, 0)
            self.prev_img_gray = self.img_gray
            h, w = self.img_gray.shape[:2]
            y, x = np.mgrid[step/2:h:step, step/2:w:step].reshape(2, -1).astype(int)
            fx, fy = flow[y, x].T

            lines = np.vstack([x, y, x - fx, y - fy]).T.reshape(-1, 2, 2)
            lines = np.int32(lines + 0.5)

            img_bgr = cv2.cvtColor(self.img_gray, cv2.COLOR_GRAY2BGR)
            cv2.polylines(img_bgr, lines, 0, (0, 255, 0))

            fast_moving_detected_below = False
            obs_detected=False
            min_x, min_y = float('inf'), float('inf')
            max_x, max_y = -float('inf'), -float('inf')
            line_above_thresh=0
            for (x1, y1), (x2, y2) in lines:
                
                cv2.circle(img_bgr, (x1, y1), 1, (0, 255, 0), -1)
                line_length = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
                
                if line_length > threshold:
                    line_above_thresh=line_above_thresh+1
                    min_x = min(min_x, min(x1, x2))
                    min_y = min(min_y, min(y1, y2))
                    max_x = max(max_x, max(x1, x2))
                    max_y = max(max_y, max(y1, y2))                
                    
            if min_x!=-float('inf') and min_y!=-float('inf') and max_x!=float('inf') and max_y!=float('inf') and line_above_thresh>10:
                self.movement_detected = True
            else:
                self.movement_detected = False
                self.counter = 0
            
            if self.movement_detected == True:
                self.counter += 1
                    
            if self.counter >= 5:
                print("OBSTACLE OBSTACLE OBSTACLE")
                pub_msg_obs.data=True
                self.publisher_obs.publish(pub_msg_obs)  
                pub_box.data=[int(min_x),int(min_y),int(max_x),int(max_y)]
                self.publisher_box.publish(pub_box)
                cv2.rectangle(img_bgr, (min_x, min_y), (max_x, max_y), (0, 0, 255), 2)
                if max_y>=self.horiz:
                    fast_moving_detected_below = True
            else:
                pub_msg_obs.data=False
                self.publisher_obs.publish(pub_msg_obs)    
                        
            if fast_moving_detected_below:
                pub_msg_stop.data=True
                self.publisher_stop.publish(pub_msg_stop)
            else:
                pub_msg_stop.data=False
                self.publisher_stop.publish(pub_msg_stop)
                
            # cv2.imshow('flow', img_bgr)
            # cv2.waitKey(1)
            
    def horizon_callback(self, msg):
        if not self.horizon_detected:
            self.horiz=msg.data
            self.horizon_detected = True
         
            
def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)
    
    minimal_subscriber.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()