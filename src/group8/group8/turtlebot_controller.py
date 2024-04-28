import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import Int64MultiArray
from std_msgs.msg import Bool

class TurtlebotController(Node):

    def __init__(self):
        super().__init__('turtlebot_controller')              # Initialise node name

        self.horizon_level = 0
        self.camera_subscription = self.create_subscription(Image,
                                                     '/camera/image_raw',
                                                     self.camera_callback,
                                                     qos_profile_sensor_data)               # Create subscriber
        
        self.horizon_subscription = self.create_subscription(Int32,
                                                             '/horizon_level',
                                                             self.horizon_callback,
                                                             10)
        
        self.camera_feed_publisher = self.create_publisher(Image,
                                                           '/camera_feed',
                                                           qos_profile_sensor_data)
        
        self.stop_box_subscription = self.create_subscription(Int64MultiArray,
                                                        '/box_stop',
                                                        self.stop_box_callback,
                                                        qos_profile_sensor_data)
        
        self.stop_subscription = self.create_subscription(Bool,
                                                '/stop',
                                                self.stop_callback,
                                                qos_profile_sensor_data)
        
        self.camera_subscription  # prevent unused variable warning
        self.horizon_subscription  # prevent unused variable warning
        self.stop_box=[]
        self.stop=False
        self.bridge = CvBridge()

    '''Camera callback funtion'''
    def camera_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')                          # Convert image from msg to bgr
        if len(self.stop_box)!=0:
            [x1,y1,x2,y2]=self.stop_box
            if self.stop==True:
                cv2.rectangle(image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        cv2.putText(image, 'Horizon', (0, self.horizon_level-10), cv2.FONT_HERSHEY_SIMPLEX, 
                            0.8, (255, 0, 0), 2, cv2.LINE_AA) 
        cv2.line(image, (0, self.horizon_level), (image.shape[1], self.horizon_level), (255, 0, 0), 2, cv2.LINE_AA)
        ros_image = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        self.camera_feed_publisher.publish(ros_image)

    def horizon_callback(self, msg):
        self.horizon_level = msg.data
        
    def stop_box_callback(self, msg):
        self.stop_box = msg.data

    def stop_callback(self, msg):
        self.stop = msg.data

def main(args=None):
    rclpy.init(args=args)

    node = TurtlebotController()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

