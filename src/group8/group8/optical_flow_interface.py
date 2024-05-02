import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

class opticalflow(Node):

    def __init__(self):
        super().__init__('opticalflow')
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.camera_callback, qos_profile_sensor_data)
        self.publisher = self.create_publisher(Bool,'/stop_optical',qos_profile_sensor_data)
        self._bridge = CvBridge()
        self._optical_flow()

    def _optical_flow(self):
        self.feature_parameters = dict(maxCorners=100, qualityLevel=0.3, minDistance=7, blockSize=7)
        self.lk_parameters = dict(
            winSize=(15, 15),
            maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03)
        )
        self.color=np.random.randint(0, 255, (100, 3))
        self.old_gray = None
        self.p0 = None
        self.mask=None

    def camera_callback(self, msg):
        stop_signal = Bool()
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.old_gray is None:
            self.old_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            self.p0 = cv2.goodFeaturesToTrack(self.old_gray, mask=None, **self.feature_parameters)
            self.mask = np.zeros_like(frame)
            return 
        
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        p1, st, err = cv2.calcOpticalFlowPyrLK(self.old_gray, frame_gray, self.p0, None, **self.lk_parameters)
        if p1 is not None and st.any():
            good_new = p1[st==1]
            good_old = self.p0[st==1]
            max_velocity = 0
            for i, (new, old) in enumerate(zip(good_new, good_old)):
                a, b = int(new[0]), int(new[1])
                c, d = int(old[0]), int(old[1])
                dx = a - c
                dy = b - d
                velocity = np.sqrt(dx**2 + dy**2)
                max_velocity = max(max_velocity, velocity)
                self.mask = cv2.line(self.mask, (a, b), (c, d), self.color[i].tolist(), 2)
                self.frame = cv2.circle(frame, (a, b), 5, self.color[i].tolist(), -1)
            
            img = cv2.add(frame, self.mask)
            cv2.imshow('frame', img)
            cv2.waitKey(1)

            if max_velocity > 10:
                stop_signal.data = True
            else:
                stop_signal.data = False

            self.old_gray = frame_gray.copy()
            self.p0 = good_new.reshape(-1, 1, 2)
            print(stop_signal.data)
        else:
            self.get_logger().info('Optical flow failed')
        
        self.publisher.publish(stop_signal)


def main(args=None):

    rclpy.init(args=args)
    of = opticalflow()

    try:
        rclpy.spin(of)
    except KeyboardInterrupt:
        of.get_logger().info('Keyboard Interrupt, Exiting opticalflow node')
    finally:
        of.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()