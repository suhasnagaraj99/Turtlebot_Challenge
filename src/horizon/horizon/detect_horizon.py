import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
import math
import numpy as np


class HorizonDetector(Node):

    def __init__(self):
        super().__init__('horizon_detector')
        self.subscription = self.create_subscription(Image,'/camera/image_raw',self.camera_callback,qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Int32, '/horizon_level', 10)

    def camera_callback(self, msg):
        horizon = Int32()
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        edges = self.find_edges(image)
        points1, points2 = self.detect_lines(edges)
        vanishing_points = self.detect_vanishing_points(points1, points2)
        horizon_point = self.ransac_average(points=vanishing_points)
        horizon.data = int(horizon_point[1])
        self.publisher_.publish(horizon)
        # print(horizon_point[1])
        # cv2.line(image, (0, horizon_point[1]), (image.shape[1] ,horizon_point[1]), (255, 0, 0), 2, cv2.LINE_AA)
        # cv2.circle(image, horizon_point, 5, (0,0,255), -1)
        # cv2_imshow(image)

    def find_edges(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (3,3), 0)
        edges = cv2.Canny(blurred, 50, 150, None, 3)
        return edges
    
    def detect_lines(self, edges):
        lines = cv2.HoughLines(edges, 1, np.pi/180, 120, None, 0, 0)
        points1 = []
        points2 = []

        # Draw the lines
        if lines is not None:
            for i in range(0, len(lines)):
                rho = lines[i][0][0]
                theta = lines[i][0][1]
                a = math.cos(theta)
                b = math.sin(theta)
                x0 = a * rho
                y0 = b * rho
                pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
                pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
                points1.append(pt1)
                points2.append(pt2)

        selected_points1 = []
        selected_points2 = []
        for i in range(len(points1)):
            x1, y1 = points1[i]
            x2, y2 = points2[i]
            if (x2 != x1):
                slope = (y2 - y1)/(x2 - x1)
                slope_in_rad = math.atan2(y2-y1, x2-x1)
                slope_in_deg = math.degrees(slope_in_rad)
                if (80<abs(slope_in_deg)<110) or 0<abs(slope_in_deg)<10:
                    continue
                else:
                    selected_points1.append(points1[i])
                    selected_points2.append(points2[i])

        return selected_points1, selected_points2
    
    def detect_vanishing_points(self, points1, points2):
        vanishing_points = []
        i = 0
        while i<(len(points1)):
            x1 = points1[i][0]
            y1 = points1[i][1]
            x2 = points2[i][0]
            y2 = points2[i][1]
            j = i + 1
            while(j<len(points1)):
                x3 = points1[j][0]
                y3 = points1[j][1]
                x4 = points2[j][0]
                y4 = points2[j][1]

                A = np.array([[y1-y2, x2-x1],
                                [y3-y4, x4-x3]])
                b = np.array([(x2-x1)*y1 - (y2-y1)*x1, (x4-x3)*y3 - (y4-y3)*x3])

                det = np.linalg.det(A)
                if abs(det) > 0:  # You can adjust the threshold as needed
                    pt = np.linalg.solve(A, b)
                    x = int(pt[0])
                    y = int(pt[1])
                    vanishing_points.append((x,y))
                j += 1
            i = i+1
        return vanishing_points
    
    def ransac_average(self, points, num_iterations=100, threshold=10.0):
        points = np.array(points)
        best_avg_point = None
        best_inliers = []

        if len(points)<2:
            return tuple(points[0].astype(int))

        for _ in range(num_iterations):
            # Randomly select two points
            sample_indices = np.random.choice(len(points), size=2, replace=False)
            sample_points = points[sample_indices]

            # Calculate average of the sample points
            avg_point = np.mean(sample_points, axis=0)

            # Calculate distance of all points to the average
            distances = np.linalg.norm(points - avg_point, axis=1)

            # Find inliers (points within threshold distance)
            inliers = points[distances < threshold]

            # Update best model if we found more inliers
            if len(inliers) > len(best_inliers):
                best_avg_point = np.mean(inliers, axis=0)
                best_inliers = inliers
        try:
            return tuple(best_avg_point.astype(int))
        except:
            return (0,0)



def main(args=None):
    rclpy.init(args=args)

    print('Hi from horizon.')

    node = HorizonDetector()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

