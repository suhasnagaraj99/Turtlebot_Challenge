import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32
import math
import numpy as np


class HorizonDetector(Node):

    def __init__(self):
        super().__init__('horizon_detector')              # Initialise node name
        self.horizon_point_buffer = []
        self.horizon = Int32()
        self.subscription = self.create_subscription(CompressedImage,
                                                     '/camera/image_raw/compressed',
                                                     self.camera_callback,
                                                     qos_profile_sensor_data)

        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Int32, '/horizon_level', 10)                # Create publisher
        self.get_logger().info("HORIZON DETECTION INITIATED")
        self.image=None

    '''Camera callback function'''
    def camera_callback(self, msg):
        bridge = CvBridge()
        self.image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        edges = self.find_edges(self.image)                                                      # Detect edges using canny edge detection
        points1, points2 = self.detect_lines(edges)                                         # Detect lines usign hough transform
        vanishing_points = self.detect_vanishing_points(points1, points2)                   # detect vanishing points
        horizon_point = self.ransac_average(points=vanishing_points)                        # Use RANSAC tondinf best horizon line
        if horizon_point != (0,0):
            if len(self.horizon_point_buffer)<=5:
                self.horizon_point_buffer.append(horizon_point)
            else:
                self.horizon.data = int(self.ransac_average(points=self.horizon_point_buffer)[1])
                # print(self.horizon.data)
                self.publisher_.publish(self.horizon)                                                # publish horizon level
                self.get_logger().info(f"HORIZON DETECTED AT: {self.horizon.data}")

    '''Function to find edges using canny edge detection'''
    def find_edges(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)                          # Convert to graysacle
        blurred = cv2.GaussianBlur(gray, (3,3), 0)                              # Apply Gaussian blur
        edges = cv2.Canny(blurred, 50, 150, None, 3)   
        return edges
    
    '''Function to detect lines using Hough transform'''
    def detect_lines(self, edges):
        lines = cv2.HoughLines(edges, 1, np.pi/180, 120, None, 0, 0)            # get hough lines
        points1 = []
        points2 = []

        if lines is not None:                                                   # check if no lines are detected
            for i in range(0, len(lines)):  
                rho = lines[i][0][0]
                theta = lines[i][0][1]
                a = math.cos(theta)
                b = math.sin(theta)
                x0 = a * rho
                y0 = b * rho
                pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))                 # First endpoint of line segment
                pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))                 # Second endpoint of line segment
                points1.append(pt1)
                points2.append(pt2)
        selected_points1 = []
        selected_points2 = []

        '''Loop to filter out vertical and horizontal lines'''
        for i in range(len(points1)):
            x1, y1 = points1[i]
            x2, y2 = points2[i]
            if (x2 != x1):                                                      # Ignore if line is perpendicular
                slope = (y2 - y1)/(x2 - x1)                                     # Caluclate sope
                slope_in_rad = math.atan2(y2-y1, x2-x1)                         # Get angle im radians
                slope_in_deg = math.degrees(slope_in_rad)                       # Get angle in degrees
                if (87<abs(slope_in_deg)<103) or 0<abs(slope_in_deg)<5:        #  Check if line is either vertical or horizontal
                    continue
                else:
                    selected_points1.append(points1[i])               # Store selected line endpoint
                    selected_points2.append(points2[i])               # Store selected line endpoint
                    # cv2.line(self.image,points1[i],points2[i],(0,0,255),2,cv2.LINE_AA)
                    # cv2.imshow("I",self.image)
                    # cv2.waitKey(1)
        return selected_points1, selected_points2
    
    '''Function to detect vanishing point'''
    def detect_vanishing_points(self, points1, points2):
        vanishing_points = []
        i = 0

        # loops to calculate intersection of every line
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

                det = np.linalg.det(A)     # Calculate ddeterminant
                if abs(det) > 0:  # Check if the matrix is singular
                    pt = np.linalg.solve(A, b)           # Solve to get the intersection of line
                    x = int(pt[0])
                    y = int(pt[1])
                    vanishing_points.append((x,y))
                j += 1
            i = i+1
        return vanishing_points
    
    '''Function to apply RANSAC to set of vanishing points to eliminate outliers'''
    def ransac_average(self, points, num_iterations=100, threshold=10.0):
        points = np.array(points)
        best_avg_point = None
        best_inliers = []

        if len(points)<2:                         # Check if only one vanishing point is present
            return (0, 0)

        for _ in range(num_iterations):
            sample_indices = np.random.choice(len(points), size=2, replace=False)             # Randomly select two points
            sample_points = points[sample_indices]
            avg_point = np.mean(sample_points, axis=0)                                        # Calculate average of the sample points
            distances = np.linalg.norm(points - avg_point, axis=1)                            # Calculate distance of all points to the average
            inliers = points[distances < threshold]                                           # Find inliers (points within threshold distance)
            if len(inliers) > len(best_inliers):                                              # Update best model if we found more inliers
                best_avg_point = np.mean(inliers, axis=0)
                best_inliers = inliers
        try:
            return tuple(best_avg_point.astype(int))
        except:
            return (0,0)



def main(args=None):
    rclpy.init(args=args)

    node = HorizonDetector()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

