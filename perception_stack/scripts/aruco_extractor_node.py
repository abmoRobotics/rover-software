#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from space_interface.msg import Marker, MarkerList

def matrix_to_quaternion(rotation_matrix):
    """
    Convert a 3x3 rotation matrix to a quaternion.

    Parameters:
    - rotation_matrix (numpy.ndarray): 3x3 rotation matrix.

    Returns:
    - numpy.ndarray: Quaternion in the format [w, x, y, z].
    """

    trace = rotation_matrix[0, 0] + rotation_matrix[1, 1] + rotation_matrix[2, 2]

    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) * s
        y = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) * s
        z = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) * s
    elif rotation_matrix[0, 0] > rotation_matrix[1, 1] and rotation_matrix[0, 0] > rotation_matrix[2, 2]:
        s = 2.0 * np.sqrt(1.0 + rotation_matrix[0, 0] - rotation_matrix[1, 1] - rotation_matrix[2, 2])
        w = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / s
        x = 0.25 * s
        y = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
        z = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
    elif rotation_matrix[1, 1] > rotation_matrix[2, 2]:
        s = 2.0 * np.sqrt(1.0 + rotation_matrix[1, 1] - rotation_matrix[0, 0] - rotation_matrix[2, 2])
        w = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / s
        x = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
        y = 0.25 * s
        z = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + rotation_matrix[2, 2] - rotation_matrix[0, 0] - rotation_matrix[1, 1])
        w = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / s
        x = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
        y = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
        z = 0.25 * s

    return np.array([w, x, y, z])


def my_estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
    '''
    This will estimate the rvec and tvec for each of the marker corners detected by:
       corners, ids, rejectedImgPoints = detector.detectMarkers(image)
    corners - is an array of detected corners for each detected marker in the image
    marker_size - is the size of the detected markers
    mtx - is the camera matrix
    distortion - is the camera distortion matrix
    RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
    '''
    marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
    trash = []
    rvecs = []
    tvecs = []
    for c in corners:
        nada, R, t = cv2.solvePnP(marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
        rvecs.append(R)
        tvecs.append(t)
        trash.append(nada)
    return rvecs, tvecs, trash


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('aruco_extractor_node')
        self.subscription = self.create_subscription(Image,'camera_raw',self.aruco_extractor,10)
        self.publisher = self.create_publisher(MarkerList, 'detected_aruco_markers', 10)

        self.bridge = CvBridge()
        self.mtx = np.array(((933.15867, 0, 657.59), (0, 933.1586, 400.36993), (0, 0, 1)))
        self.dist = np.array((-0.43948, 0.18514, 0, 0))
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        parameters =  cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        self.markersize=1
        self.subscription  # prevent unused variable warning

    def aruco_extractor(self, msg):
        img=self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        markerCorners, markerIds, rejectedCandidates = self.detector.detectMarkers(img)
        if len(markerCorners) > 0:
            rvecs,tvecs,trash=my_estimatePoseSingleMarkers(markerCorners,self.markersize,self.mtx,self.dist)
            markerList=MarkerList()
            for i in range(np.shape(markerIds)[0]):
                marker=Marker()
                marker.id=int(markerIds[i][0])
                marker.position.x=tvecs[i][0][0]
                marker.position.y=tvecs[i][1][0]
                marker.position.z=tvecs[i][2][0]
                r,_=cv2.Rodrigues(rvecs[0])
                R = np.eye(4)
                R[:3, :3] = r  
                R[3, 3] = 1  
                q=matrix_to_quaternion(R)
                #[w, x, y, z]
                marker.orientation.w=q[0]
                marker.orientation.x=q[1]
                marker.orientation.y=q[2]
                marker.orientation.z=q[3]

                markerList.markers.append(marker)

        
        # Set values in marker_list_msg as needed
        self.publisher.publish(markerList)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()