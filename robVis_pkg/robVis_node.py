import cv2
import cv2.aruco as aruco
import numpy as np
import glob

from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
import tf_transformations
import transformations as tf
from scipy.spatial.transform import Rotation as R
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster



#Token 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


path = "cali.yml"
images_gray = []
mtx = []
dist = []
rvecs = []
tvecs = []
calibration_d = False
hasCalibrated = False
foundArucosMarkers = 0
# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*9,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

camera_pos = [0.96, 0.54, 2.16]
camera_rot = [-3.14159265, 0, 3.14159265]
arucoHeight = 0.1 # Given in meters


def findArucosMakers(img, makerSize=6, totalMarkers=250, draw=False):
    '''Finds aruco markers in an image with the given key.'''

    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # We use the key 6x6_250 for the Aruco markers type
    key = getattr(aruco, f'DICT_{makerSize}X{makerSize}_{totalMarkers}')
    arucoDict = aruco.Dictionary_get(key)
    arucoParam = aruco.DetectorParameters_create()
    bbox, ids, rejected = aruco.detectMarkers(imgGray, arucoDict, parameters=arucoParam)

    if draw:
        aruco.drawDetectedMarkers(img, bbox)

    return[bbox, ids]


def save_coefficients(mtx, dist):
    '''Saves the coefficients from a camera calibration.'''
    global path

    """ Save the camera matrix and the distortion coefficients to given path/file. """
    cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
    cv_file.write("K", mtx)
    cv_file.write("D", dist)
    # note you *release* you don't close() a FileStorage object
    cv_file.release()


def load_coefficients():
    '''Loads the coefficients from a calibration file.'''

    global path

    """ Loads camera matrix and distortion coefficients. """
    # FILE_STORAGE_READ
    cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)

    camera_matrix = cv_file.getNode("K").mat()
    dist_matrix = cv_file.getNode("D").mat()

    cv_file.release()
    return camera_matrix, dist_matrix


class StaticFramePublisher(Node):
   """
   Broadcast transforms that never change.

   This publishes transforms from `world` to a static camera frame.
   The transforms are only published once at startup, and are constant for all
   time.

   """

   def __init__(self):
      super().__init__('static_camera_frame')

      self._tf_publisher = StaticTransformBroadcaster(self)

      # Publish static transforms once at startup
      self.make_transforms()

   def make_transforms(self):

        global camera_pos, camera_rot

        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = self.get_clock().now().to_msg()
        static_transformStamped.header.frame_id = 'world'
        static_transformStamped.child_frame_id = 'cam'
        static_transformStamped.transform.translation.x = float(camera_pos[0])
        static_transformStamped.transform.translation.y = float(camera_pos[1])
        static_transformStamped.transform.translation.z = float(camera_pos[2])
        quat = tf_transformations.quaternion_from_euler(
                float(camera_rot[0]), float(camera_rot[1]), float(camera_rot[2]))
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]

        print("Finished!")

        self._tf_publisher.sendTransform(static_transformStamped)
        exit()


class DynamicFrameBroadcaster(Node):

    '''Broadcasts the found AruCo markers as frames.'''

    def __init__(self):
        super().__init__('dynamic_frame_tf2_broadcaster')

        print("Node initialized!")
        
        print("Starting Broadcaster...")
        self.br = TransformBroadcaster(self)
        print("Broadcaster started!")
        self.timer = self.create_timer(0.01, self.handle_rob_pose)


    def handle_rob_pose(self):

        global rvecs, tvecs, foundArucosMarkers, arucoHeight


        # Set rotation and transforms
        # Aruco: tvecs[0][0][1] -> tvecs[ArucomarkerID][Cornor][Transform] maybe? 
        for i in range(foundArucosMarkers):
            
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'cam'
            t.child_frame_id = 'aruco' + str(i)

            try:
                tx = tvecs[i][0][0]
                ty = tvecs[i][0][1]
                tz = tvecs[i][0][2]

                # Get the prober rotation. Converts to quaternion from rotation vectors.
                r = R.from_rotvec(np.array([rvecs[i][0][0], rvecs[i][0][1], rvecs[i][0][2]]))
                qua = R.as_quat(r)

                rx = qua[1]
                ry = qua[2]
                rz = qua[3]
                w = qua[0]

                #print("Rotation aruco:" + str(rx) + ", " + str(ry) + ", " + str(rz) + ", " + str(w))

                t.transform.translation.x = tx
                t.transform.translation.y = ty
                t.transform.translation.z = tz
                t.transform.rotation.x = rx
                t.transform.rotation.y = ry
                t.transform.rotation.z = rz
                t.transform.rotation.w = w

            except Exception as e:
                t.transform.translation.x = 0.0
                t.transform.translation.y = 0.0
                t.transform.translation.z = 0.0
                t.transform.rotation.x = 0.0
                t.transform.rotation.y = 0.0
                t.transform.rotation.z = 0.0
                t.transform.rotation.w = 1.0
                

                print("Something went wrong with the transform!")
                print(e)

            self.br.sendTransform(t)


class PosePublisher(Node):

    '''Publishes the found AruCo markers position and orientation as a PoseWithCovariance to be used in an EKF filter.'''

    # https://get-help.robotigniteacademy.com/t/ros2-navigation-for-dummies/12825/5
    # https://blog.hadabot.com/ros2-navigation-tf2-tutorial-using-turtlesim.html

    def __init__(self):
        super().__init__('acuro_pos')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, 'pose', 10)
        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.hasSetStartFrameOffset = False
        self.offset = [0, 0, 0]

    def timer_callback(self):

        global rvecs, tvecs, foundArucosMarkers, camera_pos, camera_rot, arucoHeight

        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'odom'
        pose.header.stamp = self.get_clock().now().to_msg()

        # Create an offset so that when the program starts, thats the zero point in space
        if not self.hasSetStartFrameOffset and foundArucosMarkers > 0:
            self.offset = [tvecs[0][0][0], tvecs[0][0][1], tvecs[0][0][2]]
            self.hasSetStartFrameOffset = True
            print("Has set the offset!")

        try: 
            tx = tvecs[0][0][0] - self.offset[0]
            ty = tvecs[0][0][1] - self.offset[1]
            tz = tvecs[0][0][2] - self.offset[2]

            # Get the proper rotation. Converts to quaternion from rotation vectors.
            r = R.from_rotvec(np.array([rvecs[0][0][2], rvecs[0][0][1], rvecs[0][0][0]]))
            p = R.as_euler(r, seq='xyz', degrees=True)
            r = R.from_euler(seq='zyx', angles=p, degrees=True)
            qua = R.as_quat(r)

            rx = qua[0]
            ry = qua[1]
            rz = qua[2]
            w = qua[3]

            pose.pose.pose.position.x = tx
            pose.pose.pose.position.y = -ty
            pose.pose.pose.position.z = tz

            pose.pose.pose.orientation.x = rx
            pose.pose.pose.orientation.y = ry
            pose.pose.pose.orientation.z = rz
            pose.pose.pose.orientation.w = w

            # https://github.com/methylDragon/ros-sensor-fusion-tutorial/blob/master/02%20-%20Global%20Pose%20Estimate%20Fusion%20(Example%20Implementation).md

            pose.pose.covariance = [0.017, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.017, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.017, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.017, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.017, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.017]

        except Exception as e:
            pose.pose.pose.position.x = 0.0
            pose.pose.pose.position.y = 0.0
            pose.pose.pose.position.z = 0.0

            pose.pose.pose.orientation.x = 0.0
            pose.pose.pose.orientation.y = 0.0
            pose.pose.pose.orientation.z = 0.0
            pose.pose.pose.orientation.w = 0.0    

            pose.pose.covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

            print("Something went wrong with the pose!")
            print(e)

        self.publisher_.publish(pose)



def main(args=None):
    
    global rvecs, tvecs, foundArucosMarkers, calibration_d, hasCalibrated

    # Init ros
    rclpy.init(args=args)

    # Start Camera
    cap = cv2.VideoCapture(0) # Normal Camera
    #cap2 = cv2.VideoCapture(2)

    # Start node

    # publish camera position and orientation, only do this once when a new RVIZ is started. Just a quick fix for now
    #node_camPublish = StaticFramePublisher()
    #rclpy.spin_once(node_camPublish)

    while True:
        success, img = cap.read() # Normal Camera
        #success2, img2 = cap2.read()
        if calibration_d:
            img_g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            if len(images_gray) < 50:
                print("Waiting for input...")
                cv2.imshow("Image", img)
                if cv2.waitKey(33) == ord('a'):
                    images_gray.append(img_g)
                    print("Image captured")
                    print(len(images_gray))
            else:
                print("Calibrating...")
                for img_g in images_gray:
                    # Find the chess board corners
                    ret, corners = cv2.findChessboardCorners(img_g, (9, 6), None)
                    # If found, add object points, image points (after refining them)
                    if ret == True:
                        objpoints.append(objp)
                        corners2 = cv2.cornerSubPix(img_g, corners, (11, 11), (-1, -1), criteria)
                        imgpoints.append(corners)
                        # Draw and display the corners
                        cv2.drawChessboardCorners(img_g, (9, 6), corners2, ret)
                        cv2.imshow('img', img_g)
                        # cv2.waitKey(500)

                ret, mtx, dist, _, __ = cv2.calibrateCamera(objpoints, imgpoints, img_g.shape[::-1], None, None)
                calibration_d = False
                save_coefficients(mtx, dist)
                print("Done calibrating!")
                cv2.destroyWindow("Image")
                # Start the broadcaster
        else:

            if not hasCalibrated:
                mtx, dist = load_coefficients()
                # Make new Node object
                node = DynamicFrameBroadcaster()
                poseNode = PosePublisher()
                hasCalibrated = True

            foundArucos = findArucosMakers(img)
            foundArucosMarkers = len(foundArucos[0])
            if foundArucosMarkers > 0:
                aruco.drawDetectedMarkers(img, foundArucos[0], foundArucos[1])
                counter = 0
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(foundArucos[0], 0.173, mtx, dist) # Aruco markers length are given in meters

                for bbox, id in zip(foundArucos[0], foundArucos[1]):
                    aruco.drawAxis(img, mtx, dist, rvecs[counter], tvecs[counter], 0.1)
                    counter += 1

                rclpy.spin_once(node)
                rclpy.spin_once(poseNode)
            else:
                print("No Aruco markers found")
            
            cv2.imshow("Aruco Markers", img)
            #cv2.imshow("Second Camera", img2)
            cv2.waitKey(1)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
