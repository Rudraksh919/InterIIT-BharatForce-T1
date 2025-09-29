#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import math

def quaternion_to_yaw(qx, qy, qz, qw):
    """Convert quaternion to yaw (rotation around Z-axis)."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy*qy + qz*qz)
    return math.atan2(siny_cosp, cosy_cosp)

class ConeDetectorNode(Node):
    def __init__(self):
        super().__init__('cone_detector_node')

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/bcr_bot/kinect_camera/image_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/bcr_bot/kinect_camera/depth/image_raw', self.depth_callback, 10)
        self.cam_info_sub = self.create_subscription(
            CameraInfo, '/bcr_bot/kinect_camera/camera_info', self.cam_info_callback, 10)
        self.bot_pose_sub = self.create_subscription(
            PoseStamped, '/bot/location', self.bot_pose_callback, 10)

        # Publishers
        self.position_pub = self.create_publisher(PointStamped, '/cone_position', 10)
        self.global_pub = self.create_publisher(PointStamped, '/cone_obj/global_pose', 10)

        self.bridge = CvBridge()
        self.depth_image = None
        self.cam_intrinsics = None
        self.bot_pos = None
        self.bot_yaw = None

        # Load YOLO model
        self.model = YOLO("best.pt")
        self.get_logger().info("YOLO model loaded successfully.")

    def cam_info_callback(self, msg):
        self.cam_intrinsics = {
            'fx': msg.k[0],
            'fy': msg.k[4],
            'cx': msg.k[2],
            'cy': msg.k[5]
        }

    def depth_callback(self, msg):
        # Convert depth image to meters if needed
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        if self.depth_image.max() > 20:  # assume depth in mm
            self.depth_image = self.depth_image / 1000.0  # convert to meters

    def bot_pose_callback(self, msg):
        self.bot_pos = np.array([msg.pose.position.x,
                                 msg.pose.position.y,
                                 msg.pose.position.z])
        q = msg.pose.orientation
        self.bot_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def image_callback(self, msg):
        if self.depth_image is None or self.cam_intrinsics is None or self.bot_pos is None:
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(cv_image)

        if len(results) > 0:
            res = results[0]
            detections = res.boxes.xyxy.cpu().numpy()
            confidences = res.boxes.conf.cpu().numpy()
            classes = res.boxes.cls.cpu().numpy()

            for det, conf, cls in zip(detections, confidences, classes):
                x1, y1, x2, y2 = det
                if int(cls) != 0:  # only cone
                    continue

                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)

                # Get depth
                z_depth = self.depth_image[cy, cx]
                if z_depth == 0:
                    continue

                fx = self.cam_intrinsics['fx']
                fy = self.cam_intrinsics['fy']
                cx_cam = self.cam_intrinsics['cx']
                cy_cam = self.cam_intrinsics['cy']

                # Camera frame coordinates (Kinect: X-right, Y-down, Z-forward)
                x_cam = (cx - cx_cam) * z_depth / fx
                y_cam = (cy - cy_cam) * z_depth / fy
                z_cam = z_depth

                # Publish camera-relative position
                point_msg = PointStamped()
                point_msg.header.stamp = self.get_clock().now().to_msg()
                point_msg.header.frame_id = 'kinect_camera_frame'
                point_msg.point.x = float(x_cam)
                point_msg.point.y = float(y_cam)
                point_msg.point.z = float(z_cam)
                self.position_pub.publish(point_msg)

                # Transform to robot frame
                x_robot = z_cam        # camera forward → robot forward
                y_robot = -x_cam       # camera right → robot left
                z_robot = -y_cam       # camera down → robot up
                cone_robot_frame = np.array([x_robot, y_robot, z_robot])

                # Rotate by robot yaw and add global position
                R_yaw = np.array([
                    [np.cos(self.bot_yaw), -np.sin(self.bot_yaw), 0],
                    [np.sin(self.bot_yaw),  np.cos(self.bot_yaw), 0],
                    [0, 0, 1]
                ])
                cone_global = R_yaw @ cone_robot_frame + self.bot_pos

                # Publish global position
                global_msg = PointStamped()
                global_msg.header.stamp = self.get_clock().now().to_msg()
                global_msg.header.frame_id = 'map'
                global_msg.point.x = float(cone_global[0])
                global_msg.point.y = float(cone_global[1])
                global_msg.point.z = float(cone_global[2])
                self.global_pub.publish(global_msg)

                # Draw bbox and text
                cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(cv_image, f"Cone {conf:.2f}", (int(x1), int(y1)-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(cv_image, f"Global: x={cone_global[0]:.2f} y={cone_global[1]:.2f} z={cone_global[2]:.2f}",
                            (int(x1), int(y2)+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        cv2.imshow("Cone Detection", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ConeDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
