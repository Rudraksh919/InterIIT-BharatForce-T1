#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import os
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import torch
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from ultralytics import YOLO


class ConeDetectorNode(Node):
    def __init__(self):
        super().__init__('cone_detector_node')

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/bcr_bot/kinect_camera/image_raw',
            self.image_callback,
            10
        )
        self.depth_sub = self.create_subscription(
            Image,
            '/bcr_bot/kinect_camera/depth/image_raw',
            self.depth_callback,
            10
        )
        self.cam_info_sub = self.create_subscription(
            CameraInfo,
            '/bcr_bot/kinect_camera/camera_info',
            self.cam_info_callback,
            10
        )

        # Publisher for 3D position
        self.position_pub = self.create_publisher(
            PointStamped, 'cone_position', 10)

        self.bridge = CvBridge()
        self.depth_image = None
        self.cam_intrinsics = None

        # Load YOLO model
        model_path = os.path.join(get_package_share_directory('bcr_bot'), 'Detectionmodels', 'cone_detection.pt')
        self.model = YOLO(model_path)
        self.get_logger().info("YOLO model loaded successfully.")

    def cam_info_callback(self, msg):
        # Store camera intrinsics
        self.cam_intrinsics = {
            'fx': msg.k[0],
            'fy': msg.k[4],
            'cx': msg.k[2],
            'cy': msg.k[5]
        }

    def depth_callback(self, msg):
        # Convert depth image to numpy array
        self.depth_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough')

    def image_callback(self, msg):
        if self.depth_image is None or self.cam_intrinsics is None:
            return  # wait for both depth and camera info

        # Convert RGB image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Run YOLO detection
        results = self.model(cv_image)

        # Run YOLOv8 detection

        if len(results) > 0:
            res = results[0]  # first result
            detections = res.boxes.xyxy.cpu().numpy()  # x1, y1, x2, y2
            confidences = res.boxes.conf.cpu().numpy()  # confidence scores
            classes = res.boxes.cls.cpu().numpy()       # class IDs

            for det, conf, cls in zip(detections, confidences, classes):
                x1, y1, x2, y2 = det
                if int(cls) == 0:  # assuming cone class = 0
                    cx = int((x1 + x2) / 2)
                    cy = int((y1 + y2) / 2)

                    # get depth
                    z = self.depth_image[cy, cx]
                    if z == 0:
                        continue

                    # back-project to 3D
                    fx = self.cam_intrinsics['fx']
                    fy = self.cam_intrinsics['fy']
                    cx_cam = self.cam_intrinsics['cx']
                    cy_cam = self.cam_intrinsics['cy']

                    x = (cx - cx_cam) * z / fx
                    y = (cy - cy_cam) * z / fy

                    # publish PointStamped
                    point_msg = PointStamped()
                    point_msg.header.stamp = self.get_clock().now().to_msg()
                    point_msg.header.frame_id = 'kinect_camera_frame'
                    point_msg.point.x = float(x)
                    point_msg.point.y = float(y)
                    point_msg.point.z = float(z)
                    self.position_pub.publish(point_msg)

                    # draw bbox
                    cv2.rectangle(cv_image, (int(x1), int(y1)),
                                  (int(x2), int(y2)), (0, 255, 0), 2)
                    cv2.putText(cv_image, f"Cone {conf:.2f}", (int(x1), int(
                        y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    # Display 3D position
                    cv2.putText(cv_image, f"Pos: x={x:.2f} y={y:.2f} z={z:.2f}m",
                                (int(x1), int(y2)+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        # Optional: show image
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
