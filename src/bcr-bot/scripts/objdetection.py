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


class ObjectDetectorNode(Node):
    def __init__(self):
        super().__init__('obj_detector_node')

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
            PointStamped, 'object_position', 10)

        self.bridge = CvBridge()
        self.depth_image = None
        self.cam_intrinsics = None

        # Load YOLO model
        coco_model_path = os.path.join(get_package_share_directory('bcr_bot'), 'Detectionmodels', 'yolov8x.pt') # Pre-trained COCO model
        cone_model_path = os.path.join(get_package_share_directory('bcr_bot'), 'Detectionmodels', 'cone_detection.pt') # Custom-trained model for cones
        self.cone_model = YOLO(cone_model_path)
        self.coco_model = YOLO(coco_model_path)  # Pre-trained COCO model        
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
        coco_results = self.coco_model(cv_image , verbose=False)[0]
        cone_results = self.cone_model(cv_image , verbose=False)[0]
        all_results = [coco_results , cone_results]

        # Run YOLOv8 detection

        for res in all_results:
            if res.boxes is None:
                continue

            detections = res.boxes.xyxy.cpu().numpy()  # x1, y1, x2, y2
            confidences = res.boxes.conf.cpu().numpy()
            classes = res.boxes.cls.cpu().numpy()

            for det, conf, cls in zip(detections, confidences, classes):
                x1, y1, x2, y2 = det
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)

                # Get depth
                z = self.depth_image[cy, cx]
                if z == 0:
                    continue

                fx = self.cam_intrinsics['fx']
                fy = self.cam_intrinsics['fy']
                cx_cam = self.cam_intrinsics['cx']
                cy_cam = self.cam_intrinsics['cy']

                X = (cx - cx_cam) * z / fx
                Y = (cy - cy_cam) * z / fy

                # Publish 3D point
                point_msg = PointStamped()
                point_msg.header.stamp = self.get_clock().now().to_msg()
                point_msg.header.frame_id = 'kinect_camera_frame'
                point_msg.point.x = float(X)
                point_msg.point.y = float(Y)
                point_msg.point.z = float(z)
                self.position_pub.publish(point_msg)

                # Draw bounding box
                cv2.rectangle(cv_image, (int(x1), int(y1)),
                            (int(x2), int(y2)), (0, 255, 0), 2)
                label = f"{self.coco_model.names[int(cls)] if res==coco_results else 'cone'} {conf:.2f}"
                cv2.putText(cv_image, label, (int(x1), int(y1)-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Show 3D position
                cv2.putText(cv_image, f"Pos: x={X:.2f} y={Y:.2f} z={z:.2f}m",
                            (int(x1), int(y2)+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

                # Optional: show image
                cv2.imshow("Object Detection", cv_image)
                cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
