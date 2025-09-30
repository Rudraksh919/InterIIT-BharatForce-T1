#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
from functools import partial
from ultralytics import YOLO
import os
from ament_index_python.packages import get_package_share_directory

class SwarmObjectDetector(Node):
    def __init__(self):
        super().__init__('swarm_obj_detector')

        # Parameter: list of robot namespaces (e.g. ["bcr_bot_1", "bcr_bot_2"])
        self.declare_parameter('robot_namespaces', ['bcr_bot_1', 'bcr_bot_2', 'bcr_bot_3'])
        self.robot_namespaces = self.get_parameter('robot_namespaces').get_parameter_value().string_array_value
        if not self.robot_namespaces:
            # fallback if param wasn't set as string_array
            self.robot_namespaces = self.get_parameter('robot_namespaces').get_parameter_value().string_value.split(',')

        # Optionally model paths (custom or coco)
        pkg_share = get_package_share_directory('bcr_bot')
        cone_model_path = os.path.join(pkg_share, 'Detectionmodels', 'cone_detection.pt')
        # load models once
        self.get_logger().info('Loading YOLO models ...')
        self.coco_model = YOLO("yolov8x.pt")   # ensure this path/model is accessible in your environment
        self.cone_model = YOLO(cone_model_path)
        self.get_logger().info('YOLO models loaded')

        # cv bridge
        self.bridge = CvBridge()

        # Per-robot storage
        self.robots = {}
        for ns in self.robot_namespaces:
            self.robots[ns] = {
                'depth': None,
                'intrinsics': None,
                'image_topic': f'/{ns}/kinect_camera/image_raw',
                'depth_topic': f'/{ns}/kinect_camera/depth/image_raw',
                'caminfo_topic': f'/{ns}/kinect_camera/camera_info',
                'pos_pub': self.create_publisher(PointStamped, f'/{ns}/object_position', 10)
            }
            # Create subscriptions with callbacks that know the namespace
            self.create_subscription(Image, self.robots[ns]['image_topic'],
                                     partial(self.image_callback, ns), 10)
            self.create_subscription(Image, self.robots[ns]['depth_topic'],
                                     partial(self.depth_callback, ns), 10)
            self.create_subscription(CameraInfo, self.robots[ns]['caminfo_topic'],
                                     partial(self.caminfo_callback, ns), 10)

        # Optionally show images (one window reused). If you have many robots, consider disabling display.
        self.show_windows = True

    # Callbacks bound to namespace via partial
    def caminfo_callback(self, ns, msg):
        intr = {
            'fx': msg.k[0],
            'fy': msg.k[4],
            'cx': msg.k[2],
            'cy': msg.k[5]
        }
        self.robots[ns]['intrinsics'] = intr

    def depth_callback(self, ns, msg):
        try:
            depth_cv = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f"{ns} depth conversion failed: {e}")
            return
        self.robots[ns]['depth'] = depth_cv

    def image_callback(self, ns, msg):
        robot = self.robots[ns]
        if robot['depth'] is None or robot['intrinsics'] is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"{ns} image conversion failed: {e}")
            return

        # Run detectors (synchronous; if slow, consider moving inference to a thread pool)
        # You can combine or prioritize models as desired
        coco_res = self.coco_model(cv_image, verbose=False)[0]
        cone_res = self.cone_model(cv_image, verbose=False)[0]
        results = [(coco_res, 'coco'), (cone_res, 'cone')]

        for res, tag in results:
            if getattr(res, 'boxes', None) is None:
                continue
            boxes = res.boxes.xyxy.cpu().numpy()
            confs = res.boxes.conf.cpu().numpy()
            classes = res.boxes.cls.cpu().numpy()

            for (x1, y1, x2, y2), conf, cls in zip(boxes, confs, classes):
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)

                # Clip indices
                h, w = robot['depth'].shape[:2]
                cx_clamped = max(0, min(w - 1, cx))
                cy_clamped = max(0, min(h - 1, cy))

                z = float(robot['depth'][cy_clamped, cx_clamped])
                if z == 0.0 or z != z:  # skip invalid
                    continue

                fx = robot['intrinsics']['fx']
                fy = robot['intrinsics']['fy']
                cx_cam = robot['intrinsics']['cx']
                cy_cam = robot['intrinsics']['cy']

                X = (cx - cx_cam) * z / fx
                Y = (cy - cy_cam) * z / fy

                # Publish
                pt = PointStamped()
                pt.header.stamp = self.get_clock().now().to_msg()
                pt.header.frame_id = f'{ns}/kinect_camera_frame'
                pt.point.x = float(X)
                pt.point.y = float(Y)
                pt.point.z = float(z)
                robot['pos_pub'].publish(pt)

                # Draw box & label on image
                label = f"{(self.coco_model.names[int(cls)] if tag=='coco' else 'cone')} {conf:.2f}"
                cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(cv_image, label, (int(x1), int(y1)-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
                cv2.putText(cv_image, f"Pos: x={X:.2f} y={Y:.2f} z={z:.2f}m",
                            (int(x1), int(y2)+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 2)

        # Display (single window per robot can be heavy for many robots)
        if self.show_windows:
            winname = f"Detections - {ns}"
            cv2.imshow(winname, cv_image)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = SwarmObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()


