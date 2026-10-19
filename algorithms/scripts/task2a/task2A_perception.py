#!/usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		        StrataCobot (SC) Theme (eYRC 2026-27)
*        		===============================================
*
*  This script implements Task 2A (Perception) of StrataCobot (SC) Theme (eYRC 2026-27).
*  Detects the 4 ores on the conveyor using colour and aligned depth, and broadcasts
*  a tf2 transform for each ore relative to 'base_link' continuously while on conveyor.
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:          [ Team-ID ]
# Author List:      [ e-Yantra StrataCobot Team ]
# Filename:         task2A_perception.py
# Functions:        detect_ores, transform_point_to_base, main
# Nodes:            ore_tf_publisher
#                   Publishing Topics  - [ /tf ]
#                   Subscribing Topics - [ /camera/camera/color/image_raw,
#                                          /camera/camera/aligned_depth_to_color/image_raw,
#                                          /camera/camera/color/camera_info ]


################### IMPORT MODULES #######################

import sys
import rclpy
import cv2
import tf2_ros
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image


##################### TASK CONSTANTS #######################

ore_types = ['azurite_ore', 'malachite_ore', 'vanadinite_ore']

# RealSense camera topics (depth aligned to colour)
color_topic = '/camera/camera/color/image_raw'
depth_topic = '/camera/camera/aligned_depth_to_color/image_raw'
camera_info_topic = '/camera/camera/color/camera_info'

base_frame = 'base_link'

# Conveyor region limits in base_link coordinates (metres)
CONVEYOR_X_MIN = -0.90
CONVEYOR_X_MAX = 0.35
CONVEYOR_Y_MIN = -0.75
CONVEYOR_Y_MAX = -0.30
CONVEYOR_Z_MIN = 0.10
CONVEYOR_Z_MAX = 0.30


##################### FUNCTION DEFINITIONS #######################

def transform_point_to_base(pt_cam, transform_stamped):
    '''
    Description:    Transforms a 3D point from optical frame into base_link frame
                    using quaternion rotation and translation.

    Args:
        pt_cam              (tuple/list):   3D coordinates (x, y, z) in optical camera frame
        transform_stamped   (TransformStamped): Transform from base_link to camera frame

    Returns:
        p_base              (np.ndarray):   3D coordinates in base_link
    '''
    tx = transform_stamped.transform.translation.x
    ty = transform_stamped.transform.translation.y
    tz = transform_stamped.transform.translation.z
    qx = transform_stamped.transform.rotation.x
    qy = transform_stamped.transform.rotation.y
    qz = transform_stamped.transform.rotation.z
    qw = transform_stamped.transform.rotation.w

    qv = np.array([qx, qy, qz], dtype=np.float64)
    v = np.array(pt_cam, dtype=np.float64)
    v_rot = v + 2.0 * np.cross(qv, np.cross(qv, v) + qw * v)
    return v_rot + np.array([tx, ty, tz], dtype=np.float64)


def detect_ores(image):
    '''
    Description:    Function to detect the ores present in a colour image frame and
                    return the pixel location and the type of each one found.

    Args:
        image                   (np.ndarray): Input BGR colour image frame received from camera topic

    Returns:
        center_ore_list         (list):       Center pixel (cX, cY) of every ore detected in the frame
        ore_type_list           (list):       Type of each ore detected, taken from 'ore_types'
    '''
    center_ore_list = []
    ore_type_list = []

    if image is None:
        return center_ore_list, ore_type_list

    # Convert BGR frame to HSV colour space
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # HSV thresholds for Gazebo lighting conditions
    color_ranges = {
        'azurite_ore': [
            (np.array([100, 70, 30], dtype=np.uint8), np.array([135, 255, 255], dtype=np.uint8))
        ],
        'malachite_ore': [
            (np.array([35, 70, 30], dtype=np.uint8), np.array([85, 255, 255], dtype=np.uint8))
        ],
        'vanadinite_ore': [
            (np.array([0, 70, 30], dtype=np.uint8), np.array([22, 255, 255], dtype=np.uint8)),
            (np.array([160, 70, 30], dtype=np.uint8), np.array([180, 255, 255], dtype=np.uint8))
        ]
    }

    kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    kernel_open = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

    for ore_type, ranges in color_ranges.items():
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in ranges:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))

        # Morphological operations to clean up mask and eliminate noise
        cleaned_mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close)
        cleaned_mask = cv2.morphologyEx(cleaned_mask, cv2.MORPH_OPEN, kernel_open)

        contours, _ = cv2.findContours(cleaned_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            # Filter contours by plausible ore pixel area
            if 200 < area < 15000:
                M = cv2.moments(cnt)
                if M['m00'] > 0:
                    cX = int(M['m10'] / M['m00'])
                    cY = int(M['m01'] / M['m00'])
                    center_ore_list.append((cX, cY))
                    ore_type_list.append(ore_type)

    return center_ore_list, ore_type_list


##################### CLASS DEFINITION #######################

class ore_tf(Node):
    '''
    ___CLASS___

    Description:    Node that detects ores on the conveyor and continuously broadcasts
                    a transform for each ore relative to 'base_link'.
    '''

    def __init__(self):
        '''
        Description:    Initialization of class ore_tf
        '''
        super().__init__('ore_tf_publisher')

        # Topic Subscriptions
        self.color_cam_sub = self.create_subscription(Image, color_topic, self.colorimagecb, 10)
        self.depth_cam_sub = self.create_subscription(Image, depth_topic, self.depthimagecb, 10)
        self.cam_info_sub = self.create_subscription(CameraInfo, camera_info_topic, self.caminfocb, 10)

        # Transformation and CV bridge
        image_processing_rate = 0.1  # 10 Hz processing rate
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(image_processing_rate, self.process_image)

        self.cv_image = None
        self.depth_image = None
        self.cam_info = None

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.cam_frame = None

        # Tracked ores dictionary: key is ore_type, value is list of tracked ore dicts
        # Each dict: {'id': int, 'pos': np.ndarray, 'miss_count': int}
        self.tracked_ores = {t: [] for t in ore_types}

    def depthimagecb(self, data):
        '''
        Description:    Callback function for the aligned depth camera topic.
        '''
        try:
            depth = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
            if depth.dtype == np.uint16:
                self.depth_image = depth.astype(np.float32) / 1000.0
            else:
                self.depth_image = depth.astype(np.float32)
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge depth error: {e}")

    def colorimagecb(self, data):
        '''
        Description:    Callback function for the colour camera raw topic.
        '''
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge color error: {e}")

    def caminfocb(self, data):
        '''
        Description:    Callback function for the camera info topic.
        '''
        self.cam_info = data
        self.cam_frame = data.header.frame_id
        # K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        self.fx = float(data.k[0])
        self.cx = float(data.k[2])
        self.fy = float(data.k[4])
        self.cy = float(data.k[5])

    def process_image(self):
        '''
        Description:    Timer function used to detect ores and publish transforms.
        '''
        # Return early until images and intrinsics are received
        if (self.cv_image is None or self.depth_image is None or
                self.cam_info is None or self.fx is None):
            return

        # Lookup transform from camera optical frame to base_link
        try:
            tf_cam_to_base = self.tf_buffer.lookup_transform(
                base_frame, self.cam_frame, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return

        centers, types = detect_ores(self.cv_image)
        h, w = self.depth_image.shape[:2]

        current_frame_ores = {t: [] for t in ore_types}

        for (cX, cY), ore_type in zip(centers, types):
            # Sample median depth from 9x9 patch around centroid
            patch = self.depth_image[max(0, cY - 4):min(h, cY + 5),
                                     max(0, cX - 4):min(w, cX + 5)]
            valid = patch[np.isfinite(patch) & (patch > 0.1) & (patch < 5.0)]
            if len(valid) < 5:
                continue

            z = float(np.median(valid))
            # Deproject into camera optical frame
            xc = (cX - self.cx) * z / self.fx
            yc = (cY - self.cy) * z / self.fy
            zc = z

            # Transform into base_link frame
            p_base = transform_point_to_base((xc, yc, zc), tf_cam_to_base)

            # Check if detection lies within conveyor volume
            if (CONVEYOR_X_MIN <= p_base[0] <= CONVEYOR_X_MAX and
                    CONVEYOR_Y_MIN <= p_base[1] <= CONVEYOR_Y_MAX and
                    CONVEYOR_Z_MIN <= p_base[2] <= CONVEYOR_Z_MAX):
                current_frame_ores[ore_type].append(p_base)

        # Update persistent tracking for each ore kind
        for ore_type in ore_types:
            detected_positions = current_frame_ores[ore_type]
            matched_indices = set()

            for tracked in self.tracked_ores[ore_type]:
                best_match_idx = None
                best_dist = float('inf')
                for i, pos in enumerate(detected_positions):
                    if i in matched_indices:
                        continue
                    dist = float(np.linalg.norm(tracked['pos'] - pos))
                    if dist < 0.12 and dist < best_dist:
                        best_dist = dist
                        best_match_idx = i

                if best_match_idx is not None:
                    matched_indices.add(best_match_idx)
                    # Smooth update position with exponential moving average
                    tracked['pos'] = 0.8 * tracked['pos'] + 0.2 * detected_positions[best_match_idx]
                    tracked['miss_count'] = 0
                else:
                    tracked['miss_count'] += 1

            # Register new ores for unmatched detections
            for i, pos in enumerate(detected_positions):
                if i not in matched_indices:
                    # Allocate next unused ID within this ore type (starting from 1)
                    used_ids = {t['id'] for t in self.tracked_ores[ore_type]}
                    next_id = 1
                    while next_id in used_ids:
                        next_id += 1
                    self.tracked_ores[ore_type].append({
                        'id': next_id,
                        'pos': pos,
                        'miss_count': 0
                    })

        # Broadcast TF transforms for each tracked ore on conveyor
        now_msg = self.get_clock().now().to_msg()
        for ore_type in ore_types:
            for tracked in self.tracked_ores[ore_type]:
                # Stop broadcasting when ore has been picked up (missing for > 10 cycles = 1.0s)
                if tracked['miss_count'] <= 10:
                    t = TransformStamped()
                    t.header.stamp = now_msg
                    t.header.frame_id = base_frame
                    t.child_frame_id = f"{ore_type}_{tracked['id']}"
                    t.transform.translation.x = float(tracked['pos'][0])
                    t.transform.translation.y = float(tracked['pos'][1])
                    t.transform.translation.z = float(tracked['pos'][2])
                    t.transform.rotation.x = 0.0
                    t.transform.rotation.y = 0.0
                    t.transform.rotation.z = 0.0
                    t.transform.rotation.w = 1.0
                    self.br.sendTransform(t)


##################### FUNCTION DEFINITION #######################

def main():
    '''
    Description:    Main function initializing node and spinning for perception task.
    '''
    rclpy.init(args=sys.argv)
    node = ore_tf()
    node.get_logger().info('Task 2A Perception Node started: Publishing ore transforms')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
