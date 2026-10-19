#!/usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		        StrataCobot (SC) Theme (eYRC 2026-27)
*        		===============================================
*
*  This script implements Task 2A (Manipulation) of StrataCobot (SC) Theme (eYRC 2026-27).
*  Controls the UR7e robotic arm to pick 4 detected ores off the conveyor with the magnet tool
*  and drop each ore into the designated container on the cargo rover according to its kind:
*    - Azurite: Farthest container
*    - Malachite: Middle container
*    - Vanadinite: Nearest container
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
# Filename:         task2A_manipulation.py
# Functions:        quat_to_rot_matrix, main
# Nodes:            task2a_manipulation_node
#                   Publishing Topics  - [ /delta_twist_cmds ]
#                   Subscribing Topics - [ /tcp_pose_raw, /joint_states,
#                                          /arm_status, /ur7e/end_effector/force_magnitude ]


################### IMPORT MODULES #######################

import sys
import rclpy
import tf2_ros
import numpy as np
from rclpy.node import Node
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Int32
from std_srvs.srv import SetBool, Trigger


##################### TASK CONSTANTS #######################

servo_ns = '/ur_arm_controller'
twist_controller = 'delta_twist_controller'
joint_controller = 'delta_joint_controller'
base_frame = 'base_link'

# Velocity limits (below the safety caps: 0.15 m/s linear, 0.35 rad/s angular)
MAX_LINEAR_VEL = 0.10       # m/s
MAX_DESCENT_VEL = 0.035     # m/s for gentle pick approach
MAX_ANGULAR_VEL = 0.20      # rad/s

# Cargo rover container drop positions in base_link (metres)
# Gazebo view: Azurite farthest (most negative X), Malachite middle, Vanadinite nearest
CONTAINER_TARGETS = {
    'azurite_ore':   np.array([-0.7430, 0.5280, 0.2300]),
    'malachite_ore': np.array([-0.4097, 0.5280, 0.2300]),
    'vanadinite_ore': np.array([-0.0763, 0.5280, 0.2300]),
}

# Safe intermediate transit waypoint avoiding obstacles and singularities
TRANSIT_WAYPOINT = np.array([-0.8000, 0.0000, 0.4000])

ORE_KINDS = ['azurite_ore', 'malachite_ore', 'vanadinite_ore']


##################### HELPER FUNCTIONS #######################

def quat_to_rot_matrix(qx, qy, qz, qw):
    '''
    Description:    Converts unit quaternion to 3x3 rotation matrix.
    '''
    R = np.zeros((3, 3), dtype=np.float64)
    R[0, 0] = 1.0 - 2.0 * (qy * qy + qz * qz)
    R[0, 1] = 2.0 * (qx * qy - qz * qw)
    R[0, 2] = 2.0 * (qx * qz + qy * qw)

    R[1, 0] = 2.0 * (qx * qy + qz * qw)
    R[1, 1] = 1.0 - 2.0 * (qx * qx + qz * qz)
    R[1, 2] = 2.0 * (qy * qz - qx * qw)

    R[2, 0] = 2.0 * (qx * qz - qy * qw)
    R[2, 1] = 2.0 * (qy * qz + qx * qw)
    R[2, 2] = 1.0 - 2.0 * (qx * qx + qy * qy)
    return R


##################### CLASS DEFINITION #######################

class arm_manipulation(Node):
    '''
    ___CLASS___

    Description:    State-machine controller node that coordinates picking ores from
                    the conveyor and placing them into their matching cargo rover containers.
    '''

    def __init__(self):
        '''
        Description:    Initialization of class arm_manipulation
        '''
        super().__init__(
            'arm_manipulation_node',
            parameter_overrides=[rclpy.parameter.Parameter(
                'use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        # Publishers
        self.twist_pub = self.create_publisher(TwistStamped, '/delta_twist_cmds', 10)

        # Subscriptions
        self.tcp_sub = self.create_subscription(PoseStamped, '/tcp_pose_raw', self.tcpposecb, 20)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.jointstatecb, 50)
        self.status_sub = self.create_subscription(Int32, '/arm_status', self.armstatuscb, 10)
        self.force_sub = self.create_subscription(Float32, '/ur7e/end_effector/force_magnitude', self.forcecb, 20)

        # Service Clients
        self.switch_cli = self.create_client(SwitchController, f'{servo_ns}/switch_controller')
        self.magnet_cli = self.create_client(SetBool, '/magnet')
        self.unlock_cli = self.create_client(Trigger, f'{servo_ns}/unlock_protective_stop')

        # TF Buffer
        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # State Variables
        self.tcp_pose = None
        self.arm_status = 0
        self.force_magnitude = 0.0
        self.controller_active = False

        # State Machine States:
        # INIT_WAIT -> FIND_NEXT_ORE -> MOVE_HOVER_ORE -> DESCEND_PICK -> WAIT_MAGNET ->
        # LIFT_ORE -> TRANSIT_CONTAINER -> MOVE_HOVER_CONTAINER -> DESCEND_CONTAINER ->
        # RELEASE_ORE -> ASCEND_CONTAINER -> TRANSIT_CONVEYOR -> (Repeat / ALL_DONE)
        self.state = 'INIT_WAIT'
        self.state_start_time = None
        self.init_start_time = None

        self.current_ore_frame = None
        self.current_ore_kind = None
        self.current_ore_pick_pos = None
        self.current_container_pos = None

        self.completed_ores = set()
        self.magnet_state = False

        # 20 Hz control loop
        self.control_rate = 0.05
        self.timer = self.create_timer(self.control_rate, self.control_cycle)

    def tcpposecb(self, data):
        self.tcp_pose = data

    def jointstatecb(self, data):
        pass

    def forcecb(self, data):
        self.force_magnitude = float(data.data)

    def armstatuscb(self, data):
        self.arm_status = int(data.data)
        if self.arm_status in (20, 21, 22, 23):
            self.get_logger().warn(
                f"Protective stop active (code {self.arm_status}). Requesting unlock...")
            if self.unlock_cli.service_is_ready():
                req = Trigger.Request()
                self.unlock_cli.call_async(req)

    def set_magnet(self, enable: bool):
        '''
        Description:    Calls /magnet service to engage or release the magnetic tool.
        '''
        self.magnet_state = enable
        if self.magnet_cli.service_is_ready():
            req = SetBool.Request()
            req.data = enable
            self.magnet_cli.call_async(req)
            self.get_logger().info(f"Magnet service called: {enable}")

    def activate_twist_controller(self):
        '''
        Description:    Switches UR arm active controller to delta_twist_controller.
        '''
        if not self.switch_cli.service_is_ready():
            return
        req = SwitchController.Request()
        req.activate_controllers = [twist_controller]
        req.deactivate_controllers = [joint_controller]
        req.strictness = SwitchController.Request.STRICT
        self.switch_cli.call_async(req)
        self.controller_active = True
        self.get_logger().info("Requested switch to delta_twist_controller")

    def compute_twist(self, target_pos, max_linear_vel=MAX_LINEAR_VEL):
        '''
        Description:    Calculates linear and angular velocities to smoothly drive the tool
                        towards target_pos while keeping the tool pointed vertically downward.

        Args:
            target_pos      (np.ndarray): Target 3D position in base_link (x, y, z)
            max_linear_vel  (float):      Maximum linear velocity cap

        Returns:
            twist           (TwistStamped): Velocity command in base_link
            dist            (float):        Distance to target position
        '''
        twist = TwistStamped()
        twist.header.frame_id = base_frame
        twist.header.stamp = self.get_clock().now().to_msg()

        curr_pos = np.array([
            self.tcp_pose.pose.position.x,
            self.tcp_pose.pose.position.y,
            self.tcp_pose.pose.position.z
        ], dtype=np.float64)

        pos_err = np.array(target_pos, dtype=np.float64) - curr_pos
        dist = float(np.linalg.norm(pos_err))

        # Linear velocity (proportional)
        if dist > 0.002:
            kp_lin = 1.0
            v = kp_lin * pos_err
            speed = float(np.linalg.norm(v))
            if speed > max_linear_vel:
                v = v * (max_linear_vel / speed)
            twist.twist.linear.x = float(v[0])
            twist.twist.linear.y = float(v[1])
            twist.twist.linear.z = float(v[2])

        # Angular orientation control: tool pointing vertically down (b = [0, 0, -1])
        qx = self.tcp_pose.pose.orientation.x
        qy = self.tcp_pose.pose.orientation.y
        qz = self.tcp_pose.pose.orientation.z
        qw = self.tcp_pose.pose.orientation.w

        R = quat_to_rot_matrix(qx, qy, qz, qw)
        a = R[:, 2]  # TCP Z-axis (tool pointing vector)
        b = np.array([0.0, 0.0, -1.0], dtype=np.float64)

        c = np.cross(a, b)
        c_norm = float(np.linalg.norm(c))
        if c_norm > 1e-4:
            dot_val = float(np.clip(np.dot(a, b), -1.0, 1.0))
            angle = float(np.arctan2(c_norm, dot_val))
            axis = c / c_norm
            kp_rot = 1.5
            omega = kp_rot * angle * axis
            w_norm = float(np.linalg.norm(omega))
            if w_norm > MAX_ANGULAR_VEL:
                omega = omega * (MAX_ANGULAR_VEL / w_norm)
            twist.twist.angular.x = float(omega[0])
            twist.twist.angular.y = float(omega[1])
            twist.twist.angular.z = float(omega[2])

        return twist, dist

    def control_cycle(self):
        '''
        Description:    20 Hz timer execution loop driving the arm state machine.
        '''
        now = self.get_clock().now()

        # Guard: wait until tool pose is received
        if self.tcp_pose is None:
            return

        # Ensure active velocity controller
        if not self.controller_active:
            self.activate_twist_controller()
            return

        # STATE 0: Initial wait to allow perception node to detect and publish transforms
        if self.state == 'INIT_WAIT':
            if self.init_start_time is None:
                self.init_start_time = now
                self.set_magnet(False)
            elapsed = (now - self.init_start_time).nanoseconds * 1e-9
            # Publish stationary twist continuously during initialization
            zero_twist = TwistStamped()
            zero_twist.header.frame_id = base_frame
            zero_twist.header.stamp = now.to_msg()
            self.twist_pub.publish(zero_twist)

            if elapsed > 3.0:
                self.get_logger().info("Initial perception wait complete. Proceeding to find ores.")
                self.state = 'FIND_NEXT_ORE'
            return

        # STATE 1: Scan TF tree for remaining detected ores
        if self.state == 'FIND_NEXT_ORE':
            candidates = []
            for kind in ORE_KINDS:
                for idx in (1, 2):
                    frame_name = f"{kind}_{idx}"
                    if frame_name in self.completed_ores:
                        continue
                    try:
                        tf = self.tf_buffer.lookup_transform(
                            base_frame, frame_name, rclpy.time.Time())
                        pos = np.array([
                            tf.transform.translation.x,
                            tf.transform.translation.y,
                            tf.transform.translation.z
                        ], dtype=np.float64)
                        candidates.append((frame_name, kind, pos))
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                            tf2_ros.ExtrapolationException):
                        pass

            if not candidates:
                if len(self.completed_ores) >= 4:
                    self.get_logger().info("All 4 ores successfully sorted!")
                    self.state = 'ALL_DONE'
                else:
                    # Still waiting for perception or arm settling
                    zero_twist = TwistStamped()
                    zero_twist.header.frame_id = base_frame
                    zero_twist.header.stamp = now.to_msg()
                    self.twist_pub.publish(zero_twist)
                return

            # Sort candidate ores by X position (accessible order on conveyor)
            candidates.sort(key=lambda item: item[2][0])
            chosen_frame, chosen_kind, chosen_pos = candidates[0]

            self.current_ore_frame = chosen_frame
            self.current_ore_kind = chosen_kind
            self.current_ore_pick_pos = chosen_pos
            self.current_container_pos = CONTAINER_TARGETS[chosen_kind]

            self.get_logger().info(
                f"Selected ore: {chosen_frame} at pos {np.round(chosen_pos, 3)}, "
                f"target container at {np.round(self.current_container_pos, 3)}")
            self.state = 'MOVE_HOVER_ORE'

        # STATE 2: Move hover above selected ore on conveyor
        elif self.state == 'MOVE_HOVER_ORE':
            hover_pos = np.array([
                self.current_ore_pick_pos[0],
                self.current_ore_pick_pos[1],
                0.35
            ], dtype=np.float64)
            cmd, dist = self.compute_twist(hover_pos, max_linear_vel=MAX_LINEAR_VEL)
            self.twist_pub.publish(cmd)
            if dist < 0.015:
                self.get_logger().info(f"Hover reached above {self.current_ore_frame}. Descending to pick.")
                self.state = 'DESCEND_PICK'

        # STATE 3: Descend slowly to make contact with ore top face
        elif self.state == 'DESCEND_PICK':
            # Target pick height: top face z
            pick_pos = np.array([
                self.current_ore_pick_pos[0],
                self.current_ore_pick_pos[1],
                self.current_ore_pick_pos[2] + 0.003
            ], dtype=np.float64)
            cmd, dist = self.compute_twist(pick_pos, max_linear_vel=MAX_DESCENT_VEL)
            self.twist_pub.publish(cmd)

            curr_z = self.tcp_pose.pose.position.z
            # Contact detection: force threshold or reached proximity
            if (self.force_magnitude > 1.0 or
                    dist < 0.008 or
                    curr_z <= (self.current_ore_pick_pos[2] + 0.006)):
                self.get_logger().info(
                    f"Contact detected (force: {self.force_magnitude:.2f}N). Engaging magnet.")
                self.set_magnet(True)
                self.state_start_time = now
                self.state = 'WAIT_MAGNET'

        # STATE 4: Wait briefly for magnetic field to capture ore
        elif self.state == 'WAIT_MAGNET':
            zero_twist = TwistStamped()
            zero_twist.header.frame_id = base_frame
            zero_twist.header.stamp = now.to_msg()
            self.twist_pub.publish(zero_twist)

            elapsed = (now - self.state_start_time).nanoseconds * 1e-9
            if elapsed > 0.5:
                self.get_logger().info("Magnet engaged. Lifting ore.")
                self.state = 'LIFT_ORE'

        # STATE 5: Lift ore vertically off conveyor
        elif self.state == 'LIFT_ORE':
            lift_pos = np.array([
                self.current_ore_pick_pos[0],
                self.current_ore_pick_pos[1],
                0.38
            ], dtype=np.float64)
            cmd, dist = self.compute_twist(lift_pos, max_linear_vel=0.08)
            self.twist_pub.publish(cmd)
            if dist < 0.015:
                self.get_logger().info("Ore lifted off conveyor. Moving to transit waypoint.")
                self.state = 'TRANSIT_CONTAINER'

        # STATE 6: Transit waypoint between conveyor and cargo rover
        elif self.state == 'TRANSIT_CONTAINER':
            cmd, dist = self.compute_twist(TRANSIT_WAYPOINT, max_linear_vel=MAX_LINEAR_VEL)
            self.twist_pub.publish(cmd)
            if dist < 0.020:
                self.get_logger().info(f"Transit reached. Moving over {self.current_ore_kind} container.")
                self.state = 'MOVE_HOVER_CONTAINER'

        # STATE 7: Hover above target container
        elif self.state == 'MOVE_HOVER_CONTAINER':
            hover_pos = np.array([
                self.current_container_pos[0],
                self.current_container_pos[1],
                0.35
            ], dtype=np.float64)
            cmd, dist = self.compute_twist(hover_pos, max_linear_vel=MAX_LINEAR_VEL)
            self.twist_pub.publish(cmd)
            if dist < 0.015:
                self.get_logger().info("Hover reached above container. Descending to release height.")
                self.state = 'DESCEND_CONTAINER'

        # STATE 8: Lower into container opening
        elif self.state == 'DESCEND_CONTAINER':
            drop_pos = np.array([
                self.current_container_pos[0],
                self.current_container_pos[1],
                0.23
            ], dtype=np.float64)
            cmd, dist = self.compute_twist(drop_pos, max_linear_vel=0.06)
            self.twist_pub.publish(cmd)
            if dist < 0.015:
                self.get_logger().info("At release height. Releasing magnet.")
                self.set_magnet(False)
                self.state_start_time = now
                self.state = 'RELEASE_ORE'

        # STATE 9: Wait for ore to settle inside container
        elif self.state == 'RELEASE_ORE':
            zero_twist = TwistStamped()
            zero_twist.header.frame_id = base_frame
            zero_twist.header.stamp = now.to_msg()
            self.twist_pub.publish(zero_twist)

            elapsed = (now - self.state_start_time).nanoseconds * 1e-9
            if elapsed > 0.8:
                self.completed_ores.add(self.current_ore_frame)
                self.get_logger().info(
                    f"Successfully sorted {self.current_ore_frame} into container "
                    f"({len(self.completed_ores)}/4 complete). Ascending.")
                self.state = 'ASCEND_CONTAINER'

        # STATE 10: Ascend vertically from container
        elif self.state == 'ASCEND_CONTAINER':
            lift_pos = np.array([
                self.current_container_pos[0],
                self.current_container_pos[1],
                0.38
            ], dtype=np.float64)
            cmd, dist = self.compute_twist(lift_pos, max_linear_vel=0.08)
            self.twist_pub.publish(cmd)
            if dist < 0.015:
                self.state = 'TRANSIT_CONVEYOR'

        # STATE 11: Transit back towards conveyor via intermediate waypoint
        elif self.state == 'TRANSIT_CONVEYOR':
            cmd, dist = self.compute_twist(TRANSIT_WAYPOINT, max_linear_vel=MAX_LINEAR_VEL)
            self.twist_pub.publish(cmd)
            if dist < 0.020:
                self.state = 'FIND_NEXT_ORE'

        # STATE 12: All ores completed, park at transit waypoint and hold position
        elif self.state == 'ALL_DONE':
            cmd, dist = self.compute_twist(TRANSIT_WAYPOINT, max_linear_vel=0.05)
            self.twist_pub.publish(cmd)


##################### FUNCTION DEFINITION #######################

def main():
    '''
    Description:    Main entry point initializing node and running arm manipulation.
    '''
    rclpy.init(args=sys.argv)
    node = arm_manipulation()
    node.get_logger().info('Task 2A Manipulation Node started')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
