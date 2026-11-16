#!/usr/bin/env python3
"""Feedback-checked Gazebo pick-and-place, using the CAD adaptive gripper and idealized grasp retention."""
import json
import time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from gz.transport13 import Node as GzNode
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.stringmsg_pb2 import StringMsg
from gp7_motion.kinematics import ARM_JOINTS, Kinematics
from gp7_motion.reach import carry_waypoints


class PickPlace(Node):
    def __init__(self):
        super().__init__('gp7_pick_place')
        self.declare_parameter('pickup_x', .5)
        self.declare_parameter('pickup_y', .5)
        self.declare_parameter('result_file', '')
        self.declare_parameter('place_x', .5)
        self.declare_parameter('place_y', -.5)
        self.destination = np.array([self.get_parameter('place_x').value,
                                     self.get_parameter('place_y').value, .225])
        self.pickup = np.array([self.get_parameter('pickup_x').value,
                                self.get_parameter('pickup_y').value, .025])
        source = Path(get_package_share_directory('gp7_description'), 'urdf/gp7.urdf').read_text()
        self.kin = Kinematics(source)
        self.q, self.q_time, self.box, self.box_time = None, 0., None, 0.
        self.grasp = None
        self.finger_angle = None
        self.gripper_pub = self.create_publisher(Float64MultiArray, '/gripper_controller/commands', 10)
        self.carrying = False
        self.max_orientation_error = 0.
        self.initial_rotation = None
        self.active_goal = None
        self.stop_requested = False
        self.create_subscription(JointState, '/joint_states', self.joints, 10)
        from std_msgs.msg import Empty as RosEmpty
        self.create_subscription(RosEmpty, '/gp7/stop', self.stop, 10)
        self.client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gz = GzNode()
        self.attach = self.gz.advertise('/gp7/grasp/attach', Empty)
        self.detach = self.gz.advertise('/gp7/grasp/detach', Empty)
        self.gz.subscribe(StringMsg, '/gp7/grasp/state', self.grasp_state)
        self.gz.subscribe(Pose_V, '/model/box_b/pose', self.box_pose)

    def stop(self, _msg):
        self.stop_requested = True
        if self.active_goal is not None:
            self.active_goal.cancel_goal_async()

    def joints(self, msg):
        values = dict(zip(msg.name, msg.position))
        if 'gripper_driver' in values:
            self.finger_angle = values['gripper_driver']
        if all(name in values for name in ARM_JOINTS):
            self.q = np.array([values[name] for name in ARM_JOINTS])
            self.q_time = time.monotonic()

    def grasp_state(self, msg):
        self.grasp = msg.data

    def box_pose(self, msg):
        for pose in msg.pose:
            if pose.name == 'box_b':
                p, q = pose.position, pose.orientation
                self.box = (np.array([p.x, p.y, p.z]), np.array([q.x, q.y, q.z, q.w]))
                self.box_time = time.monotonic()
                if self.carrying and self.initial_rotation is not None:
                    angle = (self.initial_rotation.inv() * Rotation.from_quat(self.box[1])).magnitude()
                    self.max_orientation_error = max(self.max_orientation_error, float(angle))

    def wait(self, predicate, timeout, label):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if self.stop_requested:
                raise RuntimeError('Stop requested; active motion cancelled, grasp retained')
            if predicate():
                return
            rclpy.spin_once(self, timeout_sec=.02)
        raise TimeoutError(label)

    def settle(self, seconds):
        start = self.get_clock().now().nanoseconds
        self.wait(lambda: (self.get_clock().now().nanoseconds-start)*1e-9 >= seconds,
                  max(15., seconds*10), 'Simulation clock stalled')

    def fingers(self, target):
        self.wait(lambda: self.finger_angle is not None and
                  self.gripper_pub.get_subscription_count() > 0, 10., 'No gripper feedback/controller')
        start = self.finger_angle
        duration = max(1., abs(target-start)/.25)
        stamp = self.get_clock().now().nanoseconds
        while True:
            u = min(1., (self.get_clock().now().nanoseconds-stamp)*1e-9/duration)
            blend = 10*u**3-15*u**4+6*u**5
            self.gripper_pub.publish(Float64MultiArray(data=[float(start+blend*(target-start))]))
            if u >= 1.:
                break
            self.settle(.02)
        self.settle(.4)
        if abs(self.finger_angle-target) > .02:
            raise RuntimeError('Gripper did not reach the commanded angle')

    def switch_grasp(self, attached):
        publisher = self.attach if attached else self.detach
        desired = 'attached' if attached else 'detached'
        deadline = time.monotonic() + 8.
        while self.grasp != desired and time.monotonic() < deadline:
            publisher.publish(Empty())
            self.settle(.1)
        if self.grasp != desired:
            raise RuntimeError(f'No confirmation that grasp is {desired}')

    def move(self, trajectory, label):
        t, qs, vs = trajectory
        self.get_logger().info(f'{label} ({t[-1]:.1f} s)')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS
        for stamp, q, v in zip(t, qs, vs):
            ns = round(float(stamp)*1e9)
            goal.trajectory.points.append(JointTrajectoryPoint(
                positions=q.tolist(), velocities=v.tolist(),
                time_from_start=Duration(sec=ns//10**9, nanosec=ns%10**9)))
        future = self.client.send_goal_async(goal)
        self.wait(future.done, 10., 'Action goal response timed out')
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError(f'Trajectory rejected: {label}')
        self.active_goal = handle
        result = handle.get_result_async()
        try:
            self.wait(result.done, float(t[-1])*5+20, f'Trajectory timed out: {label}')
        except Exception:
            handle.cancel_goal_async()
            raise
        self.active_goal = None
        response = result.result()
        if response.status != GoalStatus.STATUS_SUCCEEDED or response.result.error_code != 0:
            raise RuntimeError(f'{label}: {response.result.error_string}')
        self.settle(.2)
        if time.monotonic()-self.q_time > 1. or np.max(np.abs(self.q-qs[-1])) > .015:
            raise RuntimeError(f'Joint feedback does not confirm arrival: {label}')
        if self.carrying:
            if time.monotonic()-self.box_time > 1. or self.grasp != 'attached':
                raise RuntimeError('Grasp or box feedback lost')
            if self.max_orientation_error > np.deg2rad(1.):
                raise RuntimeError('Box rotated by more than 1 degree while carried')

    def run(self):
        self.wait(lambda: self.q is not None and self.box is not None and
                  self.get_clock().now().nanoseconds > 0, 60., 'No joint/box/clock feedback')
        if not self.client.wait_for_server(timeout_sec=30.):
            raise RuntimeError('Arm action server unavailable')
        # Gazebo Harmonic's built-in plugin starts attached. Detach at rest before any motion.
        self.switch_grasp(False)
        self.settle(.5)
        if np.linalg.norm(self.box[0]-self.pickup) > .01:
            raise RuntimeError('Box is not at the configured pickup position; restart the scene')
        self.initial_rotation = Rotation.from_quat(self.box[1])
        if self.initial_rotation.magnitude() > .01:
            raise RuntimeError('Box is not upright before pickup')
        pickup_tcp = self.box[0].copy()
        above, waypoints = carry_waypoints(pickup_tcp, self.destination)
        q_above = self.kin.ik(above, self.q, restart=True)
        # Preflight the complete Cartesian route before moving the robot.
        paths = []
        seed = q_above
        for target, duration, label in waypoints:
            path = self.kin.segment(seed, target, duration)
            paths.append((path, label))
            seed = path[1][-1]
        self.fingers(.15)
        self.move(self.kin.joint_segment(self.q, q_above), 'Approach pickup')
        self.move(*paths[0])
        tcp = self.kin.fk(self.q)[:3, 3]
        if np.linalg.norm(tcp-(self.box[0])) > .004:
            raise RuntimeError('Grasp frame is not close enough to the box; refusing grasp')
        self.fingers(-.162)
        self.switch_grasp(True)
        self.carrying = True
        for path in paths[1:-1]:
            self.move(*path)
        if np.linalg.norm(self.box[0]-self.destination) > .008:
            raise RuntimeError('Box feedback does not confirm placement; retaining grasp')
        self.switch_grasp(False)
        self.carrying = False
        self.fingers(.15)
        self.move(*paths[-1])
        self.settle(2.)
        final, quaternion = self.box
        error = float(np.linalg.norm(final-self.destination))
        orientation = float((self.initial_rotation.inv()*Rotation.from_quat(quaternion)).magnitude())
        if time.monotonic()-self.box_time > 1. or error > .008 or orientation > np.deg2rad(1.):
            raise RuntimeError(f'Placement verification failed: position error {error}, angle {orientation}')
        report = {'success': True, 'pickup_center_m': self.pickup.tolist(),
                  'pickup_radius_m': float(np.linalg.norm(self.pickup[:2])), 'final_box_center_m': final.tolist(),
                  'position_error_m': error, 'final_orientation_error_rad': orientation,
                  'max_carried_orientation_error_rad': self.max_orientation_error,
                  'grasp_model': 'Adaptive fingers with Gazebo fixed-joint grasp retention'}
        filename = self.get_parameter('result_file').value
        if filename:
            Path(filename).write_text(json.dumps(report, indent=2)+'\n')
        self.get_logger().info('PICK AND PLACE VERIFIED: '+json.dumps(report))


def main(args=None):
    rclpy.init(args=args)
    node = PickPlace()
    try:
        node.run()
    except (Exception, KeyboardInterrupt) as error:
        node.get_logger().error(f'Pick-and-place stopped: {error}')
        if node.active_goal is not None:
            future = node.active_goal.cancel_goal_async()
            rclpy.spin_until_future_complete(node, future, timeout_sec=2.)
        raise SystemExit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
