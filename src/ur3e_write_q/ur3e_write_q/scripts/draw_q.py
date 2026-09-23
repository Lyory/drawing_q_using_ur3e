#!/usr/bin/env python3
import copy
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetCartesianPath
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker


class DrawQ(Node):
    FRAME, EEF, GROUP = "base_link", "tool0", "ur_manipulator"
    STEP, SPEED = 0.004, 0.08
    WIDTH, HEIGHT = 0.06, 0.12
    Z_OFFSET = 0.05  # drawing starts 5 cm above the safe pose
    SAFE_Z_LIFT = 0.02  # raise the safe pose by 2 cm before drawing

    SAFE_JOINTS = {
        "shoulder_pan_joint": -0.8062525553,
        "shoulder_lift_joint": -0.2823116024,
        "elbow_joint": 1.2251390614,
        "wrist_1_joint": 1.2194131064,
        "wrist_2_joint": 1.0917290302,
        "wrist_3_joint": -1.0695566106,
    }

    def __init__(self):
        super().__init__("draw_q")
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.path_pub = self.create_publisher(Path, "/draw_q/path", qos)
        self.plane_pub = self.create_publisher(Marker, "/draw_q/plane", qos)

        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)

        self.cartesian = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.move = ActionClient(self, MoveGroup, "/move_action")
        self.execute_action = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")

    def wait(self, future, timeout=60.0):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            raise RuntimeError("ROS/MoveIt timeout")
        return future.result()

    def tool_pose(self):
        t = self.tf.lookup_transform(self.FRAME, self.EEF, rclpy.time.Time())
        p = PoseStamped()
        p.header.frame_id = self.FRAME
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = t.transform.translation.x
        p.pose.position.y = t.transform.translation.y
        p.pose.position.z = t.transform.translation.z
        p.pose.orientation = t.transform.rotation
        return p

    def move_safe(self):
        if not self.move.wait_for_server(timeout_sec=30.0):
            raise RuntimeError("/move_action unavailable")

        c = Constraints()
        c.joint_constraints = [
            JointConstraint(
                joint_name=name, position=value,
                tolerance_above=0.01, tolerance_below=0.01, weight=1.0
            )
            for name, value in self.SAFE_JOINTS.items()
        ]

        goal = MoveGroup.Goal()
        goal.request.group_name = self.GROUP
        goal.request.pipeline_id = "move_group"
        goal.request.start_state.is_diff = True
        goal.request.goal_constraints = [c]
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 10.0
        goal.request.max_velocity_scaling_factor = 0.10
        goal.request.max_acceleration_scaling_factor = 0.10
        goal.planning_options.plan_only = False

        handle = self.wait(self.move.send_goal_async(goal), 30.0)
        if not handle.accepted:
            raise RuntimeError("Start goal rejected")
        result = self.wait(handle.get_result_async(), 90.0)

        if result.result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError("Cannot reach safe start")

        start = self.tool_pose()
        raised = copy.deepcopy(start)
        raised.pose.position.z += self.SAFE_Z_LIFT
        path = Path()
        path.header = copy.deepcopy(start.header)
        path.poses = [start, raised]
        self.get_logger().info("Raising safe start by 2 cm...")
        self.execute(self.compute_q(path, label="Safe lift"))

    def make_q(self, start):
        x = start.pose.position.x
        y0 = start.pose.position.y
        z0 = start.pose.position.z + self.Z_OFFSET
        ry, rz = self.WIDTH / 2.0, self.HEIGHT / 2.0
        cy, cz = y0, z0 + rz

        sweep, n = 2.0 * math.pi + math.pi / 4.0, 56
        xyz = [(x, y0, z0)] + [
            (x,
             cy + ry * math.sin(sweep * i / n),
             cz - rz * math.cos(sweep * i / n))
            for i in range(1, n + 1)
        ]

        y, z = xyz[-1][1], xyz[-1][2]
        xyz += [(x, y + 0.018*i/6, z - 0.022*i/6) for i in range(1, 7)]

        path = Path()
        path.header = copy.deepcopy(start.header)
        for x, y, z in xyz:
            p = PoseStamped()
            p.header = copy.deepcopy(path.header)
            p.pose.position.x, p.pose.position.y, p.pose.position.z = x, y, z
            p.pose.orientation = copy.deepcopy(start.pose.orientation)
            path.poses.append(p)

        self.path_pub.publish(path)
        self.publish_plane(start, cz)
        return path

    def publish_plane(self, start, center_z):
        m = Marker()
        m.header = copy.deepcopy(start.header)
        m.ns, m.id = "writing_plane", 0
        m.type, m.action = Marker.CUBE, Marker.ADD
        m.pose.position.x = start.pose.position.x - 0.003
        m.pose.position.y = start.pose.position.y
        m.pose.position.z = center_z
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = 0.003, 0.16, 0.20
        m.color.r = m.color.g = m.color.b = 0.55
        m.color.a = 0.20
        self.plane_pub.publish(m)

    def compute_q(self, path, label="Q"):
        if not self.cartesian.wait_for_service(timeout_sec=30.0):
            raise RuntimeError("/compute_cartesian_path unavailable")

        req = GetCartesianPath.Request()
        req.header = path.header
        req.start_state.is_diff = True
        req.group_name, req.link_name = self.GROUP, self.EEF
        # Include the first point, which may be above the current TCP.
        req.waypoints = [p.pose for p in path.poses]
        req.max_step = self.STEP
        req.jump_threshold = 5.0
        req.revolute_jump_threshold = 0.3
        req.avoid_collisions = True

        if hasattr(req, "max_velocity_scaling_factor"):
            req.max_velocity_scaling_factor = self.SPEED
            req.max_acceleration_scaling_factor = self.SPEED

        res = self.wait(self.cartesian.call_async(req))
        self.get_logger().info(f"Cartesian {label}: {res.fraction:.1%}")

        if res.error_code.val != MoveItErrorCodes.SUCCESS or res.fraction < 0.999:
            raise RuntimeError(f"{label} is not fully reachable/collision-free")
        return res.solution

    def execute(self, trajectory):
        if not self.execute_action.wait_for_server(timeout_sec=30.0):
            raise RuntimeError("/execute_trajectory unavailable")

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        handle = self.wait(self.execute_action.send_goal_async(goal), 30.0)

        if not handle.accepted:
            raise RuntimeError("Trajectory rejected")

        duration = trajectory.joint_trajectory.points[-1].time_from_start
        timeout = max(60.0, duration.sec + duration.nanosec / 1e9 + 20.0)
        result = self.wait(handle.get_result_async(), timeout)

        if result.result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError("Trajectory execution failed")

    def run(self):
        deadline = time.time() + 30.0
        while rclpy.ok():
            try:
                self.tool_pose()
                break
            except TransformException:
                if time.time() > deadline:
                    raise RuntimeError("No base_link -> tool0 TF")
                rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info("Moving to safe start...")
        self.move_safe()

        q = self.make_q(self.tool_pose())
        self.get_logger().info("Fixed plane + Q ready. Starting in 2 s...")
        time.sleep(2.0)

        trajectory = self.compute_q(q)
        self.get_logger().info("Tracing Q...")
        self.execute(trajectory)
        self.get_logger().info("Done: tool0 traced the fixed Q.")


def main():
    rclpy.init()
    node = DrawQ()
    try:
        node.run()
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(str(e))
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
