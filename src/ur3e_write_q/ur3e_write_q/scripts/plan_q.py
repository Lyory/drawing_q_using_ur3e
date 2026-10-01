#!/usr/bin/env python3
"""Return the complete Q motion as RViz's MoveGroup plan; never execute here."""
import copy
import threading
import time

import rclpy
from rclpy.action import ActionClient, ActionServer, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetCartesianPath, GetPositionFK
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import ListControllers
from rclpy.clock import Clock, ClockType
from std_msgs.msg import Empty

from draw_q import DrawQ


def end_state(start, trajectory):
    state = copy.deepcopy(start)
    values = dict(zip(state.joint_state.name, state.joint_state.position))
    values.update(zip(trajectory.joint_trajectory.joint_names,
                      trajectory.joint_trajectory.points[-1].positions))
    state.joint_state.name = list(values)
    state.joint_state.position = list(values.values())
    state.joint_state.velocity = []
    state.joint_state.effort = []
    state.joint_state.header.stamp.sec = 0
    state.joint_state.header.stamp.nanosec = 0
    state.is_diff = False
    return state


def append_trajectory(combined, segment):
    target = combined.joint_trajectory
    source = segment.joint_trajectory
    if not target.points or not source.points:
        raise RuntimeError("Empty trajectory segment")
    if set(target.joint_names) != set(source.joint_names):
        raise RuntimeError("Trajectory joint sets differ")
    order = [source.joint_names.index(name) for name in target.joint_names]
    first = [source.points[0].positions[i] for i in order]
    if any(abs(a - b) > 0.01 for a, b in zip(target.points[-1].positions, first)):
        raise RuntimeError("Discontinuous trajectory segments")
    last = target.points[-1].time_from_start
    offset = last.sec * 1_000_000_000 + last.nanosec
    for original in source.points[1:]:
        point = copy.deepcopy(original)
        for field in ('positions', 'velocities', 'accelerations', 'effort'):
            values = getattr(point, field)
            if values:
                setattr(point, field, [values[i] for i in order])
        stamp = point.time_from_start
        total = offset + stamp.sec * 1_000_000_000 + stamp.nanosec
        previous = target.points[-1].time_from_start
        if total <= previous.sec * 1_000_000_000 + previous.nanosec:
            raise RuntimeError("Non-increasing trajectory timestamps")
        stamp.sec, stamp.nanosec = divmod(total, 1_000_000_000)
        target.points.append(point)


class PlanQ(DrawQ):
    def __init__(self):
        super().__init__()
        self.move.destroy()
        self.move = ActionClient(self, MoveGroup, '/draw_q/backend_move_action')
        self.fk = self.create_client(GetPositionFK, '/compute_fk')
        self.lock = threading.Lock()
        self.have_joints = False
        self.last_joint_wall = 0.0
        self.last_joint_stamp = 0
        self.controllers_ready = False
        self.controller_check = None
        self.controller_checked_at = 0.0
        self.controllers = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self.requested = False
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.trigger = self.create_publisher(Empty, '/rviz/moveit/plan', 10)
        self.server = ActionServer(
            self, MoveGroup, '/move_action', self.plan,
            goal_callback=self.accept,
            callback_group=ReentrantCallbackGroup())
        self.timer = self.create_timer(
            1.0, self.auto_plan, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_joints(self, message):
        self.have_joints = (set(self.SAFE_JOINTS).issubset(message.name) and
                            len(message.position) == len(message.name))
        if self.have_joints:
            self.last_joint_wall = time.monotonic()
            self.last_joint_stamp = (message.header.stamp.sec * 1_000_000_000 +
                                     message.header.stamp.nanosec)

    def ready(self):
        age = self.get_clock().now().nanoseconds - self.last_joint_stamp
        return (self.have_joints and self.last_joint_stamp > 0 and
                0 <= age < 1_000_000_000 and
                time.monotonic() - self.last_joint_wall < 2.0 and
                self.controllers_ready and
                time.monotonic() - self.controller_checked_at < 3.0)

    def check_controllers(self):
        if self.controller_check is not None:
            if not self.controller_check.done():
                return
            try:
                active = {c.name for c in self.controller_check.result().controller
                          if c.state == 'active'}
                self.controllers_ready = {'joint_state_broadcaster',
                                          'joint_trajectory_controller'}.issubset(active)
                self.controller_checked_at = time.monotonic()
            except Exception:
                self.controllers_ready = False
            self.controller_check = None
        if self.controllers.service_is_ready():
            self.controller_check = self.controllers.call_async(ListControllers.Request())
        else:
            self.controllers_ready = False

    def auto_plan(self):
        self.check_controllers()
        if self.requested:
            return
        if (self.ready() and self.move.server_is_ready() and
                self.cartesian.service_is_ready() and self.fk.service_is_ready() and
                self.trigger.get_subscription_count()):
            self.trigger.publish(Empty())
            self.get_logger().info('Requesting full Q plan in RViz; robot remains stationary.')

    def accept(self, request):
        # RViz Execute uses the real /execute_trajectory action, independently.
        if not request.planning_options.plan_only or request.request.group_name != self.GROUP:
            self.get_logger().error('Use Plan then Execute for the Q workflow.')
            return GoalResponse.REJECT
        if not self.ready():
            self.get_logger().error(
                'Cannot plan: waiting for active Gazebo controllers and fresh joint states.')
            return GoalResponse.REJECT
        if not self.lock.acquire(blocking=False):
            return GoalResponse.REJECT
        self.requested = True
        return GoalResponse.ACCEPT

    async def cartesian_segment(self, state, poses):
        req = GetCartesianPath.Request()
        req.header.frame_id = self.FRAME
        req.start_state = state
        req.group_name, req.link_name = self.GROUP, self.EEF
        req.waypoints = poses
        req.max_step = self.STEP
        req.jump_threshold = 5.0
        req.revolute_jump_threshold = 0.3
        req.avoid_collisions = True
        if hasattr(req, 'max_velocity_scaling_factor'):
            req.max_velocity_scaling_factor = self.SPEED
            req.max_acceleration_scaling_factor = self.SPEED
        result = await self.cartesian.call_async(req)
        if result.error_code.val != MoveItErrorCodes.SUCCESS or result.fraction < 0.999:
            raise RuntimeError(f'Cartesian Q incomplete: {result.fraction:.1%}')
        return result.solution

    async def plan(self, handle):
        result = MoveGroup.Result()
        try:
            goal = MoveGroup.Goal()
            goal.planning_options.plan_only = True
            goal.request.group_name = self.GROUP
            goal.request.start_state.is_diff = True
            goal.request.num_planning_attempts = 10
            goal.request.allowed_planning_time = 10.0
            goal.request.max_velocity_scaling_factor = 0.1
            goal.request.max_acceleration_scaling_factor = 0.1
            constraint = Constraints()
            constraint.joint_constraints = [
                JointConstraint(joint_name=name, position=value,
                                tolerance_above=0.01, tolerance_below=0.01, weight=1.0)
                for name, value in self.SAFE_JOINTS.items()]
            goal.request.goal_constraints = [constraint]
            safe_handle = await self.move.send_goal_async(goal)
            if not safe_handle.accepted:
                raise RuntimeError('Safe-start plan rejected')
            safe = (await safe_handle.get_result_async()).result
            if safe.error_code.val != MoveItErrorCodes.SUCCESS:
                raise RuntimeError('Cannot plan safe start')
            trajectory = copy.deepcopy(safe.planned_trajectory)
            if not trajectory.joint_trajectory.points:
                raise RuntimeError('Empty safe-start plan')
            state = end_state(safe.trajectory_start, trajectory)
            req = GetPositionFK.Request()
            req.header.frame_id = self.FRAME
            req.fk_link_names = [self.EEF]
            req.robot_state = state
            fk = await self.fk.call_async(req)
            if fk.error_code.val != MoveItErrorCodes.SUCCESS or not fk.pose_stamped:
                raise RuntimeError('Cannot compute planned start pose')
            raised = copy.deepcopy(fk.pose_stamped[0])
            raised.pose.position.z += self.SAFE_Z_LIFT
            lift = await self.cartesian_segment(state, [raised.pose])
            append_trajectory(trajectory, lift)
            state = end_state(state, lift)
            # Use FK of the actual planned endpoint, not live TF of the unmoved robot.
            req.robot_state = state
            fk = await self.fk.call_async(req)
            if fk.error_code.val != MoveItErrorCodes.SUCCESS or not fk.pose_stamped:
                raise RuntimeError('Cannot compute lifted pose')
            path = self.make_q(fk.pose_stamped[0])
            drawing = await self.cartesian_segment(state, [p.pose for p in path.poses])
            append_trajectory(trajectory, drawing)
            if not self.ready():
                raise RuntimeError('Gazebo state/controller connection lost while planning')
            result.trajectory_start = safe.trajectory_start
            result.planned_trajectory = trajectory
            result.error_code.val = MoveItErrorCodes.SUCCESS
            handle.succeed()
            self.get_logger().info('Full Q plan ready. Press Execute in RViz to draw.')
        except Exception as exc:
            self.get_logger().error(str(exc))
            result.error_code.val = MoveItErrorCodes.PLANNING_FAILED
            handle.abort()
        finally:
            self.lock.release()
        return result


def main():
    rclpy.init()
    node = PlanQ()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
