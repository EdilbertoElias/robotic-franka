import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


DEFAULT_JOINT_NAMES = [f'fr3_joint{i}' for i in range(1, 8)]
DEFAULT_SAFE_ZONE = [0.0, -0.785398, 0.0, -2.356194, 0.0, 1.570796, 0.785398]
DEFAULT_WORK_ZONE = [0.0, -0.5, 0.0, -2.0, 0.0, 1.5, 0.5]


class FrankaComm(Node):

    def __init__(self):
        super().__init__('franka_comm')

        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        self.joint_names = list(self.declare_parameter(
            'joint_names', DEFAULT_JOINT_NAMES).value)

        try:
            safe_zone_val = self.declare_parameter('safe_zone_positions', DEFAULT_SAFE_ZONE).value
        except Exception:
            try:
                safe_zone_val = self.get_parameter_or('safe_zone_positions', rclpy.Parameter('safe_zone_positions', rclpy.Parameter.Type.DOUBLE_ARRAY, DEFAULT_SAFE_ZONE)).value
            except Exception:
                safe_zone_val = DEFAULT_SAFE_ZONE
        if isinstance(safe_zone_val, str):
            try:
                import json
                safe_zone_val = json.loads(safe_zone_val)
            except Exception:
                try:
                    import ast
                    safe_zone_val = ast.literal_eval(safe_zone_val)
                except Exception:
                    safe_zone_val = DEFAULT_SAFE_ZONE
        try:
            self.safe_zone_positions = [float(value) for value in safe_zone_val]
        except Exception:
            self.safe_zone_positions = list(DEFAULT_SAFE_ZONE)

        try:
            work_zone_val = self.declare_parameter('work_zone_positions', DEFAULT_WORK_ZONE).value
        except Exception:
            try:
                work_zone_val = self.get_parameter_or('work_zone_positions', rclpy.Parameter('work_zone_positions', rclpy.Parameter.Type.DOUBLE_ARRAY, DEFAULT_WORK_ZONE)).value
            except Exception:
                work_zone_val = DEFAULT_WORK_ZONE
        if isinstance(work_zone_val, str):
            try:
                import json
                work_zone_val = json.loads(work_zone_val)
            except Exception:
                try:
                    import ast
                    work_zone_val = ast.literal_eval(work_zone_val)
                except Exception:
                    work_zone_val = DEFAULT_WORK_ZONE
        try:
            self.work_zone_positions = [float(value) for value in work_zone_val]
        except Exception:
            self.work_zone_positions = list(DEFAULT_WORK_ZONE)
        self.move_duration = float(self.declare_parameter(
            'move_duration', 3.0).value)
        self.auto_move_to_safe_zone = bool(self.declare_parameter(
            'auto_move_to_safe_zone', True).value)
        self.startup_delay = float(self.declare_parameter(
            'startup_delay', 8.0).value)
        controller_topic = self.declare_parameter(
            'controller_topic', '/fr3_arm_controller/joint_trajectory').value

        if len(self.joint_names) != len(self.safe_zone_positions):
            raise ValueError(
                'joint_names and safe_zone_positions must have the same length')
        if len(self.joint_names) != len(self.work_zone_positions):
            raise ValueError(
                'joint_names and work_zone_positions must have the same length')

        self.current_positions = None
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory, controller_topic, 10)
        self.create_subscription(
            Float64MultiArray, '/franka/joint_targets',
            self.on_joint_targets, 10)
        self.create_subscription(
            JointState, '/joint_states', self.on_joint_state, 10)
        self.create_service(Trigger, '/franka/home', self.on_go_safe_zone)
        self.create_service(Trigger, '/franka/go_safe_zone', self.on_go_safe_zone)
        self.create_service(Trigger, '/franka/go_work_zone', self.on_go_work_zone)
        self.create_service(Trigger, '/franka/stop', self.on_stop)

        self._auto_timer = None
        if self.auto_move_to_safe_zone:
            self._auto_timer = self.create_timer(self.startup_delay, self._go_to_safe_zone_once)

        self.get_logger().info(
            f'Publishing trajectories to {controller_topic}; '
            f'targets on /franka/joint_targets; auto_move_to_safe_zone={self.auto_move_to_safe_zone}')

    def on_joint_state(self, msg):
        positions = dict(zip(msg.name, msg.position))
        if all(name in positions for name in self.joint_names):
            self.current_positions = [positions[name] for name in self.joint_names]

    def on_joint_targets(self, msg):
        if not self.publish_positions(msg.data, self.move_duration):
            self.get_logger().error(
                f'Expected {len(self.joint_names)} joint positions, '
                f'received {len(msg.data)}')

    def _go_to_safe_zone_once(self):
        if self._auto_timer is not None:
            self._auto_timer.cancel()
            self._auto_timer = None
        if self.publish_positions(self.safe_zone_positions, self.move_duration):
            self.get_logger().info('Auto-moving to SAFE_ZONE')
        else:
            self.get_logger().error('Failed to auto-move to SAFE_ZONE')

    def on_go_safe_zone(self, request, response):
        if self.publish_positions(self.safe_zone_positions, self.move_duration):
            response.success = True
            response.message = 'Moving to SAFE_ZONE'
        else:
            response.success = False
            response.message = 'Invalid SAFE_ZONE configuration'
        return response

    def on_go_work_zone(self, request, response):
        if self.publish_positions(self.work_zone_positions, self.move_duration):
            response.success = True
            response.message = 'Moving to WORK_ZONE'
        else:
            response.success = False
            response.message = 'Invalid WORK_ZONE configuration'
        return response

    def on_home(self, request, response):
        return self.on_go_safe_zone(request, response)

    def on_stop(self, request, response):
        if self.current_positions is None:
            response.success = False
            response.message = 'No joint state received yet'
            return response

        if self.publish_positions(self.current_positions, 0.2):
            response.success = True
            response.message = 'Holding the current joint positions'
        else:
            response.success = False
            response.message = 'Could not publish the hold trajectory'
        return response

    def publish_positions(self, positions, duration):
        if len(positions) != len(self.joint_names):
            return False

        duration = max(0.5, float(duration))
        seconds = int(duration)
        nanoseconds = int(round((duration - seconds) * 1e9))

        point = JointTrajectoryPoint()
        point.positions = [float(position) for position in positions]
        point.velocities = [0.0] * len(positions)
        point.accelerations = [0.0] * len(positions)
        point.time_from_start = Duration(sec=seconds, nanosec=nanoseconds)

        trajectory = JointTrajectory()
        try:
            trajectory.header.stamp = self.get_clock().now().to_msg()
        except Exception:
            pass
        trajectory.joint_names = list(self.joint_names)
        trajectory.points = [point]
        self.trajectory_publisher.publish(trajectory)
        return True


def main(args=None):
    rclpy.init(args=args)
    node = FrankaComm()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
