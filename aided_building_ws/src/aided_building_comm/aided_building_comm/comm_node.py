import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


DEFAULT_JOINT_NAMES = [f'fr3_joint{i}' for i in range(1, 8)]
DEFAULT_HOME = [0.0, -0.785398, 0.0, -2.356194, 0.0, 1.570796, 0.785398]


class FrankaComm(Node):

    def __init__(self):
        super().__init__('franka_comm')

        self.joint_names = list(self.declare_parameter(
            'joint_names', DEFAULT_JOINT_NAMES).value)
        self.home_positions = [float(value) for value in self.declare_parameter(
            'home_positions', DEFAULT_HOME).value]
        self.move_duration = float(self.declare_parameter(
            'move_duration', 3.0).value)
        controller_topic = self.declare_parameter(
            'controller_topic', '/fr3_arm_controller/joint_trajectory').value

        if len(self.joint_names) != len(self.home_positions):
            raise ValueError(
                'joint_names and home_positions must have the same length')

        self.current_positions = None
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory, controller_topic, 10)
        self.create_subscription(
            Float64MultiArray, '/franka/joint_targets',
            self.on_joint_targets, 10)
        self.create_subscription(
            JointState, '/joint_states', self.on_joint_state, 10)
        self.create_service(Trigger, '/franka/home', self.on_home)
        self.create_service(Trigger, '/franka/stop', self.on_stop)

        self.get_logger().info(
            f'Publishing trajectories to {controller_topic}; '
            f'targets on /franka/joint_targets')

    def on_joint_state(self, msg):
        positions = dict(zip(msg.name, msg.position))
        if all(name in positions for name in self.joint_names):
            self.current_positions = [positions[name] for name in self.joint_names]

    def on_joint_targets(self, msg):
        if not self.publish_positions(msg.data, self.move_duration):
            self.get_logger().error(
                f'Expected {len(self.joint_names)} joint positions, '
                f'received {len(msg.data)}')

    def on_home(self, request, response):
        if self.publish_positions(self.home_positions, self.move_duration):
            response.success = True
            response.message = 'Moving to the home configuration'
        else:
            response.success = False
            response.message = 'Invalid home configuration'
        return response

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

        seconds = int(duration)
        nanoseconds = int(round((duration - seconds) * 1e9))

        point = JointTrajectoryPoint()
        point.positions = [float(position) for position in positions]
        point.velocities = [0.0] * len(positions)
        point.time_from_start = Duration(sec=seconds, nanosec=nanoseconds)

        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
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
