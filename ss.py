import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
import time
import random


class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control')

        # Publisher for position control
        self.pose_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)

        # QoS settings
        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=10)

        # Subscribers
        self.local_pos_sub = self.create_subscription(Odometry, '/mavros/local_position/odom', self.local_position_callback, qos_profile)
        self.gps_sub = self.create_subscription(NavSatFix, '/mavros/global_position/global', self.gps_callback, qos_profile)
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, 10)

        # Service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # Ensure services are available
        self.wait_for_services()

        # Timer to continuously send position setpoints
        self.timer = self.create_timer(0.1, self.publish_pose)

        # Setpoints
        self.setpoints = [
            [0.0, 0.0, 20.0],
            [-50.0, -70.0, 20.0],    # Takeoff position
            [40.0, 20.0, 20.0],    # Second setpoint
             # Third setpoint
            [-30.0, -50.0, 20.0],  # Fourth setpoint
        ]
        self.current_setpoint_index = 0

        # Initialize position
        self.pose = PoseStamped()
        self.update_pose()

        # State tracking
        self.reached_setpoint = False
        self.current_state = None
        self.current_lat = None
        self.current_lon = None

    def wait_for_services(self):
        """ Waits for required MAVROS services to be available """
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for arming service...')
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_mode service...')

    def update_pose(self):
        """ Updates the current pose target """
        target = self.setpoints[self.current_setpoint_index]
        self.pose.pose.position.x, self.pose.pose.position.y, self.pose.pose.position.z = target

    def publish_pose(self):
        """ Publishes the current target pose """
        self.pose.header.stamp = self.get_clock().now().to_msg()
        self.pose_pub.publish(self.pose)

    def arm_and_takeoff(self):
        """ Arms the drone and sets OFFBOARD mode """
        self.get_logger().info('Publishing initial setpoints...')
        for _ in range(10):  # Publish some setpoints before switching modes
            self.publish_pose()
            time.sleep(0.1)

        self.get_logger().info('Switching to OFFBOARD mode...')
        mode_req = SetMode.Request()
        mode_req.custom_mode = 'OFFBOARD'
        self.set_mode_client.call_async(mode_req)
        time.sleep(2)

        self.get_logger().info('Arming drone...')
        arm_req = CommandBool.Request()
        arm_req.value = True
        self.arming_client.call_async(arm_req)
        self.get_logger().info('Takeoff initiated.')

    def local_position_callback(self, msg):
        """ Monitors drone position and checks if a setpoint is reached """
        current_position = msg.pose.pose.position
        target_position = self.setpoints[self.current_setpoint_index]

        # Check proximity to target
        if (abs(current_position.x - target_position[0]) < 0.2 and
            abs(current_position.y - target_position[1]) < 0.2 and
            abs(current_position.z - target_position[2]) < 0.2):

            if not self.reached_setpoint:
                self.reached_setpoint = True
                self.log_sensor_data()

                # Hover at the setpoint for 5 seconds
                time.sleep(5)

                # Move to the next setpoint
                if self.current_setpoint_index < len(self.setpoints) - 1:
                    self.current_setpoint_index += 1
                    self.update_pose()
                    self.reached_setpoint = False
                else:
                    self.get_logger().info('Mission complete. All setpoints reached.')

    def log_sensor_data(self):
        """ Logs sensor data when a setpoint is reached """
        fake_lidar = round(random.uniform(1.5, 8.0), 2)  # Simulated Lidar data
        fake_co2 = round(random.uniform(350, 480), 2)  # Simulated CO2 levels

        lat = self.current_lat if self.current_lat is not None else "Unknown"
        lon = self.current_lon if self.current_lon is not None else "Unknown"

        self.get_logger().info(f'Setpoint {self.current_setpoint_index + 1} reached.')
        self.get_logger().info(f'GPS Coordinates: Lat {lat}, Lon {lon}')
        self.get_logger().info(f'Lidar Distance: {fake_lidar}m')
        self.get_logger().info(f'CO2 Thermal Data: {fake_co2} ppm')

    def gps_callback(self, msg):
        """ Updates GPS coordinates """
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

    def state_callback(self, msg):
        """ Monitors MAVROS state """
        self.current_state = msg


def main(args=None):
    rclpy.init(args=args)
    node = OffboardControl()
    node.arm_and_takeoff()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

