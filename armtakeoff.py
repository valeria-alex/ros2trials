import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode

class DroneMission(Node):
    def __init__(self):
        super().__init__('drone_mission')

        # Publishers
        self.pose_publisher = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        
        # Service Clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # Ensure connections to services
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Arming service...')
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for SetMode service...')

        # Takeoff parameters
        self.takeoff_altitude = 2.0
        self.target_pose = PoseStamped()
        self.target_pose.pose.position.x = 0.0
        self.target_pose.pose.position.y = 0.0
        self.target_pose.pose.position.z = self.takeoff_altitude

        # Timer
        self.timer = self.create_timer(0.1, self.publish_position)

        # Start mission
        self.start_mission()

    def start_mission(self):
        # Set mode to OFFBOARD and arm the drone
        self.set_mode('OFFBOARD')
        self.arm_drone()

    def set_mode(self, mode):
        request = SetMode.Request()
        request.custom_mode = mode
        future = self.set_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result().mode_sent:
            self.get_logger().info(f'{mode} mode set successfully')
        else:
            self.get_logger().error(f'Failed to set {mode} mode')

    def arm_drone(self):
        request = CommandBool.Request()
        request.value = True
        future = self.arming_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info('Drone armed successfully')
        else:
            self.get_logger().error('Failed to arm drone')

    def publish_position(self):
        # Publish the target position for takeoff
        self.target_pose.header.stamp = self.get_clock().now().to_msg()
        self.pose_publisher.publish(self.target_pose)

    def land(self):
        # Land the drone
        self.target_pose.pose.position.z = 0.0
        self.get_logger().info('Landing...')
        self.publish_position()

def main(args=None):
    rclpy.init(args=args)
    drone_mission = DroneMission()

    try:
        rclpy.spin(drone_mission)
    except KeyboardInterrupt:
        drone_mission.land()

    drone_mission.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
