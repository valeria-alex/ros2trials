import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class DroneNavigator(Node):
    def __init__(self):
        super().__init__('drone_navigator')
        # Publisher to send position commands to the drone
        self.pose_publisher = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)

        # Timer to send commands at a regular interval
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_position)

        # Set the initial target position
        self.target_pose = PoseStamped()
        self.target_pose.pose.position.x = 0.0
        self.target_pose.pose.position.y = 0.0
        self.target_pose.pose.position.z = 2.0  # Hover at 2 meters altitude

    def publish_position(self):
        # Update the timestamp
        self.target_pose.header.stamp = self.get_clock().now().to_msg()

        # Log the current target position
        self.get_logger().info(f'Navigating to x={self.target_pose.pose.position.x}, y={self.target_pose.pose.position.y}, z={self.target_pose.pose.position.z}')

        # Publish the pose command
        self.pose_publisher.publish(self.target_pose)

def main(args=None):
    rclpy.init(args=args)
    drone_navigator = DroneNavigator()

    try:
        rclpy.spin(drone_navigator)
    except KeyboardInterrupt:
        pass

    drone_navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
