import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.duration import Duration

class DroneVelocityCommander(Node):
    def __init__(self):
        super().__init__('drone_velocity_commander')
        
        # Publisher to send velocity commands
        self.velocity_publisher = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        
        # Timer to send velocity commands at regular intervals
        self.command_timer = self.create_timer(0.1, self.send_velocity_command)

        # Duration to send the command (e.g., 5 seconds)
        self.command_duration = Duration(seconds=5.0)

        # Store the start time
        self.start_time = self.get_clock().now()

        # Define the desired velocity
        self.velocity_command = Twist()
        self.velocity_command.linear.x = 1.0  # Move forward at 1 m/s
        self.velocity_command.linear.y = 0.0
        self.velocity_command.linear.z = 0.0
        self.velocity_command.angular.x = 0.0
        self.velocity_command.angular.y = 0.0
        self.velocity_command.angular.z = 0.0

    def send_velocity_command(self):
        # Check how much time has passed since the command started
        elapsed_time = self.get_clock().now() - self.start_time

        if elapsed_time < self.command_duration:
            # Publish the velocity command
            self.velocity_publisher.publish(self.velocity_command)
            self.get_logger().info(f'Sending velocity command: {self.velocity_command.linear.x} m/s')
        else:
            # Stop sending commands after the duration has passed
            self.command_timer.cancel()
            self.stop_drone()

    def stop_drone(self):
        # Send a zero velocity command to stop the drone
        stop_command = Twist()
        self.velocity_publisher.publish(stop_command)
        self.get_logger().info('Stopping the drone.')

def main(args=None):
    rclpy.init(args=args)
    drone_velocity_commander = DroneVelocityCommander()

    try:
        rclpy.spin(drone_velocity_commander)
    except KeyboardInterrupt:
        drone_velocity_commander.stop_drone()

    drone_velocity_commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
