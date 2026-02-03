import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import numpy as np

from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import TransformStamped

class FeedbackController(Node):
    """A simple feedback controller for the turtlebot.

    Publishes commands to /cmd_vel to drive the robot to a goal location.

    Listens to the the tf2 transform tree to get the robot's current position
    estimate.
    """
    def __init__(self):
        """Create the feedback controller node."""
        super().__init__("feedback_controller")

        # Listen for transforms, which represent positions and orientations of
        # various quantities. The localization system will be publishing these
        # special messages. 
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Operate at 10 Hz.
        self.timer = self.create_timer(0.1, self.timer_callback)

        # We'll publish to our usual /cmd_vel topic to drive the robot.
        self.publisher = self.create_publisher(TwistStamped, "/cmd_vel", 10)

        # Record the time that this node was started at.
        self.start_time = self.get_clock().now()

    def get_robot_pose(self):
        """Get the current robot pose from tf2.

        Returns:
            A tuple (x, y, theta) representing the robot's current position
            and orientation in the map frame.
        """
        try:
            # Look up the latest transform from "map" to "base_link".
            transform = self.tf_buffer.lookup_transform(
                "map",
                "base_link",
                rclpy.time.Time()
            )

            # Convert the transform into the (x, y, theta) representation, since
            # we know the robot is on a plane.
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            qw = transform.transform.rotation.w
            qz = transform.transform.rotation.z
            theta = 2.0 * np.arctan2(qz, qw)

            return (x, y, theta)

        except Exception as e:
            # In some cases, such as when the localization node is not yet
            # running, we won't have an up-to-date pose estimate available. In
            # That case, 
            self.get_logger().warn(f"Unable to compute robot pose: {e}")
            return None, None, None

    def timer_callback(self):
        """Publish a hello world message, and also log what we've sent."""
        # Get the current time, in seconds, since the node was started.
        t = self.get_clock().now() - self.start_time
        t_seconds = t.nanoseconds / 1e9
        
        # Create the message. By default, this will have be zeros.
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()

        # Get the current position and orientation of the robot.
        x, y, theta = self.get_robot_pose()
        if x is not None:
            print(f"t={t_seconds:.2f} x={x:.2f} y={y:.2f} theta={theta:.2f}")

            ########################################################
            # YOUR CONTROLLER IMPLEMENTATION HERE
            ########################################################

        # Apply velocity limits
        msg.twist.linear.x = np.clip(msg.twist.linear.x, -0.2, 0.2)
        msg.twist.angular.z = np.clip(msg.twist.angular.z, -1.0, 1.0)

        # Publish the command velocity message.
        self.publisher.publish(msg)


def main():
    rclpy.init(args=None)

    # Start up our node.
    feedback_controller = FeedbackController()

    # Wait until the user exits with CTRL+C.
    rclpy.spin(feedback_controller)

    # Clean up.
    feedback_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
