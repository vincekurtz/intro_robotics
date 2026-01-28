import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class HelloWorldPublisher(Node):
    """A example of a simple publisher.

    Adapted from https://docs.ros.org/en/jazzy/Tutorials/.
    """
    def __init__(self):
        """Create a node with a single simple publisher."""
        super().__init__("hello_world")

        # We'll publish a `String` on the `/hello_world` topic, and use
        # a queue size of 10.
        self.publisher = self.create_publisher(String, "hello_world", 10)

        # Publish messages every 0.5 seconds.
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Record the time that this node was started at.
        self.start_time = self.get_clock().now()

    def timer_callback(self):
        """Publish a hello world message, and also log what we've sent."""
        # Get the current time, in seconds, since the node was started.
        t = self.get_clock().now() - self.start_time
        t_seconds = t.nanoseconds / 1e9

        # Create the message
        msg = String()
        msg.data = f"Hello world! This node has been running for {t_seconds} seconds."

        # Publish the message.
        self.publisher.publish(msg)

        # Print the message to the terminal so we can see it from the
        # running node too.
        self.get_logger().info(f"{msg.data}")

def main():
    rclpy.init(args=None)

    # Start up our publisher node.
    hello_world_publisher = HelloWorldPublisher()

    # Wait until the user exits with CTRL+C.
    rclpy.spin(hello_world_publisher)
   
    # Clean up.
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
