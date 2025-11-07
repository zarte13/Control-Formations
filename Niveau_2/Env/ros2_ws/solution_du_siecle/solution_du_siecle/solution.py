import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from mavros_msgs.msg import State
from .qos_profiles import QOS_RELIABLE_DEFAULT

class ReliableNode(Node):
    def __init__(self):
        super().__init__("heartbeat_tester")


        # MAVROS state subscriber (use explicit QoS if you want reliable)
        self.state_sub = self.create_subscription(
            State,
            "/mavros/state",
            self.state_callback,
            qos_profile=QOS_RELIABLE_DEFAULT,
        )

        self.get_logger().info("ReliableNode started with RELIABLE QoS")


    def state_callback(self, msg: State):
        self.get_logger().info(
            f"Heartbeat received - Connected: {msg.connected}, "
            f"Armed: {msg.armed}, Mode: {msg.mode}"
        )

def main():
    rclpy.init()
    node = ReliableNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()