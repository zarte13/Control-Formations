import rclpy  
from rclpy.node import Node  
from std_msgs.msg import String, Bool, Float32  
from mavros_msgs.msg import State, PositionTarget  
from geometry_msgs.msg import PoseStamped  
from .qos_profiles import QOS_RELIABLE_DEFAULT  
  
class MissionNode(Node):  
    def __init__(self):  
        super().__init__("mission_node")  
          
        # State machine  
        self.state = "WAIT_CONNECTION"  
        self.mission_start_time = None  
          
        # MAVROS state subscriber  
        self.state_sub = self.create_subscription(  
            State,  
            "/mavros/state",  
            self.state_callback,  
            qos_profile=QOS_RELIABLE_DEFAULT,  
        )  
          
        # Balloon pose subscriber  
        self.balloon_sub = self.create_subscription(  
            PoseStamped,  
            "/Ballon_pose",  
            self.balloon_callback,  
            10  
        )  
          
        # Command publishers (to ArduPilotController topics)  
        self.arm_pub = self.create_publisher(Bool, '/ardupilot_controller/cmd/arm', 10)  
        self.mode_pub = self.create_publisher(String, '/ardupilot_controller/cmd/set_mode', 10)  
        self.takeoff_pub = self.create_publisher(Float32, '/ardupilot_controller/cmd/takeoff', 10)  
        self.position_pub = self.create_publisher(PositionTarget, '/ardupilot_controller/cmd/goto_position', 10)  
          
        # Arrival publisher  
        self.arrival_pub = self.create_publisher(String, '/arrival', 10)  
          
        # Current state  
        self.current_state = State()  
        self.latest_balloon_pose = None  
          
        # State machine timer (runs at 10 Hz)  
        self.state_machine_timer = self.create_timer(0.1, self.state_machine_callback)  
          
        # Tracking timer (runs at 10 Hz when tracking)  
        self.tracking_timer = None  
          
        self.get_logger().info("Mission node started")  
      
    def state_callback(self, msg: State):  
        self.current_state = msg  
      
    def balloon_callback(self, msg: PoseStamped):  
        """Store latest balloon pose for tracking"""  
        self.latest_balloon_pose = msg  
        if self.state == "FOLLOW_BALLOON":  
            self.get_logger().info(  
                f"Balloon pose - Position: [{msg.pose.position.x:.2f}, "  
                f"{msg.pose.position.y:.2f}, {msg.pose.position.z:.2f}] (ENU)"  
            )  
      
    def follow_balloon_callback(self):  
        """Timer callback to continuously track balloon position"""  
        if self.latest_balloon_pose is not None:  
            # Create position target from balloon pose  
            pos_msg = PositionTarget()  
            pos_msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED  
            pos_msg.type_mask = (  
                PositionTarget.IGNORE_VX |  
                PositionTarget.IGNORE_VY |  
                PositionTarget.IGNORE_VZ |  
                PositionTarget.IGNORE_AFX |  
                PositionTarget.IGNORE_AFY |  
                PositionTarget.IGNORE_AFZ |  
                PositionTarget.IGNORE_YAW_RATE  
            )  
              
            # Convert ENU to NED coordinates  
            # ENU: x=East, y=North, z=Up  
            # NED: x=North, y=East, z=Down  
            pos_msg.position.x = self.latest_balloon_pose.pose.position.y  # North  
            pos_msg.position.y = self.latest_balloon_pose.pose.position.x  # East  
            pos_msg.position.z = self.latest_balloon_pose.pose.position.z  # Up
            pos_msg.yaw = 0.0  
              
            self.position_pub.publish(pos_msg)  
      
    def state_machine_callback(self):  
        """Main state machine - executes mission steps"""  
          
        if self.state == "WAIT_CONNECTION":  
            if self.current_state.connected:  
                self.get_logger().info("FCU connected! Starting mission...")  
                self.state = "SET_GUIDED"  
              
        elif self.state == "SET_GUIDED":  
            self.get_logger().info("Setting mode to GUIDED...")  
            mode_msg = String()  
            mode_msg.data = 'GUIDED'  
            self.mode_pub.publish(mode_msg)  
            self.state = "VERIFY_GUIDED"  
            self.state_start_time = self.get_clock().now()  
              
        elif self.state == "VERIFY_GUIDED":  
            if self.current_state.mode == "GUIDED":  
                self.get_logger().info("Mode set to GUIDED successfully")  
                self.state = "ARM"  
            elif (self.get_clock().now() - self.state_start_time).nanoseconds > 5e9:  
                self.get_logger().error("Timeout waiting for GUIDED mode")  
                self.state = "SET_GUIDED"  
                  
        elif self.state == "ARM":  
            self.get_logger().info("Arming...")  
            arm_msg = Bool()  
            arm_msg.data = True  
            self.arm_pub.publish(arm_msg)  
            self.state = "VERIFY_ARMED"  
            self.state_start_time = self.get_clock().now()  
              
        elif self.state == "VERIFY_ARMED":  
            if self.current_state.armed:  
                self.get_logger().info("Armed successfully")  
                self.state = "TAKEOFF"  
            elif (self.get_clock().now() - self.state_start_time).nanoseconds > 10e9:  
                self.get_logger().error("Timeout waiting for arming")  
                self.state = "ARM"  
                  
        elif self.state == "TAKEOFF":  
            self.get_logger().info("Taking off to 50m...")  
            takeoff_msg = Float32()  
            takeoff_msg.data = 50.0  
            self.takeoff_pub.publish(takeoff_msg)  
            self.state = "WAIT_TAKEOFF"  
            self.state_start_time = self.get_clock().now()  
              
        elif self.state == "WAIT_TAKEOFF":  
            # Wait 15 seconds for takeoff to complete  
            if (self.get_clock().now() - self.state_start_time).nanoseconds > 15e9:  
                self.get_logger().info("Takeoff complete, proceeding to waypoint")  
                self.state = "GOTO_POSITION"  
                  
        elif self.state == "GOTO_POSITION":  
            self.get_logger().info("Going to position N=10.0, E=20.0, U=50.0...")  
            pos_msg = PositionTarget()  
            pos_msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED  
            pos_msg.type_mask = (  
                PositionTarget.IGNORE_VX |  
                PositionTarget.IGNORE_VY |  
                PositionTarget.IGNORE_VZ |  
                PositionTarget.IGNORE_AFX |  
                PositionTarget.IGNORE_AFY |  
                PositionTarget.IGNORE_AFZ |  
                PositionTarget.IGNORE_YAW_RATE  
            )  
            # NEU coordinates: N=10, E=20, U=50 (up is positive)  
            # Convert to NED: N=10, E=20, D=-50 (down is positive in NED)  
            pos_msg.position.x = 10.0  
            pos_msg.position.y = 20.0  
            pos_msg.position.z = 50.0  
            pos_msg.yaw = 0.0  
            self.position_pub.publish(pos_msg)  
            self.state = "WAIT_POSITION"  
            self.state_start_time = self.get_clock().now()  
              
        elif self.state == "WAIT_POSITION":  
            # Wait 20 seconds for position to be reached  
            if (self.get_clock().now() - self.state_start_time).nanoseconds > 20e9:  
                self.get_logger().info("Position reached")  
                self.state = "SEND_ARRIVAL"  
                  
        elif self.state == "SEND_ARRIVAL":  
            self.get_logger().info("Sending arrival message...")  
            arrival_msg = String()  
            arrival_msg.data = "Philippe"  
            self.arrival_pub.publish(arrival_msg)  
            self.state = "FOLLOW_BALLOON"  
            self.state_start_time = self.get_clock().now()  
              
            # Start tracking timer at 10 Hz  
            self.tracking_timer = self.create_timer(0.1, self.follow_balloon_callback)  
            self.get_logger().info("Started following balloon...")  
              
        elif self.state == "FOLLOW_BALLOON":  
            elapsed = (self.get_clock().now() - self.state_start_time).nanoseconds / 1e9  
            if elapsed >= 120.0:  
                self.get_logger().info("Tracking complete (120 seconds)")  
                # Stop tracking timer  
                if self.tracking_timer is not None:  
                    self.tracking_timer.cancel()  
                    self.tracking_timer = None  
                self.state = "RTL"  
              
        elif self.state == "RTL":  
            self.get_logger().info("Returning to launch...")  
            mode_msg = String()  
            mode_msg.data = 'RTL'  
            self.mode_pub.publish(mode_msg)  
            self.state = "VERIFY_RTL"  
            self.state_start_time = self.get_clock().now()  
              
        elif self.state == "VERIFY_RTL":  
            if self.current_state.mode == "RTL":  
                self.get_logger().info("RTL mode set successfully. Mission complete!")  
                self.state = "MISSION_COMPLETE"  
                self.state_machine_timer.cancel()  
            elif (self.get_clock().now() - self.state_start_time).nanoseconds > 5e9:  
                self.get_logger().error("Timeout waiting for RTL mode")  
                self.state = "RTL"  
  
def main():  
    rclpy.init()  
    node = MissionNode()  
    try:  
        rclpy.spin(node)  
    finally:  
        node.destroy_node()  
        rclpy.shutdown()  
  
if __name__ == "__main__":  
    main()