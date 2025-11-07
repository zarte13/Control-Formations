import rclpy  
from rclpy.node import Node  
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode  
from mavros_msgs.msg import State, PositionTarget  
from std_msgs.msg import String, Float32, Bool  
from geometry_msgs.msg import Point  
from mavros.base import SENSOR_QOS  
  
class ArduPilotController(Node):  
    def __init__(self):  
        super().__init__('ardupilot_controller')  
          
        # Create service clients for commands  
        self.arming_client = self.create_client(  
            CommandBool,   
            '/mavros/cmd/arming'  
        )  
        self.takeoff_client = self.create_client(  
            CommandTOL,   
            '/mavros/cmd/takeoff'  
        )  
        self.set_mode_client = self.create_client(  
            SetMode,   
            '/mavros/set_mode'  
        )  
          
        # Publisher for position setpoints (NED coordinates)  
        self.setpoint_raw_pub = self.create_publisher(  
            PositionTarget,  
            '/mavros/setpoint_raw/local',  
            SENSOR_QOS  
        )  
          
        # Subscribe to state for monitoring  
        self.state_sub = self.create_subscription(  
            State,  
            '/mavros/state',  
            self.state_callback,  
            SENSOR_QOS  
        )  
          
        # Command topic subscribers  
        self.arm_sub = self.create_subscription(  
            Bool,  
            '~/cmd/arm',  
            self.arm_callback,  
            10  
        )  
          
        self.takeoff_sub = self.create_subscription(  
            Float32,  
            '~/cmd/takeoff',  
            self.takeoff_callback,  
            10  
        )  
          
        self.mode_sub = self.create_subscription(  
            String,  
            '~/cmd/set_mode',  
            self.mode_callback,  
            10  
        )  
          
        self.position_sub = self.create_subscription(  
            PositionTarget,  
            '~/cmd/goto_position',  
            self.position_callback,  
            10  
        )  
        self.status_pub = self.create_publisher(  
            State,  
            '~/status',  
            10  
        )  
          
        # State variables  
        self.current_state = State()  
        self.target_position = None  
        self.position_timer = None  
          
        self.get_logger().info('ArduPilot controller initialized')  
      
    def state_callback(self, msg):  
        self.current_state = msg  
        # Republish status for monitoring  
        self.status_pub.publish(msg)  
          
        # Log important state changes  
        if hasattr(self, '_last_armed') and self._last_armed != msg.armed:  
            self.get_logger().info(f'Armed status changed: {msg.armed}')  
        if hasattr(self, '_last_mode') and self._last_mode != msg.mode:  
            self.get_logger().info(f'Mode changed to: {msg.mode}')  
          
        self._last_armed = msg.armed  
        self._last_mode = msg.mode
      
    def arm_callback(self, msg):  
        """Handle arm/disarm command from topic"""  
        self.get_logger().info(f'{"Arming" if msg.data else "Disarming"}...')  
        req = CommandBool.Request()  
        req.value = msg.data  
          
        future = self.arming_client.call_async(req)  
        future.add_done_callback(  
            lambda f: self.get_logger().info(  
                f'{"Armed" if msg.data else "Disarmed"} successfully'   
                if f.result().success   
                else f'{"Arming" if msg.data else "Disarming"} failed'  
            )  
        )  
      
    def takeoff_callback(self, msg):  
        """Handle takeoff command from topic"""  
        altitude = msg.data  
        self.get_logger().info(f'Taking off to {altitude}m...')  
        req = CommandTOL.Request()  
        req.min_pitch = 0.0  
        req.yaw = 0.0  
        req.latitude = 0.0  
        req.longitude = 0.0  
        req.altitude = altitude  
          
        future = self.takeoff_client.call_async(req)  
        future.add_done_callback(  
            lambda f: self.get_logger().info(  
                'Takeoff command sent successfully'   
                if f.result().success   
                else 'Takeoff failed'  
            )  
        )  
      
    def mode_callback(self, msg):  
        """Handle mode change command from topic"""  
        mode = msg.data  
        self.get_logger().info(f'Setting mode to {mode}...')  
        req = SetMode.Request()  
        req.base_mode = 0  
        req.custom_mode = mode  
          
        future = self.set_mode_client.call_async(req)  
        future.add_done_callback(  
            lambda f: self.get_logger().info(  
                f'Mode set to {mode}'   
                if f.result().mode_sent   
                else f'Failed to set mode to {mode}'  
            )  
        )  
      
    def position_callback(self, msg):  
        """Handle position command from topic"""  
        self.target_position = msg  
          
        # Cancel existing timer if any  
        if self.position_timer is not None:  
            self.position_timer.cancel()  
          
        # Create timer to publish position at 10 Hz  
        self.position_timer = self.create_timer(0.1, self.publish_position)  
        self.get_logger().info(  
            f'Started publishing position: N={msg.position.x}, '  
            f'E={msg.position.y}, D={msg.position.z}'  
        )  
      
    def publish_position(self):  
        """Timer callback to continuously publish position setpoint"""  
        if self.target_position is not None:  
            msg = PositionTarget()  
            msg.header.stamp = self.get_clock().now().to_msg()  
            msg.header.frame_id = 'map'  
            msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED  
              
            # Set position only (ignore velocity and acceleration)  
            msg.type_mask = (  
                PositionTarget.IGNORE_VX |  
                PositionTarget.IGNORE_VY |  
                PositionTarget.IGNORE_VZ |  
                PositionTarget.IGNORE_AFX |  
                PositionTarget.IGNORE_AFY |  
                PositionTarget.IGNORE_AFZ |  
                PositionTarget.IGNORE_YAW_RATE  
            )  
              
            msg.position = self.target_position.position  
            msg.yaw = self.target_position.yaw  
              
            self.setpoint_raw_pub.publish(msg)  
  
def main(args=None):  
    rclpy.init(args=args)  
    controller = ArduPilotController()  
      
    try:  
        rclpy.spin(controller)  
    except KeyboardInterrupt:  
        pass  
    finally:  
        controller.destroy_node()  
        rclpy.try_shutdown()  
  
if __name__ == '__main__':  
    main()