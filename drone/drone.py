import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool, SetMode

class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control_node')

        # Best effort QoS is usually required for MAVROS topics
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_cb, qos_profile)
        self.attitude_pub = self.create_publisher(AttitudeTarget, '/mavros/setpoint_raw/attitude', qos_profile)
        
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.current_state = State()
        self.mode_req_time = self.get_clock().now()
        self.arm_req_time = self.get_clock().now()
        
        self.timer = self.create_timer(0.05, self.timer_callback) # 20Hz
        self.start_time = self.get_clock().now()
        self.get_logger().info("Offboard control node initialized. Waiting for connection...")

    def state_cb(self, msg):
        if not self.current_state.connected and msg.connected:
            self.get_logger().info("Connected to MAVROS!")
        if self.current_state.mode != msg.mode:
            self.get_logger().info(f"Mode changed to {msg.mode}")
        if not self.current_state.armed and msg.armed:
            self.get_logger().info("Vehicle armed!")
        self.current_state = msg

    def timer_callback(self):
        # Define 5% throttle, level attitude
        cmd = AttitudeTarget()
        cmd.orientation.w = 1.0
        cmd.type_mask = AttitudeTarget.IGNORE_ROLL_RATE | AttitudeTarget.IGNORE_PITCH_RATE | AttitudeTarget.IGNORE_YAW_RATE
        cmd.thrust = 0.05
        
        self.attitude_pub.publish(cmd)

        # Wait for connection
        if not self.current_state.connected:
            return

        # Handshake: Only try to switch mode after 2 seconds of streaming
        now = self.get_clock().now()
        if (now - self.start_time).nanoseconds > 2e9:
            if self.current_state.mode != "OFFBOARD":
                if (now - self.mode_req_time).nanoseconds > 1e9:  # Limit requests to 1Hz
                    self.set_offboard_mode()
                    self.mode_req_time = now
            elif not self.current_state.armed:
                if (now - self.arm_req_time).nanoseconds > 1e9:  # Limit requests to 1Hz
                    self.arm_vehicle()
                    self.arm_req_time = now

    def set_offboard_mode(self):
        if not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/mavros/set_mode service not available')
            return
        self.get_logger().info('Requesting OFFBOARD mode...')
        req = SetMode.Request()
        req.custom_mode = "OFFBOARD"
        future = self.set_mode_client.call_async(req)
        future.add_done_callback(self.mode_cb)

    def mode_cb(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Switch mode result: {response.mode_sent}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def arm_vehicle(self):
        if not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/mavros/cmd/arming service not available')
            return
        self.get_logger().info('Requesting Arm...')
        req = CommandBool.Request()
        req.value = True
        future = self.arming_client.call_async(req)
        future.add_done_callback(self.arm_cb)

    def arm_cb(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Arming result: {response.success}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = OffboardControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
