#!/usr/bin/env python3
"""
Publishes human-readable drone status strings derived from PX4 vehicle_status.

Subscribes:
  /drone/fmu/out/vehicle_status_v1  (px4_msgs/VehicleStatus)

Publishes:
  /drone/status/mode    (std_msgs/String)  e.g. "POSITION", "OFFBOARD", "HOLD"
  /drone/status/arming  (std_msgs/String)  "ARMED" or "DISARMED"
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from px4_msgs.msg import VehicleStatus


NAV_STATE_NAMES: dict[int, str] = {
    0:  "MANUAL",
    1:  "ALTITUDE",
    2:  "POSITION",
    3:  "MISSION",
    4:  "HOLD",
    5:  "RTL",
    6:  "RC_RECOVERY",
    7:  "RTL_USERDEFINED",
    8:  "DESCEND",
    9:  "TERMINATION",
    10: "ACRO",
    12: "LAND",
    13: "AUTO_TAKEOFF",
    14: "OFFBOARD",
    15: "STABILIZED",
    17: "TAKEOFF",
    18: "LAND",
    19: "FOLLOW_TARGET",
    20: "PRECISION_LAND",
    21: "ORBIT",
    22: "VTOL_TAKEOFF",
}

ARMING_STATE_NAMES: dict[int, str] = {
    0: "INIT",
    1: "STANDBY",
    2: "ARMED",
    3: "STANDBY_ERROR",
    4: "SHUTDOWN",
    5: "IN_AIR_RESTORE",
}


class DroneStatusNode(Node):
    def __init__(self) -> None:
        super().__init__("drone_status")

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.mode_pub   = self.create_publisher(String, "/drone/status/mode",   10)
        self.arming_pub = self.create_publisher(String, "/drone/status/arming", 10)

        self.create_subscription(
            VehicleStatus,
            "/drone/fmu/out/vehicle_status_v1",
            self._vehicle_status_cb,
            px4_qos,
        )

        self.get_logger().info("drone_status node started")

    def _vehicle_status_cb(self, msg: VehicleStatus) -> None:
        mode   = NAV_STATE_NAMES.get(msg.nav_state, f"MODE_{msg.nav_state}")
        arming = ARMING_STATE_NAMES.get(msg.arming_state, f"ARMING_{msg.arming_state}")

        self.mode_pub.publish(String(data=mode))
        self.arming_pub.publish(String(data=arming))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DroneStatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
