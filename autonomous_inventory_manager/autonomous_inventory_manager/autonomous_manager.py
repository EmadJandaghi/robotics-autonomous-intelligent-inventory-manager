#!/usr/bin/env python3
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String

from custom_msgs.srv import SetAOI, PerformInventory, FaceLine


class AutonomousManager(Node):
    """
    Drives the sequence:
      For each AOI in the list:
        - call SetAOI(color)
        - wait for /aoi_arrived
        - call PerformInventory()
        - (optional) call FaceLine()
        - continue
    Everything is logged verbosely.
    """

    def __init__(self) -> None:
        super().__init__("autonomous_manager")
        self.get_logger().info("[Manager] Initializing...")

        # Mission AOIs (edit order as you like)
        self.aoi_list = ["yellow", "green", "red", "blue"]
        self.idx = 0

        # Events / state
        self.arrived = False
        self.state = "IDLE"

        # Publishers/Subscribers
        self.aoi_choice_pub = self.create_publisher(String, "/aoi_choice", 10)
        self.create_subscription(Bool, "/aoi_arrived", self.arrived_cb, 10)

        # Service clients
        self.cli_set_aoi = self.create_client(SetAOI, "SetAOI")
        self.cli_inventory = self.create_client(PerformInventory, "PerformInventory")
        self.cli_face_line = self.create_client(FaceLine, "FaceLine")

        self.wait_service(self.cli_set_aoi, "SetAOI")
        self.wait_service(self.cli_inventory, "PerformInventory")
        self.wait_service(self.cli_face_line, "FaceLine")

        # Timer main loop
        self.timer = self.create_timer(0.5, self.loop)
        self.get_logger().info("[Manager] Ready.")

    # ----------- Utilities -----------

    def wait_service(self, client, name: str):
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"[Manager] Waiting for {name} service...")

    def call(self, client, req, name: str):
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        if not fut.result():
            self.get_logger().error(f"[Manager] {name} call → no response.")
            return None
        self.get_logger().info(f"[Manager] {name} response OK.")
        return fut.result()

    def arrived_cb(self, msg: Bool):
        if msg.data:
            self.arrived = True
            self.get_logger().info("[Manager] /aoi_arrived received: True")

    def publish_choice(self, color: str):
        m = String()
        m.data = color
        self.aoi_choice_pub.publish(m)
        self.get_logger().info(f"[Manager] Published /aoi_choice='{color}' (for compatibility/logging).")

    # ----------- Main loop -----------

    def loop(self):
        if self.state == "IDLE":
            if self.idx >= len(self.aoi_list):
                self.get_logger().info("[Manager] Mission complete. No more AOIs.")
                self.state = "DONE"
                return

            color = self.aoi_list[self.idx]
            self.get_logger().info(f"[Manager] >>> Starting AOI '{color}' ({self.idx+1}/{len(self.aoi_list)})")

            # Broadcast choice (legacy) and call SetAOI
            self.publish_choice(color)
            req = SetAOI.Request()
            req.color = color
            resp = self.call(self.cli_set_aoi, req, "SetAOI")
            if not resp or not resp.accepted:
                self.get_logger().error(f"[Manager] SetAOI NOT accepted: {resp.message if resp else 'no resp'}")
                self.idx += 1
                self.state = "IDLE"
                return

            self.get_logger().info(f"[Manager] SetAOI accepted → robot should start moving.")
            self.arrived = False
            self.state = "MOVING"

        elif self.state == "MOVING":
            if self.arrived:
                self.get_logger().info("[Manager] Arrived at AOI. Triggering PerformInventory...")
                inv_resp = self.call(self.cli_inventory, PerformInventory.Request(), "PerformInventory")
                if inv_resp and inv_resp.success:
                    self.get_logger().info(
                        f"[Manager] Inventory result (AOI={inv_resp.aoi_color}): "
                        f"total={inv_resp.total_json} missing={inv_resp.missing_json} misplaced={inv_resp.misplaced_json}"
                    )
                else:
                    self.get_logger().warn("[Manager] PerformInventory failed or returned error.")

                # Optional: reacquire line (enable if needed)
                # face_resp = self.call(self.cli_face_line, FaceLine.Request(), "FaceLine")
                # self.get_logger().info(f"[Manager] FaceLine returned: success={face_resp.success if face_resp else 'n/a'}")

                # Next AOI
                self.idx += 1
                self.state = "IDLE"

        elif self.state == "DONE":
            # Nothing else to do; you could stop the timer if you want.
            return


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousManager()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # Spin in background thread so service calls never block other nodes
    t = threading.Thread(target=executor.spin, daemon=True)
    t.start()

    try:
        while rclpy.ok():
            time.sleep(0.5)
    except KeyboardInterrupt:
        node.get_logger().info("[Manager] Shutting down (KeyboardInterrupt).")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
