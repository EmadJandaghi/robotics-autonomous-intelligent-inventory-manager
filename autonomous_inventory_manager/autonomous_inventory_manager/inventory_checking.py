#!/usr/bin/env python3
import threading, time
from typing import Dict
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from custom_msgs.msg import ObjectCount
from autonomous_inventory_manager.trajectory_sender import TrajectorySender

class InventoryChecking(Node):
    def __init__(self) -> None:
        super().__init__('inventory_checking')

        self.traj = TrajectorySender(self)
        self.upper_count: Dict[str,int] = {}
        self.lower_count: Dict[str,int] = {}
        self.detected_objects: str = ""
        self.aoi_choice: str = ""
        self.aoi_reached = False 
        self.busy = False          

        # pubs/subs
        self.pub_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.inventory_done = self.create_publisher(Bool,'/inventory_done', 10)

        self.create_subscription(ObjectCount, '/detected_object_count', self.object_callback, 10)
        self.create_subscription(String, '/aoi_choice', self.choice_callback, 10)
        self.create_subscription(Bool, '/aoi_arrived', self.arrived_callback, 10)

        # services
        self.start_cli = self.create_client(Trigger, 'start_detection')
        self.stop_cli  = self.create_client(Trigger,  'stop_detection')
        self.start_cli.wait_for_service()
        self.stop_cli.wait_for_service()

        self.get_logger().info('[Inventory] Ready')

    # ---------- callbacks ----------
    def choice_callback(self, msg: String) -> None:
        self.aoi_choice = msg.data

    def object_callback(self, msg: ObjectCount) -> None:
        if not self.detected_objects:
            return
        counts = {msg.classes[i]: msg.counts[i] for i in range(len(msg.classes))}
        self.get_logger().info(f'[Inventory] {self.detected_objects}: {counts}')
        if self.detected_objects == 'upper':
            self.upper_count = counts
        elif self.detected_objects == 'lower':
            self.lower_count = counts
        self.detected_objects = ""

    def arrived_callback(self, msg: Bool) -> None:
        if not msg.data or self.busy:
            return
        self.busy = True
        self.get_logger().info('[Inventory] AOI reached → starting background thread')
        threading.Thread(target=self.perform_inventory, daemon=True).start()


    def call_trigger_sync(self, client, name: str) -> bool:
        """Synchronous Trigger call from the background thread (no executor spin)."""
        req = Trigger.Request()
        fut = client.call_async(req)
        # Wait for completion by polling; do NOT spin here.
        while not fut.done():
            time.sleep(0.01)
        res = fut.result()
        ok = bool(res and res.success)
        self.get_logger().info(f'[Inventory] {name}: {res.message if res else "failed"}')
        return ok

    # ---------- main routine ----------
    def perform_inventory(self) -> None:
        try:
            # Take control of the base so AOI follower can’t move it anymore
            # self.zero_base()

            # ----- Upper shelf -----
            self.get_logger().info('[Inventory] Move head ↑')
            self.traj.send_trajectory(
                ['deepmind_robot1_head_base_joint','deepmind_robot1_head_joint'], [0.0, -0.2], 1
            )
            self.detected_objects = 'upper'
            self.call_trigger_sync(self.start_cli, 'start_detection')
            time.sleep(10.0)
            self.call_trigger_sync(self.stop_cli,  'stop_detection')

            # ----- Lower shelf -----
            self.get_logger().info('[Inventory] Move head ↓')
            self.traj.send_trajectory(
                ['deepmind_robot1_head_base_joint','deepmind_robot1_head_joint'], [0.0, 0.7], 1
            )
            self.detected_objects = 'lower'
            self.call_trigger_sync(self.start_cli, 'start_detection')
            time.sleep(10.0)
            self.call_trigger_sync(self.stop_cli,  'stop_detection')

            # Compute and report
            total = self.sum_counts(self.upper_count, self.lower_count)
            self.get_logger().info(f'[Inventory] total={total}')
            self.check_inventory(total)

            self.inventory_done.publish(Bool(data=True))
            self.get_logger().info('[Inventory] Done.')

        except Exception as e:
            self.get_logger().error(f'[Inventory] Exception: {e}')
            # If you prefer to allow retrigger after an exception, leave self.done as False.
            # Otherwise set self.done = True here too.
        finally:
            # Stay “busy” only while running; release afterwards
            self.busy = False

    # ---------- utils ----------
    def sum_counts(self, upper: Dict[str,int], lower: Dict[str,int]) -> Dict[str,int]:
        out = dict(upper)
        for k, v in lower.items():
            out[k] = out.get(k, 0) + v
        return out

    def check_inventory(self, total: Dict[str,int]) -> None:

        if self.aoi_choice in ['red', 'blue']:
            expected, allowed = {'banana': 6, 'pizza': 4}, ['banana', 'pizza']
        else:
            expected, allowed = {'cell phone': 8, 'laptop': 4}, ['cell phone', 'laptop']

        missing = {k: expected[k] - total.get(k, 0) for k in expected if total.get(k, 0) < expected[k]}
        misplaced = {k: v for k, v in total.items() if k not in allowed}
        self.get_logger().info(f'Missing:{missing or "none"}  Misplaced:{misplaced or "none"}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InventoryChecking()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
