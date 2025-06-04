#!/usr/bin/env python3
# ble_scanner/ble_node.py

'''
To run: 
make sure to be in /example_ws

Re-build the code:
colcon build
source install/local_setup.bash
(source ~/.bashrc)

For node:
ros2 run ble_scanner ble_node

For topic:
ros2 topic echo /ble_distance
'''

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from bleak import BleakScanner
import asyncio
import time

TARGET_NAME = "GuideBeacon"
TX_POWER = -59  # Adjust based on calibration
ALPHA = 0.2

def estimate_distance(rssi, tx_power=TX_POWER, n=2.0):
    """Estimate distance based on RSSI using log-distance path loss model."""
    return 10 ** ((tx_power - rssi) / (10 * n)) - 0.1

class BLEScannerNode(Node):
    def __init__(self):
        super().__init__('ble_scanner_node')
        self.last_rssi = None
        self.publisher_ = self.create_publisher(Float32, 'ble_distance', 10)
        self.get_logger().info("BLE Scanner node initialized")
        self.loop = asyncio.get_event_loop()
        self.loop.create_task(self.run_scanner())

    async def run_scanner(self):
        def detection_callback(device, advertisement_data):
            self.get_logger().info(f"Found device: {device.name} - RSSI: {device.rssi}")
            if device.name == TARGET_NAME:
                rssi = advertisement_data.rssi
                if self.last_rssi is not None:
                    rssi = ALPHA * rssi + (1 - ALPHA) * self.last_rssi
                self.last_rssi = rssi

                distance = estimate_distance(rssi)
                timestamp = time.strftime('%H:%M:%S')
                self.get_logger().info(f"[{timestamp}] RSSI: {rssi:.2f}, Distance: {distance:.2f} meters")

                msg = Float32()
                msg.data = distance
                self.publisher_.publish(msg)

        scanner = BleakScanner(detection_callback)
        await scanner.start()
        self.get_logger().info("BLE scanning started")
        try:
            while rclpy.ok():
                await asyncio.sleep(0.1)
        finally:
            await scanner.stop()

def main(args=None):
    rclpy.init(args=args)
    node = BLEScannerNode()
    try:
        node.loop.run_forever()
    except KeyboardInterrupt:
        pass
    finally:
        node.loop.run_until_complete(node.loop.shutdown_asyncgens())
        rclpy.shutdown()

if __name__ == '__main__':
    main()
