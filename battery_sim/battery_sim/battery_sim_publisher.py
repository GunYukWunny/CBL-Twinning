import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
import random

class FakeBatteryPublisher(Node):
    def __init__(self):
        super().__init__('fake_battery_publisher')
        self.publisher_ = self.create_publisher(BatteryState, '/battery_state', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.battery_level = 1.0  # Start at 100%

    def timer_callback(self):
        msg = BatteryState()
        msg.voltage = 11.1
        msg.current = -0.5
        msg.charge = self.battery_level * 100.0
        msg.capacity = 100.0
        msg.percentage = self.battery_level
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.present = True
        self.publisher_.publish(msg)

        # Decrease battery level to simulate discharge
        self.battery_level -= 0.01
        if self.battery_level < 0.0:
            self.battery_level = 0.0

        self.get_logger().info(f'Publishing simulated battery: {msg.percentage*100:.1f}%')

def main(args=None):
    rclpy.init(args=args)
    node = FakeBatteryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
