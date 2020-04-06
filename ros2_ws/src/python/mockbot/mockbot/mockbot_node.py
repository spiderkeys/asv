import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

from sensor_msgs.msg import Range
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import UInt64
from sensor_msgs.msg import NavSatFix

import random

class Talker(Node):

    def __init__(self):
        super().__init__('mockbot_node')

        qos_best_effort = QoSProfile(history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST, depth=1, reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
        qos_klr_tl = QoSProfile(history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST, depth=1, reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE, durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)

        self.start_time = random.uniform(10,100)
        self.time_elapsed = 0

        # Create publishers
        self.p_time     = self.create_publisher( UInt64, 'time_unix_ms', qos_best_effort )
        self.p_gps      = self.create_publisher( NavSatFix, 'gps', qos_klr_tl )
        self.p_attitude = self.create_publisher( Vector3Stamped, 'attitude', qos_klr_tl )
        self.p_speed    = self.create_publisher( Vector3Stamped, 'speed', qos_klr_tl )
        self.p_depth    = self.create_publisher( Range, 'depth', qos_klr_tl )

        # Setup timer
        timer_period = 1.0
        self.timer = self.create_timer( timer_period, self.timer_callback )

    def timer_callback(self):
        print( "Publishing..." )
        self.time_elapsed = self.time_elapsed + 1
        self.pub_time()
        self.pub_gps()
        self.pub_attitude()
        self.pub_speed()
        self.pub_depth()

    def pub_gps(self):
        msg = NavSatFix()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.status.status = 0
        msg.latitude        = random.uniform( 32.000, 32.000200 )
        msg.longitude       = random.uniform( 72.000, 72.000200 )
        self.p_gps.publish(msg)

    def pub_time(self):
        msg = UInt64()
        msg.data = self.time_elapsed
        self.p_time.publish(msg)

    def pub_attitude(self):
        msg = Vector3Stamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.vector.x        = random.uniform(0.0,0.5)
        msg.vector.y        = random.uniform(0.0,0.5)
        msg.vector.z        = random.uniform(0.0,6.28)
        self.p_attitude.publish(msg)

    def pub_speed(self):
        msg = Vector3Stamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.vector.z        = random.uniform(0.0,2.0)
        self.p_speed.publish(msg)

    def pub_depth(self):
        msg = Range()
        msg.radiation_type  = 0
        msg.min_range       = 0.5
        msg.max_range       = 30.0
        msg.range           = random.uniform(1.0,5.0)
        msg.field_of_view   = 100.0
        msg.header.stamp    = self.get_clock().now().to_msg()
        self.p_depth.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = Talker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()