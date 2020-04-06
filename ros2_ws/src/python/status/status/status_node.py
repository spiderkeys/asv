import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool

import datetime

import Jetson.GPIO as GPIO

class Talker(Node):

    def __init__(self):
        super().__init__('pinger_node')

        self.last_check_time    = datetime.datetime.now()
        self.sample_received    = False

        # Init GPIO
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        GPIO.setup(7, GPIO.OUT, initial=GPIO.LOW)
        GPIO.output(7, GPIO.LOW)

        # Create subscriber
        self.sub = self.create_subscription( Bool, 'enabled', self.listener_callback, 1 )

        # Setup timer
        timer_period = 1.0
        self.timer = self.create_timer( timer_period, self.timer_callback )

    def set_state( self, state ):
        if state:
            print( "Enabled!" )
            GPIO.output(7, GPIO.HIGH)
        else:
            print( "Disabled!" )
            GPIO.output(7, GPIO.LOW)

    def listener_callback(self, msg):
        self.sample_received = True
        self.set_state( msg.data )

    def timer_callback(self):
        # Check to see if we've received any status updates in the last 3 seconds, otherwise set to false
        elapsed_ms = ( datetime.datetime.now() - self.last_check_time ).total_seconds() * 1000
        if( elapsed_ms > 3000 ):
            self.last_check_time = datetime.datetime.now()
            if self.sample_received is False:
                self.set_state( False )
        
            self.sample_received = False


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