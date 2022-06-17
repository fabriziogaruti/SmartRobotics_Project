# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        #print("Inside")
        #self.get_logger().info('I heard: "%s"' % msg.ranges)
        min = float("inf")
        i=0
        angular_index = 0
        for range in msg.ranges:
            if range < min:
                min = range
                angular_index = i
            i+=1
        if min != float("inf"):
                self.get_logger().info('I heard range: "%f", angle : %d ' % (min, angular_index))

class Tf_reader(Node):

    def __init__(self):
        super().__init__('tf_reader')
        self.subscription = self.create_subscription(
            TFMessage,
            '/tf_static',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        #print("Inside")
        self.get_logger().info('I heard: "%s"' % msg)



def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()
    #tf = Tf_reader()
    #rclpy.spin_once(tf)
    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
