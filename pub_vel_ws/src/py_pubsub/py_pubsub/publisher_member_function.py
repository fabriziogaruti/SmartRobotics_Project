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
from geometry_msgs.msg import Twist

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.dict_vel = {
            0:{'linear_x': 0, 'angular_z':0},
            1:{'linear_x':0.2, 'angular_z':0},
            2:{'linear_x': 0, 'angular_z': 0.1},
            3:{'linear_x': 0, 'angular_z': -0.1},
            4:{'linear_x': 0, 'angular_z': 0},
            5:{'linear_x': -1, 'angular_z': -1}
        }

        self.slow_down = False

    def timer_callback(self):
        #msg = String()
        #msg.data = 'Hello World: %d' % self.i
        #self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % msg.data)
        #self.i += 1
        vel = Twist()
        '''for i in range(len(cmd_vel_array)):
            vel.linear.x = cmd_vel_array[i]
            print("Publishing velocity" + str(cmd_vel_array[i]))
            self.publisher_.publish(vel)'''

        with open("file.txt", "r") as f:
            value = f.read()  # perform file operations
            print("Read file", value)

        vel.linear.x = float(self.dict_vel[int(value)]['linear_x'])
        vel.angular.z = float(self.dict_vel[int(value)]['angular_z'])
        if value == 5:
            self.slow_down = True
        if self.slow_down:
            vel.linear.x = vel.linear.x / 2
        print("Publishing velocity" + str(vel.linear.x))
        self.publisher_.publish(vel)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
