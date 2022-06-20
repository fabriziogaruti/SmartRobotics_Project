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
import os
import math

import time

from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink


def get_xyz_base(out):
    idx1 = out.index("sensor_laser")
    out = out[idx1:]
    idx2 = out.index("rotation")
    out = out[:idx2]
    # print(out, "\n")

    x_str = out.splitlines()[3]
    idx = x_str.index("x:") + 3
    x_base = float(x_str[idx:])

    y_str = out.splitlines()[4]
    idx = y_str.index("y:") + 3
    y_base = float(y_str[idx:])

    z_str = out.splitlines()[5]
    idx = z_str.index("z:") + 3
    z_base = float(z_str[idx:])

    return x_base, y_base, z_base

def ik(target_position):
    arm_chain = Chain(name='left_arm', links=[
    OriginLink(),
    URDFLink(
      name="arm_base_joint",
      origin_translation=[0, 0, 0.15],
      origin_orientation=[0.0, 0.0, 0.0],
      rotation=[0, 0, 1],
    ),
    URDFLink(
      name="shoulder_joint",
      origin_translation=[-0.05166, 0.0, 0.20271],
      origin_orientation=[0, 0, 1.5708],
      rotation=[0, 1, 0],
      bounds=(-math.pi/2,math.pi/2)
    ),
    URDFLink(
      name="bottom_wrist_joint",
      origin_translation=[0.0, -0.05194, 0.269],
      origin_orientation=[0, 0, 0],
      rotation=[0, 1, 0],
      bounds=(-math.pi/4,math.pi/4)
    ),
    URDFLink(
      name="elbow_joint",
      origin_translation=[0.0, 0, 0.13522],
      origin_orientation=[0, 0, 0],
      rotation=[0, 0, 1],
    ),
    URDFLink(
      name="top_wrist_joint",
      origin_translation=[0.0, 0, 0.20994],
      origin_orientation=[0, 0, 0],
      rotation=[0, 1, 0],
    )
    
    ], active_links_mask=[False,True,True,True,True,True])
    # target_position = [ 0.5, -0.4, 0.20]
    # ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')

    # arm_chain.plot(arm_chain.inverse_kinematics([2, 2, 2]), ax)
    # matplotlib.pyplot.show()
    return arm_chain.inverse_kinematics(target_position)


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.stopped=False

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
            angular_index = (angular_index / 2) - 90
            self.get_logger().info('I heard range: "%f", angle : %d ' % (min, angular_index))

            if min < 3:
                 with open("file.txt", "w") as f:
                    f.write('5')  # perform file operations
                    print("Approaching the object. Slowing down...")

            if min < 1 and self.stopped:
                time.sleep(2)
                # calcolo componenti x,z
                y_read = min * math.sin(math.radians(angular_index))
                x_read = min * math.cos(math.radians(angular_index))
                print("x,y read:", x_read, y_read, "\n")

                try:
                    str = os.popen("ros2 topic echo tf_static 2>/dev/null | head -n 64")
                    out = str.read()
                except BrokenPipeError:
                    print('boh')
                        
                # print(out)

                x_base, y_base, z_base = get_xyz_base(out)
                print("x,y,z base:", x_base, y_base, z_base, "\n")

                pos_point = [x_base+x_read, y_base+y_read, z_base+0.5]
                print("Pos point x,y,z = ", pos_point, "\n")
                
                target = ik(pos_point)
                print(target)
                target = target[1:]

                str = os.popen(f"ros2 run arm_mover mover --ros-args -p angles_start:=\"[0., 0., 0., 0., 0.]\" -p angles_finish:=\"[{target[0]}, {target[1]}, {target[2]}, {target[3]}, {target[4]}]\"")
                               
                out = str.read()
                time.sleep(100)

                # exit(0)

            if min < 1:
                with open("file.txt", "w") as f:
                    f.write('4')  # perform file operations
                    print("Approached the object. Stopping")
                    self.stopped=True
            


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

    # str = os.popen("ros2 topic echo odom --once")
    # out = str.read()
    # print(out)

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
