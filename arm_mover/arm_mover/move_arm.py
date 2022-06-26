#! /usr/bin/env python3

import sys
import rclpy
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
# ros2 action list -t
# ros2 action info /joint_trajectory_controller/follow_joint_trajectory -t
# ros2 interface show control_msgs/action/FollowJointTrajectory


class SteeringActionClient(Node):

    def __init__(self):
        super().__init__('arm_steer_actionclient')
        self._action_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('angles_start', None),
                ('angles_finish', None),
                ('seconds', 1)
            ]
        )


    def send_goal(self):
        goal_msg = FollowJointTrajectory.Goal()

        angles_start = self.get_parameter('angles_start').value
        angles_finish = self.get_parameter('angles_finish').value
        seconds = float(self.get_parameter('seconds').value)

        angles_start = [float(x) for x in angles_start]
        angles_finish = [float(x) for x in angles_finish]
        self.get_logger().info(f'Starting Angles: {angles_start}, Finish Angles: {angles_finish}')

        # Fill in data for trajectory
        joint_names = ["arm_base_joint", "shoulder_joint", "bottom_wrist_joint", "elbow_joint", "top_wrist_joint", "plier_right_joint", "plier_left_joint"]

        points = []
        point1 = JointTrajectoryPoint()
        point1.positions = angles_start

        point2 = JointTrajectoryPoint()
        point2.time_from_start = Duration(seconds=seconds, nanoseconds=0).to_msg()
        point2.positions = angles_finish

        points.append(point1)
        points.append(point2)

        goal_msg.goal_time_tolerance = Duration(seconds=seconds, nanoseconds=0).to_msg()
        goal_msg.trajectory.joint_names = joint_names
        goal_msg.trajectory.points = points

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: '+str(result))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        #self.get_logger().info('Received feedback:'+str(feedback))


def main(args=None):
    
    rclpy.init(args=args)

    action_client = SteeringActionClient()
    future = action_client.send_goal()

    rclpy.spin(action_client)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # action_client.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()