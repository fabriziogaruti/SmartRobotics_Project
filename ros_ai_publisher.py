import rospy
from geometry_msgs.msg import Twist

def publisher():
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('cmd_vel_array_publisher')
    rate = rospy.Rate(1) #1Hz
    cmd_vel_array = [0.01,0.02,0.03,0.04,0.05]
    vel = Twist()
    while not rospy.is_shutdown():
        for i in range(len(cmd_vel_array)):
            vel.linear.x = cmd_vel_array[i]
            print("Publishing velocity" + str(cmd_vel_array[i]))
            pub.publish(vel)
            rate.sleep()
if __name__ == '__main__':
    publisher()