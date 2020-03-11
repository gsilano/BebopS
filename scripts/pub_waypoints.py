#!/usr/bin/env python
# license removed for brevity
import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectory
from mav_msgs import msgMultiDofJointTrajectoryFromPositionYaw

def talker():
    pub = rospy.Publisher('/bebop/command/waypoint', MultiDOFJointTrajectory, queue_size=10)
    rospy.init_node('way_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msg = MultiDOFJointTrajectory()
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass