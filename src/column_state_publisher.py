#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import UInt32
from std_msgs.msg import Int8
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

jointstate_pub = rospy.Publisher('jointState', JointState, queue_size=10)

def callback_position(data):
    js = JointState()
    ##js.header = Header()
    js.name = ['column']
    js.velocity = []
    js.effort = []
    js.position = [data.data/4000.00]

    jointstate_pub.publish(js)

def driver():
    rospy.init_node('column_state_publisher', anonymous=True)
    rospy.Subscriber("column/position", UInt32, callback_position)


    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        driver()
    except rospy.ROSInterruptException:
        pass