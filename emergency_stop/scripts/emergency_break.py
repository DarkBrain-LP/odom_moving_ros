#!/usr/bin/env python3

import rospy
from emergency_stop.msg import EmergencyBreak

def emergency_publisher():
    rospy.init_node('emergency_publisher', anonymous=True)
    pub = rospy.Publisher('emergency_break', EmergencyBreak, queue_size=10)

    # Create an instance of the EmergencyBrake message
    emergency_msg = EmergencyBreak()
    emergency_msg.trigger_signal = 1  
    emergency_msg.reason = "Internal temperature is too high!" 

    rate = rospy.Rate(0.1)  # Publish message at 0.1 Hz

    while not rospy.is_shutdown():
        pub.publish(emergency_msg)
        # rospy.loginfo("Published emergency braking message: Trigger Signal - {}, Reason - {}"
        #               .format(emergency_msg.trigger_signal, emergency_msg.reason))
        rate.sleep()

if __name__ == '__main__':
    try:
        emergency_publisher()
    except rospy.ROSInterruptException:
        pass
