import rospy
from std_msgs.msg import String

def talker():
    rospy.init_node('talker', anonymous=True)
    pub0 = rospy.Publisher('/chatter0', String, queue_size=10)
    pub1 = rospy.Publisher('/chatter1', String, queue_size=10)
    pub2 = rospy.Publisher('/chatter2', String, queue_size=10)
    
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        print("publishing")

        pub0.publish("talker 0 lalalala %s" % rospy.get_time())
        pub1.publish("talker one kskskskks %s" % rospy.get_time())
        pub2.publish("talker 2222 sdav %s" % rospy.get_time())
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass