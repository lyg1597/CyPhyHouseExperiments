from geometry_msgs.msg import Point, Twist
import rospy


def talker():
    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('/chatter0', Twist, queue_size=10)
    
    
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        print("publishing")
        data = Twist()

        data.linear.x = 0.5
        data.linear.y = 0.5
        data.linear.z = 0.5
        pub.publish(data)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass