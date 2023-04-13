import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64MultiArray, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf_conversions

import tf2_ros
#---------------------------------------------------


def callbackSub(msg):
	br = tf2_ros.TransformBroadcaster()
	t = TransformStamped()
	t.header.stamp = rospy.Time.now()
	t.header.frame_id = msg.header.frame_id
	t.child_frame_id = msg.child_frame_id
	t.transform.translation = msg.pose.pose.position
	t.transform.rotation = msg.pose.pose.orientation

	br.sendTransform(t)

#----------------------------------------------------


def exctractor():
    #drone_pub = rospy.Publisher('drone_position', PoseStamped, queue_size=1)
    #ball_pub = rospy.Publisher('ball_position', PoseStamped, queue_size=1)
    rospy.init_node('exctractor', anonymous=True)
    rate = rospy.Rate(50) # 5hz
    OdomSub = rospy.Subscriber('/odom',Odometry,callbackSub)
    while not rospy.is_shutdown():
        #drone_pub.publish(Pose_drone)
        #ball_pub.publish(Pose_ball)
        rate.sleep()

if __name__ == '__main__':
    try:
        exctractor()
    except rospy.ROSInterruptException:
        pass