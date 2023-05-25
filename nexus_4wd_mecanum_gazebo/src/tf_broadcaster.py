import rospy
from std_msgs.msg import Float64MultiArray, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Point, Quaternion
from sensor_msgs.msg import LaserScan
import tf_conversions

import tf2_ros
#---------------------------------------------------

laserData = LaserScan()

def callbackSub(msg):
	br = tf2_ros.TransformBroadcaster()
	t = TransformStamped()
	now = rospy.get_rostime()
	t.header.stamp = rospy.Time(now.secs, max(0,now.nsecs))#-50000000))#rospy.Time.now()#rospy.Time.from_sec(now.secs-0.5)
	#print(t.header.stamp)
	t.header.frame_id = msg.header.frame_id
	t.child_frame_id = msg.child_frame_id
	t.transform.translation = msg.pose.pose.position
	t.transform.rotation = msg.pose.pose.orientation
	br.sendTransform(t)
	
	#STATIC BROADCASTER 1
	stat1 = tf2_ros.TransformBroadcaster()
	t1 = TransformStamped()
	now = rospy.get_rostime()
	t1.header.stamp = rospy.Time(now.secs, max(0,now.nsecs))#-50000000))#rospy.Time.now()#rospy.Time.from_sec(now.secs-0.5)
	t1.header.frame_id = "base_footprint"
	t1.child_frame_id = "base_link"
	t1.transform.translation = Point(0, 0, 0.05)
	t1.transform.rotation = Quaternion(0, 0, 0, 1)
	stat1.sendTransform(t1)
	
	#STATIC BROADCASTER 2
	stat2 = tf2_ros.TransformBroadcaster()
	t2 = TransformStamped()
	now = rospy.get_rostime()
	t2.header.stamp = rospy.Time(now.secs, max(0,now.nsecs))#-50000000))#rospy.Time.now()#rospy.Time.from_sec(now.secs-0.5)
	t2.header.frame_id = "base_link"
	t2.child_frame_id = "laser_frame"
	t2.transform.translation = Point(0, 0, 0.13)
	t2.transform.rotation = Quaternion(0, 0, 0, 1)
	stat2.sendTransform(t2)


def callbackLaser(data):
    global laserData
    laserData = data
    laserData.header.stamp = rospy.Time.now()
    print("Raw: ", data.header.stamp, "real: ", laserData.header.stamp)


#----------------------------------------------------


def exctractor():
    #drone_pub = rospy.Publisher('drone_position', PoseStamped, queue_size=1)
    #ball_pub = rospy.Publisher('ball_position', PoseStamped, queue_size=1)
    rospy.init_node('exctractor', anonymous=True)
    rate = rospy.Rate(50) # 5hz
    OdomSub = rospy.Subscriber('/odom',Odometry,callbackSub)
    laserSub = rospy.Subscriber('/scan',LaserScan,callbackLaser)
    while not rospy.is_shutdown():
        #drone_pub.publish(Pose_drone)
        #ball_pub.publish(Pose_ball)
        rate.sleep()

if __name__ == '__main__':
    try:
        exctractor()
    except rospy.ROSInterruptException:
        pass