import rospy
from std_msgs.msg import Float64MultiArray, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion

import time
import math


V = 0
W = 0

X = 0
Y = 0
Theta = 0

constLinearSpeed = 0.72   #0.36m/s
constAngularSpeed = 2.50  #1.648rad/s

def callbackSub(cmd):
    global V, W
    V = cmd.linear.x
    W = cmd.angular.z

def computeOdometry(V, W, X, Y, Theta, deltaT):
    #print(deltaT)
    if (W != 0.0):
        if W > 0:
            Theta = Theta + constAngularSpeed*deltaT
        elif W < 0:
            Theta = Theta - constAngularSpeed*deltaT
    elif (V != 0.0):
        if V > 0:
            X=X+constLinearSpeed*deltaT*math.cos(Theta);
            Y=Y+constLinearSpeed*deltaT*math.sin(Theta);
        elif V < 0:
            X=X-constLinearSpeed*deltaT*math.cos(Theta);
            Y=Y-constLinearSpeed*deltaT*math.sin(Theta);
    return (X, Y, Theta)


#----------------------------------------------------


def doOdometry():
    global V, W, X, Y, Theta
    odomPub = rospy.Publisher('/odom', Odometry, queue_size=1)
    #ball_pub = rospy.Publisher('ball_position', PoseStamped, queue_size=1)
    rospy.init_node('doOdometry', anonymous=True)
    rate = rospy.Rate(50) # 50hz
    cmdVelSub = rospy.Subscriber('/cmd_vel',Twist,callbackSub)
    t_ini = time.time()
    while not rospy.is_shutdown():
        odomPosition = Odometry()
        t_fin = time.time()
        X, Y, Theta = computeOdometry(V, W, X, Y, Theta, t_fin-t_ini)
        t_ini = time.time()
        q = quaternion_from_euler(0, 0, Theta)
        odomPosition.pose.pose.position.x = X
        odomPosition.pose.pose.position.y = Y
        odomPosition.pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
        odomPosition.header.frame_id = "odom"
        odomPosition.child_frame_id = "base_footprint"
        odomPub.publish(odomPosition)
        rate.sleep()

if __name__ == '__main__':
    try:
        doOdometry()
    except rospy.ROSInterruptException:
        pass


