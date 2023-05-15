import rospy
from std_msgs.msg import Float64MultiArray, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion

import sys
import smbus2 as smbus#,smbus2
import time
import subprocess

V = 0
W = 0

# Slave Addresses
I2C_SLAVE_ADDRESS = 8 #0x0b ou 11
subprocess.run(['i2cdetect', '-y', '1'])
I2Cbus = smbus.SMBus(1)

def ConvertStringsToBytes(src):
  converted = []
  for b in src:
    converted.append(ord(b))
  return converted

def convertData(data):
    #print("converting data", data)
    toAvoid = chr(255)
    message = ""
    for i in data:
        message = message + chr(i)
    numbers = message.split("/")
    print(numbers)
    x = float(numbers[0])
    y = float(numbers[1])
    #theta = float(numbers[2].split(toAvoid)[0])
    a1 = numbers[2].rstrip('\x01')#split(chr(92))
    a1 = a1.rstrip('\x00')
    a1 = a1.rstrip('\x01')
    a1 = a1.rstrip('\x00')
    #a2=a1[0]
    #print(a2)
    #print(type(a1), a1)
    theta = float(a1)
    return (x, y, theta)

def runI2CRoutine(V, W, xOdom, yOdom, tOdom):
    global I2Cbus
    with smbus.SMBus(1) as I2Cbus:
        slaveAddress = I2C_SLAVE_ADDRESS
        time.sleep(0.1)
        #V = float(input("Enter linear: "))
        #V = 2.4
        #W = -3.6
        #W = float(input("Enter angular: "))
        if (V==0.0) and (W==0.0):
            V=99.0
            W=99.0
        cmd = str(round(V,2)) + "/" + str(round(W,2))
        BytesToSend = ConvertStringsToBytes(cmd)
        #print("Sent " + str(slaveAddress) + " the " + str(cmd) + " command.")
        #print(BytesToSend )
        try:
            I2Cbus.write_i2c_block_data(slaveAddress, 0x00, BytesToSend)
        except Exception as e:
            print("error at writing: ", e)
        #time.sleep(0.5)


        '''V = float(input("Enter linear: "))
        W = float(input("Enter angular: "))
        cmd = str(round(V,2)) + "/" + str(round(W,2))

        BytesToSend = ConvertStringsToBytes(cmd)
        print("Sent " + str(slaveAddress) + " the " + str(cmd) + " command.")
        print(BytesToSend )
        I2Cbus.write_i2c_block_data(slaveAddress, 0x00, BytesToSend)'''
        time.sleep(0.2)
        try:
            data=I2Cbus.read_i2c_block_data(slaveAddress,0x00,20)
            #print("recieve from slave")
            #print(type(data), data)
            #convertData(data)
            x, y, theta = convertData(data)
            theta = theta - 1.57    #reverting offset added by HW team
            #print(x, y, theta)
            return (x, y, theta)
        except Exception as e:
            print(e)
            #subprocess.run(['i2cdetect', '-y', '1'], stdout=subprocess.DEVNULL)
            print("remote i/o error")
            #time.sleep(1.5)
        #time.sleep(0.2)
    return (xOdom, yOdom, tOdom)
    

def callbackSub(cmd):
    global V, W
    V = cmd.linear.x
    W = cmd.angular.z

#----------------------------------------------------


def i2cBridge():
    global V, W
    odomPub = rospy.Publisher('/odom', Odometry, queue_size=1)
    #ball_pub = rospy.Publisher('ball_position', PoseStamped, queue_size=1)
    rospy.init_node('i2cBridge', anonymous=True)
    rate = rospy.Rate(5) # 5hz
    cmdVelSub = rospy.Subscriber('/cmd_vel',Twist,callbackSub)
    xOdom, yOdom, tOdom = (0, 0, 1.57)
    while not rospy.is_shutdown():
        odomPosition = Odometry()
        xOdom, yOdom, tOdom = runI2CRoutine(V, W, xOdom, yOdom, tOdom)        
        q = quaternion_from_euler(0, 0, tOdom)
        print(q)
        odomPosition.pose.pose.position.x = xOdom
        odomPosition.pose.pose.position.y = yOdom
        odomPosition.pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
        odomPosition.header.frame_id = "odom"
        odomPosition.child_frame_id = "base_footprint"
        odomPub.publish(odomPosition)
        #ball_pub.publish(Pose_ball)
        rate.sleep()

if __name__ == '__main__':
    try:
        i2cBridge()
    except rospy.ROSInterruptException:
        pass


