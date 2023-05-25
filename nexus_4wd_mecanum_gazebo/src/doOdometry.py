import rospy
from std_msgs.msg import Float64MultiArray, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion

import adafruit_bno055
import board
import smbus

import time
import math
import statistics

'''
#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47


def MPU_Init():
	#write to sample rate register
	bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
	
	#Write to power management register
	bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
	
	#Write to Configuration register
	bus.write_byte_data(Device_Address, CONFIG, 0)
	
	#Write to Gyro configuration register
	bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
	
	#Write to interrupt enable register
	bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
	#Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)
    
        #concatenate higher and lower value
        value = ((high << 8) | low)
        
        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value


bus = smbus.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address

MPU_Init()

print (" Reading Data of Gyroscope and Accelerometer")
'''

V = 0

X = 0
Y = 0
Theta = 0
histAcc = []

constLinearSpeed = 0.39#0.72   #0.36m/s
constAngularSpeed = 1.822#2.50  #1.648rad/s


###########IMU
i2c = board.I2C()
sensor = adafruit_bno055.BNO055_I2C(i2c)

def callbackSub(msg):
    global Vreal, Wreal
    Wreal = msg.angular.z
    if (Wreal < 8) and (Wreal > -8):
        Wreal = 0
    Vreal = msg.linear.x
    if ((Vreal < 20) and (Vreal > -20)) or (Wreal != 0):
        Vreal = 0
    Vreal = 1.39*0.98*3*Vreal/1000  #1.065*2*3*V/1000
    Wreal = 0.92*1.1*1.05*4*Wreal/100   #1.1*2.1*4*W/100
    

def computeOdometry(Vreal, Wreal, X, Y, Theta, deltaT):
    #print(deltaT)
    ############""IMU acquisition
    try:
        yaw, roll, pitch = sensor.euler
        ThetaIMU = -1*yaw*2*math.pi/360           #reading from gyroscope
    except:
        print("Something went wrong during yaw aquisition")
        ThetaIMU = Theta
    #############end IMU
    '''acc_x = read_raw_data(ACCEL_XOUT_H)
    acc_y = read_raw_data(ACCEL_YOUT_H)
    Ax = round(9.81*acc_x/16384.0,1)
    Ay = round(round(9.81*acc_y/16384.0,1) - 2.6,1)
    if ((Ay < 0.21) and (Ay > -0.31)):
        Ay = 0
    if Ay < 0:  # do MMA8451 measures higher backwards acceleration?
        Ay = Ay/1.3
    if len(histAcc)>4:
        oldValue = histAcc.pop(0)
    histAcc.append(Ay)
    accAvg = round(statistics.mean(histAcc), 2)
    #print("Ax = ", Ax, "Ay = ", Ay)
    if accAvg == 0: #(len(histAcc) == 1) or ((histAcc[-1] == 0.0) and (histAcc[-2] == 0.0)):
        V = 0.00000          #Reset speed to 0 if accelerometer detects no movement for the 2 last samples
    else:
        if (accAvg > 0.5) or (accAvg < -0.5):     #Only update V is abs(accAvg is high enough
            V = round(V + accAvg*deltaT, 5)       #Else: we estimate that we are in a constant speed phase
    #V = round(V + accAvg*deltaT, 5)'''
    print("Vreal: ", Vreal, "Wreal: ", Wreal)
    ThetaODOM = round(Theta + Wreal*deltaT,3)     #Integrating  the previous fusionned Theta value
    print("theta IMU: ", ThetaIMU, "theta ODOM: ", ThetaODOM)
    Theta = (2*ThetaIMU + ThetaODOM)/3
    X=round(X+Vreal*deltaT*math.cos(Theta), 3)
    Y=round(Y+Vreal*deltaT*math.sin(Theta), 3)
    print("X position: ", X, "Theta: ", Theta)
    '''if (W != 0.0):
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
            Y=Y-constLinearSpeed*deltaT*math.sin(Theta);'''
    return (X, Y, Theta)


#----------------------------------------------------


def doOdometry():
    global Vreal, Wreal, X, Y, Theta, histAcc
    odomPub = rospy.Publisher('/odom', Odometry, queue_size=1)
    #ball_pub = rospy.Publisher('ball_position', PoseStamped, queue_size=1)
    rospy.init_node('doOdometry', anonymous=True)
    Vreal, Wreal = (0,0)
    rate = rospy.Rate(50) # 50hz
    cmdVelSub = rospy.Subscriber('/robotSpeed',Twist,callbackSub)
    t_ini = time.time()
    while not rospy.is_shutdown():
        odomPosition = Odometry()
        t_fin = time.time()
        X, Y, Theta = computeOdometry(Vreal, Wreal, X, Y, Theta, t_fin-t_ini)
        t_ini = time.time()
        q = quaternion_from_euler(0, 0, Theta)
        odomPosition.pose.pose.position.x = X
        odomPosition.pose.pose.position.y = Y
        odomPosition.pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
        odomPosition.twist.twist.linear.x = Vreal
        odomPosition.twist.twist.angular.z = Wreal
        odomPosition.header.frame_id = "odom"
        odomPosition.child_frame_id = "base_footprint"
        odomPub.publish(odomPosition)
        rate.sleep()

if __name__ == '__main__':
    try:
        doOdometry()
    except rospy.ROSInterruptException:
        pass


