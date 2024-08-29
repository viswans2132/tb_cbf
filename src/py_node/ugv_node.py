 #!/usr/bin/env python
 # license removed for brevity
import rospy
import pkg_resources
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Header, Int8
from tb_cbf.msg import ConstraintMsg, UgvParamsMsg
import time
import numpy as np
import sys
# from cf_cbf.drone_lib import Drone
from tb_cbf.ugv_lib import Ugv

def dist(x_):
    return np.linalg.norm(x_)

def sq_dist(x_, y_):
    return np.sum(np.square(x_) / np.square(y_))


class UgvController:
    def __init__(self, name):
        self.name = name

        self.ugv = Ugv(name)
        if self.name == 'dcf5':
            self.ugv.KintV = np.array([-0.02, -0.02, -0.4])
        self.rate = rospy.Rate(30)

        # self.ugvOdomSub = rospy.Subscriber('/vicon/{}/{}/odom'.format(self.ugv.name, self.ugv.name), Odometry, self.odom_cb)
        self.ugvOdomSub = rospy.Subscriber('/odom'.format(self.ugv.name, self.ugv.name), Odometry, self.odom_cb)
        self.ugvConsSub = rospy.Subscriber('/{}/cons'.format(self.ugv.name), ConstraintMsg, self.cons_cb)
        # self.ugvCmdPub = rospy.Publisher('/{}/cmd_vel'.format(self.ugv.name), Twist, queue_size=10)
        self.ugvCmdPub = rospy.Publisher('/cmd_vel'.format(self.ugv.name), Twist, queue_size=10)
        self.ugvParamPub = rospy.Publisher('/{}/param'.format(self.ugv.name), UgvParamsMsg, queue_size=10)

        self.cmdVelMsg = Twist()
        self.cmdArray = np.array([0,0,0,0.0])

        time.sleep(1)
        print('Node {}: Awake'.format(self.name))

        paramMsg = UgvParamsMsg()
        paramMsg.kRad = self.ugv.kRad
        paramMsg.omegaC = self.ugv.omegaC
        paramMsg.kScaleD = self.ugv.kScaleD
        paramMsg.kRate = self.ugv.kRate
        paramMsg.kOffset = self.ugv.kOffset
        paramMsg.omegaD = self.ugv.omegaD
        paramMsg.kHeight = self.ugv.kHeight
        paramMsg.kScaleA = self.ugv.kScaleA
        paramMsg.omegaA = self.ugv.omegaA
        self.ugvParamPub.publish(paramMsg)
        self.rate.sleep()

        self.timer = rospy.get_time()

        while not rospy.is_shutdown():
            self.loop()


    def loop(self):
    	# h1 = 1 - self.ugv.pos[0]
    	# dh1dt = 
        odomReceived = self.ugv.generateControlInputs(self.cmdArray)
        if rospy.get_time() - self.timer > 0.2:
            self.ugv.landFlag = True
        if odomReceived:
            self.cmdVelMsg.linear.x = self.cmdArray[0]
            self.cmdVelMsg.angular.z = self.cmdArray[1]
            self.ugvCmdPub.publish(self.cmdVelMsg)
            print('Publishing {:.3f}: {:.3f}'.format(self.cmdArray[0], self.cmdArray[1]))
            self.rate.sleep()

    def setMode(self, msg):
        self.ugv.setMode(msg.data)

    def cons_cb(self, msg):
        matrix = np.array(msg.constraints).reshape((-1,4))
        # print('Matrix: {}'.format(matrix))
        self.ugv.updateConstraintMatrices(matrix[:,:3], matrix[:,3])

    def land_cb(self, data):
        self.ugv.landFlag = True
        print('Safety Landing: Active')

    def follow_cb(self, data):
        self.ugv.followFlag = True
        print('Trajectory: Active')

    def start_cb(self, data):
        self.ugv.startFlag = True
        print('Take off: Active')

    def ref_cb(self, msg):
        # print(msg.position)
        try:
            pose = np.array([msg.position[0], msg.position[1], msg.position[2], msg.yaw])
            vel = np.array([msg.velocity[0], msg.velocity[1], msg.velocity[2], msg.yawVelocity])
            self.ugv.setRef(pose, vel)
        except IndexError:
            print('Ref msg empty: {}'.format(msg.position))

    def odom_cb(self, msg):
        position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        quat = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        velocity = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z, msg.twist.twist.angular.z])
        self.ugv.setOdom(position, quat, velocity)
        self.timer = rospy.get_time()



if __name__ == '__main__':
     try:
        rospy.init_node('turtlebot_controller', anonymous=True)
        ugv_name = 'ugv'
        dc = UgvController(ugv_name)
     except rospy.ROSInterruptException:
        pass
