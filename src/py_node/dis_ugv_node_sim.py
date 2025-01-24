 #!/usr/bin/env python
 # license removed for brevity
import rospy
import pkg_resources
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Header, Int8
from tb_cbf.msg import UgvConstraintMsg, UgvParamsMsg, UgvPosVelMsg
from tf.transformations import euler_from_quaternion, quaternion_matrix
import time
import numpy as np
import sys
from tb_cbf.ugv_lib import Ugv

def dist(x_):
    return np.linalg.norm(x_)

def sq_dist(x_, y_):
    return np.sum(np.square(x_) / np.square(y_))


class UgvController:
    def __init__(self, name):
        self.name = name

        self.ugv = Ugv(name)
        ezp = 0.5
        theta = np.deg2rad(25)
        self.omegaD = 0.5
        d = ezp*np.tan(theta)
        self.ugv.kScaleD = np.exp(1)*ezp
        self.ugv.kRate = 1/(d*d)
        self.ugv.kRad = 0.6

        self.rate = rospy.Rate(30)

        self.ugvOdomSub = rospy.Subscriber('/{}/odom'.format(self.ugv.name), Odometry, self.odom_cb)
        self.ugvPvRefSub = rospy.Subscriber('/{}/ref'.format(self.ugv.name), UgvPosVelMsg, self.ref_pv_cb)
        self.ugvPsRefSub = rospy.Subscriber('/{}/reference'.format(self.ugv.name), PoseStamped, self.ref_ps_cb)
        self.ugvConsSub = rospy.Subscriber('/{}/cons'.format(self.ugv.name), UgvConstraintMsg, self.cons_cb)
        self.ugvModeSub = rospy.Subscriber('/{}/update_ugv_mode'.format(self.ugv.name), Int8, self.setMode)
        self.ugvCmdPub = rospy.Publisher('/{}/cmd_vel'.format(self.ugv.name), Twist, queue_size=10)
        self.ugvParamSub = rospy.Subscriber('/{}/params'.format(self.ugv.name), UgvParamsMsg, self.params_cb)
        self.ugvUpdateParamPub = rospy.Publisher('/{}/update_params'.format(self.ugv.name), UgvParamsMsg, queue_size=10)
        self.ugvModePub = rospy.Publisher('/{}/ugv_mode'.format(self.ugv.name), Int8, queue_size=10)

        self.cmdVelMsg = Twist()
        self.cmdArray = np.array([0,0,0,0.0])

        print('Node {}: Awake'.format(self.name))

        self.timer = rospy.get_time()

        while not rospy.is_shutdown():
            self.loop()


    def loop(self):
        odomReceived = self.ugv.generateControlInputs(self.cmdArray)
        if odomReceived:
            # print(self.cmdArray)
            self.cmdVelMsg.linear.x = self.cmdArray[0]
            self.cmdVelMsg.angular.z = self.cmdArray[1]
            self.ugvCmdPub.publish(self.cmdVelMsg)
        if not self.ugv.paramFlag:            
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
            paramMsg.omegaB = self.ugv.omegaB
            self.ugvUpdateParamPub.publish(paramMsg)
        self.rate.sleep()

        if rospy.get_time() - self.timer > 0.5:
            self.ugv.landFlag = True
            print('{}: Odometry not received'.format(self.name))

    def setMode(self, msg):
        self.ugv.setMode(msg.data)
        modeMsg = Int8()
        modeMsg.data = self.ugv.ugvMode
        self.ugvModePub.publish(modeMsg)

    def cons_cb(self, msg):
        matrix = np.array(msg.constraints).reshape((-1,3))
        # print('Matrix: {}'.format(matrix))
        self.ugv.updateConstraintMatrices(matrix[:,:2], matrix[:,2])

    def land_cb(self, data):
        self.ugv.landFlag = True
        print('Safety Landing: Active')

    def follow_cb(self, data):
        self.ugv.followFlag = True
        print('Trajectory: Active')

    def start_cb(self, data):
        self.ugv.startFlag = True
        print('Take off: Active')


    def params_cb(self, data):
        self.ugv.paramFlag = True


    def ref_pv_cb(self, msg):
        try:
            pose = np.array([msg.position[0], msg.position[1]])
            vel = np.array([msg.velocity[0], msg.velocity[1]])
            self.ugv.setRef(pose, vel)
        except IndexError:
            print('Ref msg empty: {}: {}'.format(msg.position, msg.velocity))

    def ref_ps_cb(self, msg):
        try:
            pose = np.array([msg.pose.position.x, msg.pose.position.y])
            vel = np.zeros(2)
            self.ugv.setRef(pose, vel)
        except IndexError:
            print('Ref msg empty: {}: {}'.format(msg.position, msg.velocity))

    def odom_cb(self, msg):
        quat = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        velocity = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z, msg.twist.twist.angular.z])
        self.ugv.setOdom(position, quat, velocity)
        self.timer = rospy.get_time()



if __name__ == '__main__':
     try:
        rospy.init_node('turtlebot_controller', anonymous=True)
        uav_name = rospy.get_param('~ugv_name')
        dc = UgvController(uav_name)
     except rospy.ROSInterruptException:
        pass
