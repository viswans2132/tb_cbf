 #!/usr/bin/env python
 # license removed for brevity
import rospy
import pkg_resources
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Header, Int8
from tb_cbf.msg import UgvConstraintMsg, UgvParamsMsg
from gazebo_msgs.msg import ModelStates
import time
import numpy as np
import sys
# from cf_cbf.drone_lib import Drone
from tb_cbf.ugv_lib import Ugv
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import euler_from_quaternion, quaternion_matrix



def dist(x_):
    return np.linalg.norm(x_)

def sq_dist(x_, y_):
    return np.sum(np.square(x_) / np.square(y_))


class UgvController:
    def __init__(self, name):
        self.name = name

        self.ugv = Ugv(name)
        if self.name == 'tb1':
            self.ugv.KintV = np.array([-0.02, -0.02, -0.4])
        self.rate = rospy.Rate(30)

        # self.ugvOdomSub = rospy.Subscriber('/vicon/{}/{}/odom'.format(self.ugv.name, self.ugv.name), Odometry, self.odom_cb)
        self.ugvOdomSub = rospy.Subscriber('/odom'.format(self.ugv.name), Odometry, self.odom_cb)
        # self.cylOdomSub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.obs_cb)
        self.ugvConsSub = rospy.Subscriber('/cons'.format(self.ugv.name), UgvConstraintMsg, self.cons_cb)
        # self.ugvCmdPub = rospy.Publisher('/{}/cmd_vel'.format(self.ugv.name), Twist, queue_size=10)
        self.ugvCmdPub = rospy.Publisher('/cmd_vel'.format(self.ugv.name), Twist, queue_size=10)
        self.ugvParamPub = rospy.Publisher('/param'.format(self.ugv.name), UgvParamsMsg, queue_size=10)

        self.markerSub = rospy.Subscriber('/marker/ellipses', MarkerArray, self.marker_cb)

        self.cmdVelMsg = Twist()
        self.cmdArray = np.array([0,0,0,0.0])

        self.obsPos = np.array([1.0, 0.0])

        self.ellipses = np.array([])
        time.sleep(1)
        print('Node {}: Awake'.format(self.name))

        # paramMsg = UgvParamsMsg()
        # paramMsg.kRad = self.ugv.kRad
        # paramMsg.omegaC = self.ugv.omegaC
        # paramMsg.kScaleD = self.ugv.kScaleD
        # paramMsg.kRate = self.ugv.kRate
        # paramMsg.kOffset = self.ugv.kOffset
        # paramMsg.omegaD = self.ugv.omegaD
        # paramMsg.kHeight = self.ugv.kHeight
        # paramMsg.kScaleA = self.ugv.kScaleA
        # paramMsg.omegaA = self.ugv.omegaA
        # paramMsg.omegaB = self.ugv.omegaB
        # self.ugvParamPub.publish(paramMsg)
        # self.rate.sleep()

        self.timer = rospy.get_time()

        self.ugv.filterFlag = True

        while not rospy.is_shutdown():
            self.loop()


    def loop(self):
        self.genConsMatrix()
        # h1 = sq_dist(errCyl, np.array([1.0, 1.0])) - 0.36
        # dh1dx = 2*errCyl[0]
        # dh1dy = 2*errCyl[1]
        # dh1dt = 2*errCyl[0]*np.sin(self.ugv.yaw)*self.ugv.off*self.ugv.ang_vel[2] - 2*errCyl[1]*np.cos(self.ugv.yaw)*self.ugv.off*self.ugv.ang_vel[2]
        # dh1dt = 0.0
        # print('h: {:.3f}'.format(h1))
        # # print('h: {:.3f}'.format(h1))
        # A = np.array([dh1dx, dh1dy])
        # b = np.array([-0.1*h1 - dh1dt])
        # Ab_  = np.hstack((A,b)).flatten()
        # droneConsMsg = UgvConstraintMsg()
        # droneConsMsg.constraints = Ab_.tolist()
        # self.cons_cb(droneConsMsg)

        odomReceived = self.ugv.generateControlInputs(self.cmdArray)
        if rospy.get_time() - self.timer > 0.2:
            self.ugv.landFlag = True
        if odomReceived:
            self.ugv.desPos = np.array([3.0, 0.2])
            self.cmdVelMsg.linear.x = self.cmdArray[0]
            self.cmdVelMsg.angular.z = self.cmdArray[1]
            self.ugvCmdPub.publish(self.cmdVelMsg)
            # print('Publishing {:.3f}: {:.3f}'.format(self.cmdArray[0], self.cmdArray[1]))
            self.rate.sleep()

    def setMode(self, msg):
        self.ugv.setMode(msg.data)

    def cons_cb(self, msg):
        matrix = np.array(msg.constraints).reshape((-1,3))
        print('Matrix: {}'.format(matrix))
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

    def marker_cb(self, msg):
        markers = msg.markers
        self.ellipses = []
        no_obs = len(markers)
        if no_obs > 0:
            self.ellipses = np.zeros((no_obs, 5))
            for i, marker in enumerate(markers):
                self.ellipses[i,0] = marker.pose.position.x
                self.ellipses[i,1] = marker.pose.position.y
                self.ellipses[i,2] = (marker.scale.x+0.6)*(marker.scale.x+0.6)/4
                self.ellipses[i,3] = (marker.scale.y+0.6)*(marker.scale.y+0.6)/4

                q = [marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w]

                self.ellipses[i,4] = euler_from_quaternion(q)[2]
                self.ellipses[i,4] = 0.0


    def genConsMatrix(self):
        if self.ellipses.size > 0:
            # self.ellipses = self.ellipses[0,:].reshape((1,-1))
            self.cons = np.zeros((len(self.ellipses), 3))
            for i in range(len(self.ellipses)):
                errPos = self.ugv.pos[:2] -  self.ellipses[i,:2]
                x_term = errPos[0]
                y_term = errPos[1]
                print(f'ErrPos: {errPos[0]}')
                x_term = errPos[0]*np.cos(self.ellipses[i,4]) + errPos[1]*np.sin(self.ellipses[i,4])
                y_term = -errPos[0]*np.sin(self.ellipses[i,4]) + errPos[1]*np.cos(self.ellipses[i,4])
                h = (x_term*x_term/self.ellipses[i,2]) + (y_term*y_term/self.ellipses[i,3]) - 1.0
                print(f'H: {h}')
                dhdx = 2*(x_term*np.cos(self.ellipses[i,4])/self.ellipses[i,2] - y_term*np.sin(self.ellipses[i,4])/self.ellipses[i,3])
                dhdy = 2*(x_term*np.sin(self.ellipses[i,4])/self.ellipses[i,2] + y_term*np.cos(self.ellipses[i,4])/self.ellipses[i,3])
                # dhdx = x_term/self.ellipses[i,2]
                # dhdy = y_term/self.ellipses[i,3]

                self.cons[i, 0] = dhdx
                self.cons[i, 1] = dhdy
                self.cons[i, 2] = -2.1*h


            droneConsMsg = UgvConstraintMsg()
            droneConsMsg.constraints = self.cons.tolist()
            self.cons_cb(droneConsMsg)

        else:
            self.ugv.constraintsReceived = False












if __name__ == '__main__':
     try:
        rospy.init_node('turtlebot_controller', anonymous=True)
        ugv_name = 'ugv'
        dc = UgvController(ugv_name)
     except rospy.ROSInterruptException:
        pass
