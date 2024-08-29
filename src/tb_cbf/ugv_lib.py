 #!/usr/bin/env python
 # license removed for brevity
import rospy
import pkg_resources
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Header
import time
import numpy as np
import cvxpy as cp
from tf.transformations import euler_from_quaternion, quaternion_matrix



class Ugv(object):
    # name = 'ugv'
    

    def __init__(self, name):
        self.name = name
        self.pos = np.array([0.0, 0, 0])
        self.pos2 = np.array([0.0, 0])
        self.quat = np.array([0.0, 0, 0, 1])

        self.vel = np.array([0, 0.0, 0])
        self.ang_vel = np.array([0.0, 0, 0])

        self.off = 0.1

        self.desPos = np.array([2.0, 0.0])

        self.kPos = np.array([-2.0, -2.0])


        self.hz = 30.0
        self.dt = 1/self.hz
        self.control_input = np.array([0.0, 0.0, 0.0])
        self.cmdVel = Twist()
        self.ref = PoseStamped()

        self.rate = rospy.Rate(self.hz)

        # self.cmd_pub = rospy.Publisher('/{}/cmd_vel'.format(name), TwistStamped, queue_size=1)
        # self.ref_pub = rospy.Publisher('/{}/ref'.format(name), PoseStamped, queue_size=1)

        # self.odom_sub = rospy.Subscriber('/vicon/{}/{}/odom'.format(name, name), Odometry, self.odom_cb)
        # self.cmd_sub = rospy.Subscriber('/old_cmd_vel', TwistStamped, self.oldControl_cb)

        ezp = 0.5
        theta = np.deg2rad(30)
        d = ezp*np.tan(theta)

        self.kScaleD = np.exp(1)*ezp
        self.kRate = 1/(d*d)
        # print([self.kScaleD, self.kRate])
        self.kOffset = 0.0
        self.omegaD = 0.7
        
        self.kRad = 0.4
        self.omegaC = 3.0

        self.kHeight = 1.0
        self.kScaleA = self.kHeight/(self.kRad*self.kRad)
        # print(self.kScaleA)
        self.omegaA = 3.0


        self.odomStatus = False
        self.constraintsReceived = False

        self.A = np.zeros((2,))
        self.b = 0.0
        self.P = np.eye(2)
        self.u = cp.Variable(2)





    def setOdom(self, position, quat, velocity):
        self.pos[0] = position[0]
        self.pos[1] = position[1]
        self.pos[2] = position[2]
        self.quat[0] = quat[0]
        self.quat[1] = quat[1]
        self.quat[2] = quat[2]
        self.quat[3] = quat[3]


        self.yaw = euler_from_quaternion(self.quat)[2]

        R_inv = quaternion_matrix(self.quat)[:-1, :-1]
        # self.R = np.linalg.inv(R_inv)
        self.R = np.array([[np.cos(self.yaw), np.sin(self.yaw)], [-np.sin(self.yaw), np.cos(self.yaw)]])

        self.vel[0] = velocity[0]
        self.vel[1] = velocity[1]
        self.vel[2] = velocity[2]
        self.ang_vel[2] = velocity[3]

        if self.odomStatus == False:
            self.odomStatus = True
            print('Odometry Received: {}'.format(self.name))

        # if self.name=="demo_turtle1" and self.pos[0] < 0.6:
        #     print('{:.3f}'.format(self.pos[0]))

    def setStop(self, data):
        self.stop_flag = True

    def setStart(self, data):
        self.start_flag = True
    
    def setFollow(self, data):
        self.follow_flag = True


    def setRef(self, pos, vel):
        if self.followFlag:
            self.desPos = np.array([pos[0], pos[1], pos[2]])
            self.desVel = np.array([vel[1], vel[2], vel[3]])
            self.desYaw = pos[3]
            self.desYawVel = vel[3]

    def publishCmdVel(self, data):
        self.cmd_pub.publish(data)
    
    def odomStatus(self):
        return self.odomStatus


    def clearConstraintMatrices(self):
        self.A = np.zeros((3,))
        self.b = 0.0

    def updateConstraintMatrices(self, A_, b_):
        self.A = A_
        self.b = b_
        self.constraintsReceived = True



    def filterValues(self, err, u_):
        # print([self.A.shape, self.u.shape])
        # if np.linalg.norm(u_) > 1.0:
        #     u_ = u_*1.0/np.linalg.norm(u_)
        # u_ = np.maximum(-np.array([0.3, 0.3, 0.2]), np.minimum(np.array([0.3, 0.3, 0.5]), u_))
        if self.constraintsReceived:
            try:
                constraints = [self.A@self.u >= self.b]
                prob = cp.Problem(cp.Minimize(cp.quad_form(self.u-u_, self.P)), constraints)
                result = prob.solve()

                desVel = self.u.value
            except ValueError:
                print("Constraint matrices have incompatible dimensions {}:{}".format(self.A.shape, self.b.shape))
                self.landFlag = True
                desVel = np.array([0,0])

        else:
            desVel[0] = u_[0]
            desVel[1] = u_[1]

        # desVel = u_

        # if self.name == "dcf3":
        #     print("{:.3f}, {:.3f}, {:.3f}".format(desVel[0], desVel[1], desVel[2]))
        #     print("{:.3f}, {:.3f}, {:.3f}".format(u_[0], u_[1], u_[2]))
        #     pass
        desVel = np.maximum(-np.array([0.15, 0.15]), np.minimum(np.array([0.15, 0.15]), desVel))

        return desVel


    def generateControlInputs(self, velArray):
        uPitch = 0.0
        uRoll = 0.0
        uThrust = 0.0
        uYaw = 0.0
        if self.odomStatus:
            pos_off = self.pos[:2] + self.off*self.R.T[:,0]
            desVel = self.kPos * (pos_off - self.desPos)
            print('Desired Velocity: {:.3f} : {:.3f}'.format(desVel[0], desVel[1]))

            RlInv = np.array([[np.cos(self.yaw), np.sin(self.yaw)], [-np.sin(self.yaw)/self.off, np.cos(self.yaw)/self.off]])


            cmdVel = RlInv.dot(desVel)
            velArray[0] = cmdVel[0]
            velArray[1] = cmdVel[1]
            

            # errPos = self.pos - self.desPos
            # # print('{:.3f}, {:.3f}, {:.3f}'.format(errPos[0], errPos[1], errPos[2]))
            # if self.returnFlag and np.linalg.norm(errPos[:2]) < 0.5:
            #     self.errInt = self.errInt + errPos*self.dt
            #     self.errInt = np.maximum(-self.maxInt, np.minimum(self.maxInt, self.errInt))

            # self.desVel = self.Kpos * errPos + self.KintP * self.errInt

            # if self.filterFlag:
            #     self.desVel = self.filterValues(errPos, self.desVel)

            # velW = self.R.T.dot(self.vel)


            # derVel = ((velW - self.desVel) - self.errVel)/self.dt
            # self.errVel = velW - self.desVel

            # if self.startFlag:
            #     self.errVelInt = self.errVelInt + self.errVel*self.dt
            # self.errVelInt = np.maximum(-self.maxVelInt, np.minimum(self.maxVelInt, self.errVelInt))
            # # print(self.errVelInt)

            # des_a = self.Kvel * self.errVel + self.Kder * derVel + self.KintV * self.errVelInt
            # # des_a = self.Kvel * self.errVel
            # des_a = self.R.dot(des_a)
            # # print("Error: {0:.3f}: {1:.3f}: {2:.3f}: \n Acc: {3:.3f}: {4:.3f}: {5:.3f}".format(self.errVel[0], errPos[1], errPos[2], des_a[0], des_a[1], des_a[2]))
            # # print(des_a)
            # # print(des_a[3])
            # des_a = np.maximum(-self.maxAcc, np.minimum(self.maxAcc, des_a))
            # # print("{:.3f}: {:.3f}: {:.3f}".format(errPos[0], errPos[1], errPos[2]))

            # # yaw_diff = np.minimum(0.2, np.maximum(self.desYaw - self.yaw, -0.2))

        return self.odomStatus