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

        self.desPos = np.array([0.0, 0.0])

        self.kPos = np.array([-2.0, -2.0])


        self.hz = 1.0
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
        self.omegaD = 3
        
        self.kRad = 0.6
        self.omegaC = 3.0

        self.kHeight = 1.0
        self.kScaleA = self.kHeight/(self.kRad*self.kRad)
        # print(self.kScaleA)
        self.omegaA = 3.0
        self.omegaB = 5.0


        self.odomStatus = False
        self.constraintsReceived = False

        self.A = np.zeros((2,))
        self.b = 0.0
        self.P = np.eye(2)
        self.u = cp.Variable(2)


        self.filterFlag = False
        self.followFlag = True
        self.returnFlag = False
        self.stopFlag = False


    def setMode(self, data):
        self.filterFlag = True
        if data == 0:
            self.followFlag = True
            print('filter: ON {}'.format(self.name))
        elif data == 1:
            self.returnFlag = True
        elif data == 2:
            print('Stopping UGV: {}'.format(self.name))
            self.stopFlag = True

        else:
            print('Invalid mode for UGV: {}'.format(self.name))



    def setOdom(self, position, quat, velocity):
        self.pos[0] = position[0]
        self.pos[1] = position[1]
        self.pos[2] = position[2] + 0.06
        self.quat[0] = quat[0]
        self.quat[1] = quat[1]
        self.quat[2] = quat[2]
        self.quat[3] = quat[3]


        self.yaw = euler_from_quaternion(self.quat)[2]

        R_inv = quaternion_matrix(self.quat)[:-1, :-1]
        # self.R = np.linalg.inv(R_inv)
        self.R = np.array([[np.cos(self.yaw), np.sin(self.yaw)], [-np.sin(self.yaw), np.cos(self.yaw)]])
        self.vel = R_inv.T.dot(np.array([velocity[0], velocity[1], velocity[2]]))


        # self.vel[0] = velocity[0]
        # self.vel[1] = velocity[1]
        # self.vel[2] = velocity[2]
        self.ang_vel[2] = velocity[3]

        if self.odomStatus == False:
            self.odomStatus = True
            self.desPos[0] = self.pos[0]
            self.desPos[1] = self.pos[1]
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
            self.desPos = np.array([pos[0], pos[1]])
            self.desVel = np.array([vel[0], vel[1]])

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



    def filterValues(self, u_):
        # print([self.A.shape, self.u.shape])
        # if np.linalg.norm(u_) > 1.0:
        #     u_ = u_*1.0/np.linalg.norm(u_)
        # u_ = np.maximum(-np.array([0.3, 0.3, 0.2]), np.minimum(np.array([0.3, 0.3, 0.5]), u_))
        if self.constraintsReceived:
            try:
                constraints = [self.A@self.u >= self.b]
                prob = cp.Problem(cp.Minimize(cp.quad_form(self.u-u_, self.P)), constraints)
                try:
                    result = prob.solve()
                    desVel = self.u.value
                except cvxpy.error.SolveError:
                    print('SolveError for {}'.format(self.name))
                    desVel = np.array([0,0.0])

            except ValueError:
                print("Constraint matrices have incompatible dimensions {}:{}".format(self.A.shape, self.b.shape))
                self.landFlag = True
                desVel = np.array([0,0])

        else:
            desVel = np.array([0.0, 0.0])
            desVel[0] = u_[0]
            desVel[1] = u_[1]
            # print('Constraints Not received')

        # desVel = u_

        # if self.name == "dcf5":
        #     print("{:.3f}, {:.3f}".format(desVel[0], desVel[1]))
        #     print("{:.3f}, {:.3f}, {:.3f}".format(u_[0], u_[1], u_[2]))
        #     pass
        try:
            desVel = np.maximum(-np.array([0.3, 0.3]), np.minimum(np.array([0.3, 0.3]), desVel))
        except TypeError:
            desVel = np.array([0.0, 0.0])

        return desVel


    def generateControlInputs(self, velArray):
        uPitch = 0.0
        uRoll = 0.0
        uThrust = 0.0
        uYaw = 0.0
        if self.odomStatus:
            if self.stopFlag:
                velArray[0] = 0.0
                velArray[1] = 0.0
            else:
                if self.filterFlag:
                    posOff = self.pos[:2] + self.off*self.R.T[:,0]
                    desPosOff = self.desPos[:2] + self.off*self.R.T[:,0]
                    errPos = posOff - desPosOff 
                    if np.linalg.norm(errPos) < 0.02:
                        desVel = np.zeros(2)

                    else:
                        desVel = self.kPos * errPos 
                    if np.linalg.norm(desVel) > 0.23:
                        desVel = 0.23*desVel/np.linalg.norm(desVel)

                    # print('Error: {:.3f} : {:.3f}, {:.3f}'.format(errPos[0], errPos[1], self.yaw))
                    # print('Desired Velocity: {:.3f} : {:.3f}, {:.3f}'.format(desVel[0], desVel[1], self.yaw))
                    desVel = self.filterValues(desVel)


                    RlInv = np.array([[np.cos(self.yaw), np.sin(self.yaw)], [-np.sin(self.yaw)/self.off, np.cos(self.yaw)/self.off]])


                    cmdVel = RlInv.dot(desVel)
                    # if np.linalg.norm(cmdVel) > 0.3:
                    #     cmdVel = 0.3*cmdVel/np.linalg.norm(cmdVel)
                    cmdVel = np.maximum(-np.array([0.1, 0.4]), np.minimum(np.array([0.1, 0.4]), cmdVel))

                    velArray[0] = cmdVel[0]
                    velArray[1] = cmdVel[1]

        return self.odomStatus