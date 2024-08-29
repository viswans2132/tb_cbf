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



class UGV(object):
    # name = 'ugv'
    

    def __init__(self, name):
        self.name = name
        self.pos = np.array([0.0, 0, 0])
        self.quat = np.array([0.0, 0, 0, 1])

        self.vel = np.array([0, 0.0, 0])
        self.ang_vel = np.array([0.0, 0, 0])


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
        print([self.kScaleD, self.kRate])
        self.kOffset = 0.0
        self.omegaD = 0.7
        
        radius = 0.4
        self.kHeight = 1.0
        self.kScaleA = self.kHeight/(radius*radius)
        print(self.kScaleA)
        self.omegaA = 3.0

        self.odomStatus = False





    def odom_cb(self, data):
        self.pos[0] = float(data.pose.pose.position.x)
        self.pos[1] = float(data.pose.pose.position.y)
        self.pos[2] = float(data.pose.pose.position.z) + 0.08
        self.quat[0] = float(data.pose.pose.orientation.x)
        self.quat[1] = float(data.pose.pose.orientation.y)
        self.quat[2] = float(data.pose.pose.orientation.z)
        self.quat[3] = float(data.pose.pose.orientation.w)
        self.vel[0] = float(data.twist.twist.linear.x)
        self.vel[1] = float(data.twist.twist.linear.y)
        self.vel[2] = float(data.twist.twist.linear.z)
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

    def publishCmdVel(self, data):
        self.cmd_pub.publish(data)
    
    def odomStatus(self):
        return self.odomStatus