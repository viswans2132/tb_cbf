#!/usr/bin/env python3
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

class APFController:
    def __init__(self):
        self.name = "demo_turtle1"
        # Current and desired positions in GLOBAL frame
        self.curPos = np.array([0.0, 0.0])  
        self.desPos = np.array([5.0, 0.0])  

        # Robot yaw (updated from odometry)
        self.yaw = 0.0  

        # Control parameters
        self.k_att = 0.5   # attractive gain
        self.k_rep = 0.1   # repulsive gain
        self.d0 = 0.3      # influence distance for obstacles
        self.vmax = 0.4    # max forward velocity
        self.wmax = 1.0    # max angular velocity
        self.goal_tol = 0.1  # stop if within 10cm of goal

        # ROS pubs/subs
        self.scan_sub = rospy.Subscriber(f'/{self.name}/scan', LaserScan, self.scan_cb)
        self.odom_sub = rospy.Subscriber(f'/{self.name}/odom', Odometry, self.odom_cb)
        self.cmd_pub = rospy.Publisher(f'/{self.name}/cmd_vel', Twist, queue_size=10)

    def odom_cb(self, msg: Odometry):
        """
        Update robot position and yaw from odometry.
        """
        # Position
        self.curPos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ])

        # Orientation (quaternion → yaw directly)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def scan_cb(self, msg: LaserScan):
        """
        Build APF-based velocity command:
        attractive velocity (to goal) + repulsive velocity (from obstacles).
        """

        # === Check if goal is reached ===
        if np.linalg.norm(self.desPos - self.curPos) < self.goal_tol:
            stop_cmd = Twist()
            self.cmd_pub.publish(stop_cmd)
            rospy.loginfo("Goal reached! Stopping robot.")
            return

        # === 1. Attractive velocity in GLOBAL frame ===
        error = self.desPos - self.curPos
        v_att = self.k_att * error

        # === 2. Repulsive velocity from obstacles ===
        v_rep = np.zeros(2)

        angle = msg.angle_min
        R = np.array([[np.cos(self.yaw), -np.sin(self.yaw)],
                      [np.sin(self.yaw),  np.cos(self.yaw)]])  # rotation robot→global

        for r in msg.ranges:
            if np.isfinite(r) and r < self.d0:
                # Obstacle in robot frame
                obs_local = np.array([r * np.cos(angle), r * np.sin(angle)])

                # Transform to global frame
                obs_global = self.curPos + R @ obs_local

                # Vector from robot to obstacle
                vec = obs_global - self.curPos
                dist = np.linalg.norm(vec)

                if dist > 1e-6:
                    # Direction away from obstacle
                    dir_away = -vec / dist

                    # Repulsive velocity contribution
                    v_rep += self.k_rep * (1.0/dist - 1.0/self.d0) * (1.0/(dist**2)) * dir_away

            angle += msg.angle_increment

        # === 3. Total velocity in GLOBAL frame ===
        v_total = v_att + v_rep

        # === 4. Transform to ROBOT frame for cmd_vel ===
        R_inv = R.T
        v_robot = R_inv @ v_total

        # === 5. Convert to Twist ===
        cmd = Twist()
        cmd.linear.x = np.clip(v_robot[0], -self.vmax, self.vmax)
        cmd.angular.z = np.clip(np.arctan2(v_robot[1], v_robot[0]), -self.wmax, self.wmax)

        self.cmd_pub.publish(cmd)

# ============================
# Main entry point
# ============================
def main():
    rospy.init_node('apf_controller')
    controller = APFController()
    rospy.loginfo("APF Controller node started.")
    rospy.spin()

if __name__ == '__main__':
    main()
