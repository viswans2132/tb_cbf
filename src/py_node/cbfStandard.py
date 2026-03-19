#!/usr/bin/env python3
import rospy
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

# try to import cvxpy and fallback with a clear error
try:
    import cvxpy as cp
except Exception as e:
    rospy.logerr("cvxpy not available: {}".format(e))
    raise

class APF_CBF_Controller:
    def __init__(self):
        # state in global frame
        self.curPos = np.array([0.0, 0.0])
        self.desPos = np.array([2.0, 2.0])
        self.yaw = 0.0

        # parameters (tunable / consider making rosparams)
        self.k_att = 0.6     # nominal attractive gain
        self.k_rep_nom = 0.5 # small repulsive used only to create nominal velocity
        self.d0 = 1.0        # influence distance for scanning obstacles (for nominal)
        self.r_safe = 0.3    # safety radius (robot radius + margin)
        self.alpha = 2.0     # CBF class-K coefficient: dh/dt + alpha*h >= -s
        self.slack_penalty = 1000.0  # penalty on slack in QP
        self.vmax = 0.5
        self.vmin = -0.1     # allow small backward movement if needed
        self.wmax = 1.5

        # ROS
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb, queue_size=1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb, queue_size=1)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # small regularization in QP objective
        self.Q_reg = np.diag([1.0, 0.05])  # weight for (v, w) deviation (if needed)

    def odom_cb(self, msg: Odometry):
        self.curPos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def nominal_velocity_from_apf(self, scan_msg):
        """
        Produce a nominal (v_nom, w_nom) using APF (go-to-goal + light repulsion).
        This is only a nominal reference — CBF QP will modify it to ensure safety.
        Returns 2-vector (v_nom, w_nom) in robot frame.
        """
        # attractive (global)
        error = self.desPos - self.curPos
        v_att_global = self.k_att * error

        # small repulsive in global (just to bias nominal away from close obstacles)
        v_rep_global = np.zeros(2)

        angle = scan_msg.angle_min
        R = np.array([[math.cos(self.yaw), -math.sin(self.yaw)],
                      [math.sin(self.yaw),  math.cos(self.yaw)]])
        for r in scan_msg.ranges:
            if np.isfinite(r) and r < self.d0:
                obs_local = np.array([r * math.cos(angle), r * math.sin(angle)])
                obs_global = self.curPos + R.dot(obs_local)
                vec = obs_global - self.curPos
                dist = np.linalg.norm(vec)
                if dist > 1e-6:
                    dir_away = -vec / dist
                    # softened repulsive (smaller magnitude for nominal)
                    v_rep_global += self.k_rep_nom * (1.0/dist - 1.0/self.d0) * (1.0/(dist**2)) * dir_away
            angle += scan_msg.angle_increment

        v_total_global = v_att_global + v_rep_global

        # transform to robot frame
        v_total_robot = R.T.dot(v_total_global)
        # convert to (v, w) nominal: forward velocity as x component,
        # angular nominal from heading of v_total_robot
        v_nom = v_total_robot[0]
        heading = math.atan2(v_total_robot[1], v_total_robot[0]) if np.linalg.norm(v_total_robot) > 1e-6 else 0.0
        w_nom = heading  # simple proportional heading as nominal angular command

        # saturate nominal to limits
        v_nom = np.clip(v_nom, self.vmin, self.vmax)
        w_nom = np.clip(w_nom, -self.wmax, self.wmax)
        return np.array([v_nom, w_nom])

    def build_cbf_constraints(self, scan_msg):
        """
        For each LIDAR return (within influence), compute linear inequality
        a_i * v >= b_i  where b_i = -alpha*h - slack_i (but we'll implement slack globally)
        Returns lists a_list and b_list and optionally distances for debug.
        """
        a_list = []
        b_list = []
        # we only create constraints for obstacles closer than d0 + some margin
        angle = scan_msg.angle_min
        R = np.array([[math.cos(self.yaw), -math.sin(self.yaw)],
                      [math.sin(self.yaw),  math.cos(self.yaw)]])
        for r in scan_msg.ranges:
            if not np.isfinite(r):
                angle += scan_msg.angle_increment
                continue
            # only consider obstacles up to some larger horizon (d0 used here)
            if r <= 0.0 or r > (self.d0 + 0.5):
                angle += scan_msg.angle_increment
                continue

            # obstacle in robot frame
            obs_local = np.array([r * math.cos(angle), r * math.sin(angle)])
            # transform to global
            obs_global = self.curPos + R.dot(obs_local)
            vec = obs_global - self.curPos      # from robot to obstacle, global
            dist = np.linalg.norm(vec)
            if dist < 1e-6:
                angle += scan_msg.angle_increment
                continue

            # h = dist - r_safe
            h = dist - self.r_safe
            # compute a_i for the linear constraint in v:
            # dh/dt = (p_robot - p_obs)^T * v_robot / dist = -vec^T * (v * [cos,yaw;sin,yaw]) / dist
            # dh/dt + alpha*h >= -s  => - (vec·n)/dist * v + alpha*h >= -s
            n = np.array([math.cos(self.yaw), math.sin(self.yaw)])  # robot heading in global
            dot = np.dot(vec, n)   # vec·n
            a_i = - (dot / dist)   # coefficient multiplying v
            b_i = - self.alpha * h # RHS without slack (we will move to other side as a_i * v >= b)
            # store only if obstacle is closer than some threshold (h <= influence)
            if h <= self.d0:  # obstacle within influence radius
                a_list.append(a_i)
                b_list.append(b_i)
            angle += scan_msg.angle_increment

        return a_list, b_list

    def scan_cb(self, msg: LaserScan):
        # compute nominal control
        u_nom = self.nominal_velocity_from_apf(msg)  # [v_nom, w_nom]

        # build CBF linear inequalities a_i * v >= b_i
        a_list, b_list = self.build_cbf_constraints(msg)

        # Setup cvxpy QP: minimize ||u - u_nom||^2 + slack_penalty * s^2
        # variables: v, w, s (single slack for all constraints)
        v = cp.Variable()
        w = cp.Variable()
        s = cp.Variable(nonneg=True)

        u_var = cp.vstack([v, w])

        # objective: quadratic around u_nom
        u_nom_vec = np.array(u_nom).reshape((2, 1))
        # quadratic term: (u - u_nom)^T W (u - u_nom)
        W = np.diag([1.0, 0.2])  # weights on deviation of v and w (tweakable)
        # CVXPY: use quad_form
        obj = cp.quad_form(u_var - u_nom_vec, W) + self.slack_penalty * cp.square(s)

        constraints = []
        # control bounds
        constraints += [v <= self.vmax, v >= self.vmin, w <= self.wmax, w >= -self.wmax]

        # CBF constraints (linear in v, using a single slack s):
        # a_i * v >= b_i - s  <=> a_i * v + s >= b_i
        for (a_i, b_i) in zip(a_list, b_list):
            constraints += [a_i * v + s >= b_i]

        prob = cp.Problem(cp.Minimize(obj), constraints)
        try:
            prob.solve(solver=cp.OSQP, warm_start=True)
        except Exception:
            # fallback solver if OSQP not available
            prob.solve(solver=cp.SCS, warm_start=True)

        # if problem infeasible or solver failed, fallback to a safe stop
        if v.value is None or w.value is None:
            rospy.logwarn_throttle(5.0, "CBF QP failed - publishing stop")
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            return

        v_safe = float(np.clip(v.value, self.vmin, self.vmax))
        w_safe = float(np.clip(w.value, -self.wmax, self.wmax))

        # publish
        cmd = Twist()
        cmd.linear.x = v_safe
        cmd.angular.z = w_safe
        self.cmd_pub.publish(cmd)

def main():
    rospy.init_node('apf_cbf_controller')
    controller = APF_CBF_Controller()
    rospy.loginfo("APF+CBF controller started.")
    rospy.spin()

if __name__ == '__main__':
    main()
