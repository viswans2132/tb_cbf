 #!/usr/bin/env python
 # license removed for brevity
import rospy
import pkg_resources
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped, Point
from std_msgs.msg import Header, Int8
from sensor_msgs.msg import LaserScan
from tb_cbf.msg import UgvConstraintMsg, UgvParamsMsg, UgvPosVelMsg
from tf.transformations import euler_from_quaternion, quaternion_matrix, quaternion_from_euler
import time
import numpy as np
import sys
from tb_cbf.ugv_lib import Ugv
from visualization_msgs.msg import Marker, MarkerArray

def dist(x_):
    return np.linalg.norm(x_)

def sq_dist(x_, y_):
    return np.sum(np.square(x_) / np.square(y_))


class LocalMinima:
    def __init__(self, name):
        self.name = name

        self.ugv = Ugv(name)
        self.rate = rospy.Rate(30)

        self.desPos = np.array([5.0, 0.0])
        self.clearance = 0.3
        self.timeStart = rospy.Time.now()

        self.ugvOdomSub = rospy.Subscriber('/{}/odom'.format(self.ugv.name), Odometry, self.odom_cb)
        self.ugvPvRefSub = rospy.Subscriber('/{}/ref'.format(self.ugv.name), UgvPosVelMsg, self.ref_pv_cb)
        self.ugvPsRefSub = rospy.Subscriber('/{}/reference'.format(self.ugv.name), PoseStamped, self.ref_ps_cb)
        self.ugvScanSub = rospy.Subscriber('/{}/scan'.format(self.ugv.name), LaserScan, self.scan_cb)

        self.ugvConsPub = rospy.Publisher('/{}/cons'.format(self.ugv.name), UgvConstraintMsg, queue_size=10)
        self.downsamplePub = rospy.Publisher('/{}/dspc'.format(self.ugv.name), MarkerArray, queue_size=10)
        

        self.goalPub = rospy.Publisher('{}/goal'.format(self.ugv.name), PoseStamped, queue_size=10)


        self.ugvConsMsg = Twist()


    def loop(self):
        odomReceived = self.ugv.generateControlInputs(self.cmdArray)
        self.rate.sleep()

        if rospy.get_time() - self.timer > 0.5:
            self.ugv.landFlag = True
            print('{}: Odometry not received'.format(self.name))


############## This is part for you: Until line 330 #####################
############## This is part for you: Until line 330 #####################
############## This is part for you: Until line 330 #####################
############## This is part for you: Until line 330 #####################
############## This is part for you: Until line 330 #####################
############## This is part for you: Until line 330 #####################
############## This is part for you: Until line 330 #####################

    def scan_cb(self, msg):
        """
        LaserScan callback: updates self.desPos if the goal is obstructed.
        """
        # Convert scan to Cartesian coordinates
        self.timeStart = rospy.Time.now()
        pos, ranges, angles = self.laser_to_cartesian(msg)


        # # Check if goal corridor is free
        # if self.is_goal_free(pos):
        #     self.publish_goal_pose(self.desPos)
        #     print("Goal OK")
        #     return


        # detect corners on downsampled scan (returns pos_sub etc.)
        corners, pos_sub, angles_sub, ranges_sub, step = self.detect_corners(pos, angles, ranges, msg.angle_increment)
        # print(corners.size)

        if corners.size == 0:
            # no corners found -> keep original goal (or apply other fallback)
            self.publish_goal_pose(self.curPos)
            return


        # Choose a setpoint inside the first valid gap
        setpoint = self.choose_setpoint(pos_sub, corners, angles_sub, ranges_sub)

        # Update the class goal and publish
        if setpoint is not None:
            self.desPos = setpoint
        self.publish_goal_pose(self.desPos)
        # print((rospy.Time.now() - self.timeStart))
        print("Start:", self.timeStart.to_nsec())
        print("Now:  ", rospy.Time.now().to_nsec())

    def laser_to_cartesian(self, msg):
        """Return pos (2,N), ranges (N,), angles (N,) at full resolution."""
        ranges = np.array(msg.ranges, dtype=np.float32)
        ranges = np.where(np.isinf(ranges), 10.0, ranges)
        ranges = np.nan_to_num(ranges, nan=0.0)
        n = len(ranges)

        angles = np.linspace(msg.angle_min, msg.angle_max, n, endpoint=False)

        # --- ROLL so that 0 rad (forward) is at the center ---
        center_idx = np.argmin(np.abs(angles))  # index closest to 0 rad
        ranges = np.roll(ranges, -center_idx)
        angles = np.roll(angles, -center_idx)

        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        pos = np.stack((x, y), axis=0)
        return pos, ranges, angles


    def is_goal_free(self, pos):
        """Check rectangular corridor along self.desPos for obstacles."""
        goal_vec = self.desPos
        goal_dist = np.linalg.norm(goal_vec)
        if goal_dist < 1e-6:
            return True

        goal_dir = goal_vec / goal_dist
        perp_dir = np.array([-goal_dir[1], goal_dir[0]])

        point_vecs = pos.T
        proj_along_goal = point_vecs @ goal_dir
        proj_perp = np.abs(point_vecs @ perp_dir)
        inside_rect = (proj_along_goal >= 0) & (proj_along_goal <= goal_dist) & (proj_perp <= self.clearance)



        return not np.any(inside_rect)



    def detect_corners(self, pos, angles, ranges, angle_increment):
        """
        Downsample and detect corners.
        Returns: corners (indices on pos_sub), pos_sub, angles_sub, ranges_sub, step
        """
        # downsample ~ every 10 degrees
        step = max(1, int((10.0 * np.pi / 180.0) / angle_increment))
        pos_sub = pos[:, ::step]
        angles_sub = angles[::step]
        ranges_sub = ranges[::step]
        frame_id = self.ugv.name +'/base_scan'
        self.publish_downsampled_points(pos_sub, frame_id)

        



        # corner detection based on jumps between consecutive downsampled points
        diff = np.diff(pos_sub, axis=1)               # shape (2, M-1)
        dist = np.linalg.norm(diff, axis=0)           # distances between consecutive downsampled points
        d_expected = ranges_sub[:-1] * np.tan(angle_increment * step)
        min_dist = np.maximum(2 * d_expected, 1.0)
        raw_corners = np.where(dist > min_dist)[0]    # indices on 0..M-2

        if raw_corners.size > 0:
            # pick the closer of the two points (i or i+1)
            closer_is_left = ranges_sub[raw_corners] < ranges_sub[raw_corners + 1]
            corners = np.where(closer_is_left, raw_corners, raw_corners + 1)


            # sort corners by closeness to forward (0 rad)
            corner_angles = angles_sub[corners]
            sort_idx = np.argsort(np.abs(corner_angles))
            corners = corners[sort_idx]
        else:
            corners = np.array([], dtype=int)

        return corners, pos_sub, angles_sub, ranges_sub, step

    def choose_setpoint(self, pos_sub, corners, angles_sub, ranges_sub):
        """
        Choose a setpoint 0.5 m away from the corner toward the closest point
        on the free side. Uses angular sector checks instead of full circle.
        """
        if corners.size == 0:
            return None

        for i in corners:
            corner = pos_sub[:, i].reshape(2, 1)
            corner_angle = angles_sub[i]

            # # sector-based selection
            # if corner_angle >= 0:  # right side corner
            #     right_mask = (angles_sub >= corner_angle) & (angles_sub <= np.pi)
            #     left_mask  = (angles_sub <= 0) & (angles_sub >= -np.pi)
            # else:  # left side corner
            #     left_mask  = (angles_sub <= corner_angle) & (angles_sub >= -np.pi)
            #     right_mask = (angles_sub >= 0) & (angles_sub <= np.pi)
            right_mask = (angles_sub >= corner_angle) & (angles_sub <= np.pi)
            left_mask  = (angles_sub <= corner_angle) & (angles_sub >= -np.pi)

            right_mask[i] = False
            left_mask[i] = False

            left_points = pos_sub[:, left_mask]
            print('-------- Left Points -------------')
            print(left_points)
            print('-------- Left Points -------------')
            left_distances = np.linalg.norm(left_points - corner, axis=0) if left_points.shape[1] > 0 else np.array([])
            # print('-------- Left Distances -------------')
            # print(left_distances)
            # print('-------- Left Distances -------------')
            left_ok = left_distances.size == 0 or np.all(left_distances >= 1.0)

            right_points = pos_sub[:, right_mask]
            print('-------- Right Points -------------')
            print(right_points)
            print('-------- Right Points -------------')
            right_distances = np.linalg.norm(right_points - corner, axis=0) if right_points.shape[1] > 0 else np.array([])
            # print('-------- Right Distances -------------')
            # print(right_distances)
            # print('-------- Right Distances -------------')
            right_ok = right_distances.size == 0 or np.all(right_distances >= 1.0)

            print(corner)
            print([left_ok, right_ok])
            input()

       
            if left_ok or right_ok:
                # pick the side that is free
                if left_ok:
                    gap_points, gap_distances, gap_side = left_points, left_distances, 'left'
                else:
                    gap_points, gap_distances, gap_side = right_points, right_distances, 'right'

                if gap_points.shape[1] > 0:
                    closest_idx = int(np.argmin(gap_distances))
                    direction = gap_points[:, closest_idx] - corner.flatten()
                    norm = np.linalg.norm(direction)
                    if norm < 1e-6:
                        continue
                    direction /= norm
                    print([gap_points[:, closest_idx], direction])
                    return corner.flatten() + 0.5 * direction

                # fallback: no points on that side -> perpendicular
                v = corner.flatten()
                if np.linalg.norm(v) > 1e-6:
                    if gap_side == 'left':
                        direction = np.array([-v[1], v[0]])
                    else:
                        direction = np.array([v[1], -v[0]])
                    direction /= np.linalg.norm(direction)
                    return corner.flatten() + 0.5 * direction
                else:
                    return np.array([0.0, 0.5])
            return None



    def check_side(self, pos_sub, corner, side, i):
        """Check if a side (left or right) of the corner is free (≥1 m)."""
        if side == 'left':
            if i > 0:
                points = pos_sub[:, :i]
                distances = np.linalg.norm(points - corner, axis=0)
                ok = np.all(distances >= 1.0)
            else:
                points = np.zeros((2,0))
                distances = np.array([])
                ok = True
        else:  # right
            if i < pos_sub.shape[1]-1:
                points = pos_sub[:, i+1:]
                distances = np.linalg.norm(points - corner, axis=0)
                ok = np.all(distances >= 1.0)
            else:
                points = np.zeros((2,0))
                distances = np.array([])
                ok = True
        return ok, points, distances


    def publish_goal_pose(self, goal_point):
        """
        Publish the goal as PoseStamped with orientation toward the goal.
        """
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = self.ugv.name + '/base_link'  # or 'map' if in global frame

        # Position
        pose_msg.pose.position.x = float(goal_point[0])
        pose_msg.pose.position.y = float(goal_point[1])
        pose_msg.pose.position.z = 0.0

        # Orientation (yaw toward the goal)
        yaw = np.arctan2(goal_point[1], goal_point[0])
        q = quaternion_from_euler(0, 0, yaw)  # roll, pitch, yaw
        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]

        self.goalPub.publish(pose_msg)


    def publish_downsampled_points(self, pos, frame_id='/base_scan', ns='downsampled_points'):
        """
        Publish downsampled laser points as visualization markers.

        Parameters
        ----------
        pos : np.array, shape (2, N)
            Cartesian coordinates of the points.
        frame_id : str
            ROS frame for the points.
        ns : str
            Namespace for markers.
        """
        marker_array = MarkerArray()
        for i, (x, y) in enumerate(pos.T):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = ns
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.05  # sphere radius
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)

        self.downsamplePub.publish(marker_array)

############## This is part for you: Until line 340 #####################
############## This is part for you: Until line 340 #####################
############## This is part for you: Until line 340 #####################
############## This is part for you: Until line 340 #####################



        


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
        rospy.init_node('local_minima', anonymous=True)
        name = "demo_turtle1"
        dc = LocalMinima(name)
        rospy.spin()

     except rospy.ROSInterruptException:
        pass
