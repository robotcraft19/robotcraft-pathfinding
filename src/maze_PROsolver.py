#!/usr/bin/env python
import rospy
import time
from map_loader import MapLoader
from path_finder import PathFinder
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from math import atan2, pi
import itertools

class Pose:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

class Cell:
    resolution = None
    start = None

    def __init__(self, row, column):
        self.row = row
        self.column = column
        self.rel_row = self.row - Cell.start.row
        self.rel_column = self.column - Cell.start.column

    def pose(self):
        # Check if cell has any walls as neighbours
        neighbors_matrix = ProSolver.matrix[self.row-1:self.row+2, self.column-1:self.column+2]
        diff_x = 0
        diff_y = 0

        if 1 in neighbors_matrix: # check if any walls at all
            if 1 in neighbors_matrix[0, :]:
                diff_y -= Cell.resolution/2 # move downwards if any wall in top row
            if 1 in neighbors_matrix[2, :]:
                diff_y += Cell.resolution/2 # move upwards if any wall in bottom row
            if 1 in neighbors_matrix[:, 0]:
                diff_x += Cell.resolution/2 # move to the right if any wall in left column
            if 1 in neighbors_matrix[:, 2]:
                diff_x -= Cell.resolution/2 # move to the left if any wall in right column
            rospy.logwarn("Path passing close to wall, using diff matrix: [%s, %s]", diff_x, diff_y)

        # Set default Pose to center of cell instead of top-left corner and add calculated diffs
        return Pose((self.rel_column * Cell.resolution) + Cell.resolution/2 + diff_x, \
                (-self.rel_row * Cell.resolution) - Cell.resolution/2+diff_y, 0)

class ProSolver:
    matrix = None
    def __init__(self):
        rospy.init_node('maze_pro_solver', anonymous=True)

        # Try to load parameters from launch file, otherwise set to None
        try:
            positions = rospy.get_param('~position')
            startX, startY =  positions['startX'], positions['startY']
            targetX, targetY =  positions['targetX'], positions['targetY']
            start = (startX, startY)
            target = (targetX, targetY)
        except:
            start, target = None, None

        # Load maze matrix
        self.map_loader = MapLoader(start, target) # do not crop if target outside of maze
        self.map_matrix = self.map_loader.loadMap()
        ProSolver.matrix = self.map_matrix
        Cell.resolution = self.map_loader.occupancy_grid.info.resolution

        # Calculate path
        self.path_finder = PathFinder(self.map_matrix)
        raw_path = self.path_finder.calculate_path()
        Cell.start = self.path_finder.start
        self.path = [Cell(r, c) for r,c in raw_path]
        self.goal = self.path[0].pose()
        self.path_index = 0
        self.pose = Pose(0, 0, 0)

        # Setup publishers
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)

        # Setup subscribers
        #odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        odom_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.odom_callback)

    def odom_callback(self, msg):
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y

        rot_q = msg.pose.pose.orientation
        (_, _, self.pose.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    def next_pose(self):
        try:
            self.path_index += 1
            self.goal = self.path[self.path_index].pose()
            rospy.loginfo("Moving to next pose: [%s, %s]", self.goal.x, self.goal.y)
        except IndexError:
            rospy.logwarn("REACHED END OF PATH!")

    def run(self):
        rate = rospy.Rate(10) # 10hz
        speed = Twist()

        while not rospy.is_shutdown():
            # Calculate command
            ang_speed = 0.4

            inc_x = self.goal.x - self.pose.x
            inc_y = self.goal.y - self.pose.y

            angle_to_goal = atan2(inc_y, inc_x)
            real_angle = self.pose.theta

            # Normalize angle_diff
            if angle_to_goal < 0:
                angle_to_goal += 2*pi
                real_angle += 2*pi

            if real_angle < 0:
                real_angle += 2*pi
                angle_to_goal += 2*pi


            angle_diff = angle_to_goal - real_angle

            if (inc_x**2 + inc_y**2)**0.5 < 0.05:
                speed.linear.x = 0.0
                speed.angular.z = 0.0
                self.next_pose()

            elif abs(angle_diff) > 0.2: # increase tolerance?
                speed.linear.x = 0.0
                if angle_diff < 0:
                    speed.angular.z = -ang_speed
                else:
                    speed.angular.z = +ang_speed
            else:
                speed.linear.x = 0.08
                speed.angular.z = 0.0

            self.cmd_vel_pub.publish(speed)

            rate.sleep()


if __name__ == '__main__':
    try:
        controller = ProSolver()
        time.sleep(5);
        controller.run()

    except rospy.ROSInterruptException:
        pass
