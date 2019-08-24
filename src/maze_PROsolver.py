#!/usr/bin/env python
import rospy
from map_loader import MapLoader
from path_finder import PathFinder
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from math import atan2
from math import pi

#path_raw = [(-1, 1), (-1, 2), (-1, 3), (-1, 4), (-1, 5), (0, 6), (1, 6), (2, 7), (3, 8), (4, 9), (4, 10), (4, 11), (4, 12), (4, 13), (4, 14), (4, 15), (4, 16), (4, 17), (4, 18), (4, 19), (3, 20), (2, 20), (1, 20), (0, 21), (0, 22), (0, 23), (1, 24), (2, 24), (3, 24), (4, 24), (5, 24), (6, 24), (7, 24), (8, 23), (8, 22), (8, 21), (9, 20), (10, 20), (11, 20), (12, 19), (13, 18), (13, 17), (13, 16), (14, 15), (15, 15), (16, 14), (16, 13), (16, 12), (15, 11), (14, 10), (13, 9), (13, 8), (13, 7), #(12, 6), (12, 5), (12, 4), (13, 3), (14, 2), (15, 2), (16, 2), (17, 2), (18, 2), (19, 2), (20, 2), (21, 3), (21, 4), (22, 5), (23, 5)]

class Pose:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

class Cell:
    resolution = 0.2
    def __init__(self, row, column):
        self.row = row
        self.column = column

    def pose(self):
        return Pose(self.column * Cell.resolution, -self.row * Cell.resolution, 0)

class ProSolver:
    def __init__(self):
        rospy.init_node('maze_pro_solver', anonymous=True)

        # Try to load parameters from launch file, otherwise set to None
        try:
            positions = rospy.get_param('~position')
            startX, startY =  positions['startX'], positions['startY']
            targetX, targetY =  positions['targetX'], positions['targetY']
            start = (startX, startY)
            target = (targetX, targetY)
            print(start, target)
        except:
            start, target = None, None

        # Load maze matrix
        self.map_loader = MapLoader(start, target) # do not crop if target outside of maze
        self.map_matrix = self.map_loader.loadMap()
        Cell.resolution = self.map_loader.occupancy_grid.info.resolution

        # Calculate path
        self.path_finder = PathFinder(self.map_matrix)
        raw_path = self.path_finder.calculate_path()
        self.path = [Cell(r-self.path_finder.robot.row, c- self.path_finder.robot.column) for r,c in raw_path] # move rows to correct starting position
        self.goal = self.path[0].pose()
        self.path_index = 0
        self.pose = Pose(0, 0, 0)

        # Setup publishers
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 5)

        # Setup subscribers
        sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)

    def odom_callback(self, msg):
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y

        rot_q = msg.pose.pose.orientation
        (_, _, self.pose.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    def next_pose(self):
        try:
            self.path_index += 1
            self.goal = self.path[self.path_index].pose()
        except IndexError:
            rospy.logwarn("REACHED END OF PATH!")

    def run(self):
        rate = rospy.Rate(10) # 10hz
        speed = Twist()

        while not rospy.is_shutdown():
            # Calculate command
            # Do other stuff

            inc_x = self.goal.x - self.pose.x
            inc_y = self.goal.y - self.pose.y

            angle_to_goal = atan2(inc_y, inc_x)

            if (inc_x**2 + inc_y**2)**0.5 < 0.05:
                speed.linear.x = 0.0
                speed.angular.z = 0.0
                self.next_pose()

            elif abs(angle_to_goal - self.pose.theta) > 0.02:
                speed.linear.x = 0.0
                if (self.pose.theta < angle_to_goal):
                    if (self.pose.theta < -0.2 and angle_to_goal > 0):
                        if (abs(self.pose.theta > (pi/2)) and abs(angle_to_goal > (pi/2))):
                            speed.angular.z = 0.3
                        else:
                            speed.angular.z = -0.3
                    else:
                        speed.angular.z = 0.3
                elif (self.pose.theta > angle_to_goal):
                    if (angle_to_goal < -0.2 and self.pose.theta > 0):
                        if (abs(self.pose.theta > (pi/2)) or abs(angle_to_goal > (pi/2))):
                            speed.angular.z = 0.3
                        else:
                            speed.angular.z = -0.3
                    else:
                        speed.angular.z = -0.3
            else:
                speed.linear.x = 0.08
                speed.angular.z = 0.0

            self.cmd_vel_pub.publish(speed)

            rate.sleep()


if __name__ == '__main__':
    try:
        controller = ProSolver()
        controller.run()

    except rospy.ROSInterruptException:
        pass
