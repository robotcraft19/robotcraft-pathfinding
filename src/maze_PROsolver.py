#!/usr/bin/env python
import rospy

class ProSolver:
    def __init__(self):
        rospy.init_node('maze_pro_solver', anonymous=True)

        # Setup publishers

        # Setup subscribers


    def run():
        rate = rospy.Rate(10) # 10hz

        while not rospy.is_shutdown():
            # Calculate command
            # Do other stuff

            rate.sleep()




if __name__ == '__main__':
    try:
        controller = ProSolver()
        controller.run()

    except rospy.ROSInterruptException:
        pass