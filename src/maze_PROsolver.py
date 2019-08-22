#!/usr/bin/env python
import rospy
import cv2
import numpy as np

class ProSolver:
    def __init__(self):
        rospy.init_node('maze_pro_solver', anonymous=True)

        self.map_matrix = self.loadMap()

        # Setup publishers

        # Setup subscribers


    def run(self):
        rate = rospy.Rate(10) # 10hz

        while not rospy.is_shutdown():
            # Calculate command
            # Do other stuff

            rate.sleep()


    def loadMap(self):
        # Load image and crop
        img = cv2.imread("/home/nico/Desktop/map_backup.pgm", cv2.IMREAD_GRAYSCALE)
        img = self.autocrop(img)

        # Convert colors into 0 and 1
        dark_colors = np.where(img <= 220)
        img[dark_colors] = 0
        light_colors = np.where(img > 220)
        img[light_colors] = 1

        # Flip number so that 0 = free and 1 = occupied
        img = np.logical_not(img).astype(int)

        # Save image to desktop
        cv2.imwrite("/home/nico/Desktop/filtered_map.pgm", img*255)

        return img

    def autocrop(self, image, lower_threshold=100, upper_threshold=220):
        """Crops any edges within to threshold boundaries (used for crop gray/unkown area)

        Crops blank image to 1x1.

        Returns cropped image.

        """
        if len(image.shape) == 3:
            flatImage = np.max(image, 2)
        else:
            flatImage = image
        assert len(flatImage.shape) == 2

        rows = np.where((np.max(flatImage, 0) < lower_threshold) | (np.max(flatImage, 0) > upper_threshold))[0]
        if rows.size:
            cols = np.where((np.max(flatImage, 1) > upper_threshold) | (np.min(flatImage, 1) < lower_threshold))[0]
            image = image[cols[0]: cols[-1] + 1, rows[0]: rows[-1] + 1]
        else:
            image = image[:1, :1]

        return image




if __name__ == '__main__':
    try:
        controller = ProSolver()
        controller.run()

    except rospy.ROSInterruptException:
        pass
