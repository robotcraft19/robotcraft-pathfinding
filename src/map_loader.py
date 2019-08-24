#!/usr/bin/env python
import os
import rospy
import cv2
import numpy as np
from nav_msgs.srv import GetMap, GetMapRequest

class MapLoader:
    def __init__(self, crop_image=False):
        self.crop_image = crop_image
        self.occupancy_grid = self.request_occupancy_grid()


    def request_occupancy_grid(self):
        # Make request to map_loader service
        rospy.wait_for_service('static_map')
        try:
            get_map_service = rospy.ServiceProxy('static_map', GetMap)
            req = GetMapRequest()
            resp = get_map_service(req)
            rospy.loginfo("Successfully loaded occupancy grid from map_server")
            return resp.map
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s"%e)

    def loadMap(self):

        # Load image (alternatively use occupancy_grid data and reshape)
        scans_dir = os.path.join(os.path.expanduser("~"),"catkin_ws/src/robotcraft_maze/scans/")
        self.orig_img = cv2.imread(os.path.join(scans_dir, "map_backup.pgm"), cv2.IMREAD_GRAYSCALE)

        if self.crop_image == True:
            img = self.autocrop(self.orig_img)
        else:
            img = self.orig_img

        # Convert colors into 0 and 1
        dark_colors = np.where(img <= 220)
        img[dark_colors] = 0
        light_colors = np.where(img > 220)
        img[light_colors] = 1

        # Flip number so that 0 = free and 1 = occupied
        img = np.logical_not(img).astype(int)

        # Save filtered image to scans folder
        cv2.imwrite(os.path.join(scans_dir, "map_filtered.pgm"), img*255)

        self.place_robot(img)
        self.place_target(img)
        np.savetxt(os.path.join(os.path.expanduser("~"),
            'catkin_ws/src/robotcraft_maze/scans/map_matrix.txt'), img, delimiter='', fmt='%d')
        rospy.loginfo('Saved map matrix to text file...')

        return img

    def place_robot(self, img):
        # Place robot at origin of map
        origin_x = self.occupancy_grid.info.origin.position.x
        origin_y = self.occupancy_grid.info.origin.position.y
        resolution = self.occupancy_grid.info.resolution

        if self.crop_image == True:
            n_rows_removed_top = self.cropped_rows[0][1]-self.cropped_rows[0][0]
            n_cols_removed_left = self.cropped_cols[0][1]-self.cropped_cols[0][0]

            row = (self.orig_img.shape[1]-1) - int(round((abs(origin_y) / resolution))) - n_rows_removed_top  # flipped coordinate system on y-axis
            column = int(round((abs(origin_x) / resolution))) - n_cols_removed_left
        else:
            # Calculate row and column of cell
            row = (img.shape[1]-1) - int(round((abs(origin_y) / resolution))) # flipped coordinate system on y-axis
            column = int(round((abs(origin_x) / resolution)))

        # Mark robot start cell with -1
        img[row, column] = -1 # changes value in place, no need to return

    def place_target(self, img):
        x_pos = 0
        y_pos = 0

        with open(os.path.join(os.path.expanduser("~"),
            'catkin_ws/src/robotcraft_maze/scans/robot_position.txt'), 'r') as f:
            x_pos = float(f.readline())
            y_pos = float(f.readline())

        # Get matrix coordinates of initial robot position
        result = np.where(img == -1)
        initial_pos = (result[0][0], result[1][0]) # extract indices

        # Calculate target cell using final pose and starting cell
        resolution = self.occupancy_grid.info.resolution
        target_row = initial_pos[0] + int(round(-y_pos / resolution))
        target_col = initial_pos[1] + int(round(x_pos / resolution))

        # Mark target cell with -3
        img[target_row, target_col] = -2 # changes value in place, no need to return

        # TODO: Move target cell by one in any direction if right next to wall,
        # otherwise A* algorithm might fail depending on implementation

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
            self.cropped_rows = [(0, cols[0]), (cols[-1], image.shape[1])]
            self.cropped_cols = [(0, rows[0]), (rows[-1], image.shape[0])]
            image = image[cols[0]: cols[-1] + 1, rows[0]: rows[-1] + 1]
        else:
            image = image[:1, :1]

        return image
