import os
import cv2
import numpy as np

def loadMap():
    # Load image and crop

    scans_dir = os.path.join(os.path.expanduser("~"),"catkin_ws/src/robotcraft_maze/scans/")
    img = cv2.imread(os.path.join(scans_dir, "map_backup.pgm"), cv2.IMREAD_GRAYSCALE)
    img = autocrop(img) 

    # Convert colors into 0 and 1
    dark_colors = np.where(img <= 220)
    img[dark_colors] = 0
    light_colors = np.where(img > 220)
    img[light_colors] = 1

    # Flip number so that 0 = free and 1 = occupied
    img = np.logical_not(img).astype(int)

    # Save image to scans folder
    cv2.imwrite(os.path.join(scans_dir, "map_filtered.pgm"), img*255)

    return img

def autocrop(image, lower_threshold=100, upper_threshold=220):
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
