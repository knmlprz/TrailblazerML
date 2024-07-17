import numpy as np
import cv2


def detect_red_colour(image: np.ndarray) -> bool:
    """
    Detects the presence of the color red in the given image.

    :param image: (np.ndarray) Input image in BGR format.
    :return: (bool) True if the color red is detected in the image, False otherwise.
    """
    # Convert the BGR image to HSV format
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Red colour range
    red_lower = np.array([136, 87, 111], np.uint8)
    red_upper = np.array([180, 255, 255], np.uint8)

    # Create a mask
    red_mask = cv2.inRange(hsv_image, red_lower, red_upper)

    # Dilate the mask to strengthen the red areas
    kernel = np.ones((5, 5), "uint8")
    red_mask = cv2.dilate(red_mask, kernel)

    # Calculate the number of red pixels in the mask
    red_pixels = cv2.countNonZero(red_mask)

    # Determine if the color red is detected in the image
    if red_pixels > 0:
        return True

    return False
