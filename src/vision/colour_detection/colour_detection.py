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
    height, width = hsv_image.shape[:2]
    hsv_image = hsv_image[(width/2) - 10: (width/2) + 10, (height/2) - 10: (height/2) + 10]

    # Red colour range
    sand_lower = np.array([136, 87, 111], np.uint8)
    sand_upper = np.array([180, 255, 255], np.uint8)

    # Create a mask
    sand_mask = cv2.inRange(hsv_image, sand_lower, sand_upper)

    # Dilate the mask to strengthen the red areas
    kernel = np.ones((5, 5), "uint8")
    sand_mask = cv2.dilate(sand_mask, kernel)

    # Calculate the number of red pixels in the mask
    sand_pixels = cv2.countNonZero(sand_mask)

    # Determine if the color red is detected in the image
    if sand_pixels > 0:
        return False

    return True
