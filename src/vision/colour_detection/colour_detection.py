import numpy as np
import cv2


def detect_sand_colour(image: np.ndarray) -> bool:
    """
    Detects the presence of the sand color in the given image.

    :param image: (np.ndarray) Input image in BGR format.
    :return: (bool) True if the sand color is detected in the image, False otherwise.
    """
    # Convert the BGR image to HSV format
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    height, width = hsv_image.shape[:2]

    # Define the center region of the image
    center_x, center_y = width // 2, height // 2
    # hsv_image = hsv_image[center_y - 10:center_y + 10, center_x - 10:center_x + 10]

    # Sand colour range (in HSV)
    red_lower1 = np.array([0, 70, 50], np.uint8)
    red_upper1 = np.array([10, 255, 255], np.uint8)
    red_lower2 = np.array([170, 70, 50], np.uint8)
    red_upper2 = np.array([180, 255, 255], np.uint8)

    # Create a mask
    red_mask1 = cv2.inRange(hsv_image, red_lower1, red_upper1)
    red_mask2 = cv2.inRange(hsv_image, red_lower2, red_upper2)
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)
    non_red_mask = cv2.bitwise_not(red_mask)

    # Dilate the mask to strengthen the sand color areas
    kernel = np.ones((5, 5), "uint8")
    sand_mask = cv2.dilate(non_red_mask, kernel)

    # Calculate the number of sand color pixels in the mask
    sand_pixels = cv2.countNonZero(sand_mask)

    # Determine if the sand color is detected in the image
    if sand_pixels > 0:
        return False

    return True


# if __name__ == "__main__":
#     # image_example = cv2.imread("Messenger_creation_107682e1-d1e2-49e0-a4ce-e4cc4d7f5931_2.jpg")
#     image_example = cv2.imread("Solid_red.jpg")
#     print(detect_sand_colour(image_example))
