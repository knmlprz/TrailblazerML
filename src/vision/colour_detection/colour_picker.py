import cv2
import numpy as np

# Callback function for trackbars, does nothing
def nothing(x):
    pass

# Initialize the webcam
cap = cv2.VideoCapture("/home/filip/Downloads/Messenger_creation_2568467f-e19a-4d6e-9a99-af0b2ff3f443.mp4")
cap.set(3, 1280)
cap.set(4, 720)

# Create a window named 'Trackbars'
cv2.namedWindow("Trackbars")

# Create trackbars for lower and upper HSV ranges
cv2.createTrackbar("L - H", "Trackbars", 0, 179, nothing)
cv2.createTrackbar("L - S", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("L - V", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("U - H", "Trackbars", 179, 179, nothing)
cv2.createTrackbar("U - S", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("U - V", "Trackbars", 255, 255, nothing)

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture image")
        break

    # Flip the frame horizontally
    frame = cv2.flip(frame, 1)

    # Convert the frame to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Get the trackbar positions for HSV values
    l_h = cv2.getTrackbarPos("L - H", "Trackbars")
    l_s = cv2.getTrackbarPos("L - S", "Trackbars")
    l_v = cv2.getTrackbarPos("L - V", "Trackbars")
    u_h = cv2.getTrackbarPos("U - H", "Trackbars")
    u_s = cv2.getTrackbarPos("U - S", "Trackbars")
    u_v = cv2.getTrackbarPos("U - V", "Trackbars")

    # Set the lower and upper HSV range
    lower_range = np.array([l_h, l_s, l_v])
    upper_range = np.array([u_h, u_s, u_v])

    # Create a mask based on the HSV range
    mask = cv2.inRange(hsv, lower_range, upper_range)

    # Apply the mask to get the result
    res = cv2.bitwise_and(frame, frame, mask=mask)

    # Convert the mask to a 3-channel image for stacking
    mask_3 = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

    # Stack the mask, original frame, and result horizontally
    stacked = np.hstack((mask_3, frame, res))

    # Show the stacked images
    cv2.imshow('Trackbars', cv2.resize(stacked, None, fx=0.4, fy=0.4))

    # Break the loop if 'ESC' is pressed
    key = cv2.waitKey(1)
    if key == 27:
        break

    # Save the HSV values if 's' is pressed
    if key == ord('s'):
        thearray = [[l_h, l_s, l_v], [u_h, u_s, u_v]]
        print(thearray)
        break

# Release the webcam and destroy all OpenCV windows
cap.release()
cv2.destroyAllWindows()
