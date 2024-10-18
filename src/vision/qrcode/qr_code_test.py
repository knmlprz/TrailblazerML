import cv2
from qr_code import ReadARUCOCode

vid = cv2.VideoCapture(0)
aruco = ReadARUCOCode()
while True:
    ret, frame = vid.read()
    isMarker, markerDict = aruco.read(frame, True)
    print(isMarker, f"dict: {markerDict}")
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

vid.release()
cv2.destroyAllWindows()
