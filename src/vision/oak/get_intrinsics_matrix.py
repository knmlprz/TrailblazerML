import depthai as dai
import numpy as np

# Utworzenie pipeline
pipeline = dai.Pipeline()

# Utworzenie węzła kamery
cam = pipeline.createColorCamera()

# Pobrane parametrów kamery
device = dai.Device(pipeline)
calibration_handler = device.readCalibration()

# Pobrane macierzy wewnętrznych dla głównej kamery (np. kamery RGB)
intrinsics = calibration_handler.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A)
intrinsics_matrix = np.array(intrinsics).reshape(3, 3)

# Ustawienie dokładności wyświetlania

# Zapisanie macierzy do pliku K.txt
with open('K.txt', 'w') as f:
    for row in intrinsics_matrix:
        np.savetxt(f, [row])

print("Macierz wewnętrznych parametrów kamery została zapisana do pliku K.txt")
