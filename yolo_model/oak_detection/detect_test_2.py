from pathlib import Path
import sys
import cv2
import depthai as dai
import numpy as np

# Ścieżka do pliku blob z modelem
nnPath = str((Path(__file__).parent / Path(
    'depthai-python/examples/models/mobilenet-ssd/mobilenet-ssd_openvino_2021.4_5shave.blob')).resolve().absolute())
if len(sys.argv) > 1:
    nnPath = sys.argv[1]

if not Path(nnPath).exists():
    raise FileNotFoundError(f'Required file/s not found, please run "{sys.executable} install_requirements.py"')

# Lista etykiet modelu MobileNetSSD
labelMap = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow",
            "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"]

# Ustawiamy, że interesuje nas tylko obiekt "bottle" (indeks 5)
allowed_class_ids = [5]

# Tworzymy pipeline
pipeline = dai.Pipeline()

# Definiujemy źródła i wyjścia
monoRight = pipeline.create(dai.node.MonoCamera)
monoLeft = pipeline.create(dai.node.MonoCamera)
stereo = pipeline.create(dai.node.StereoDepth)
manip = pipeline.create(dai.node.ImageManip)
nn = pipeline.create(dai.node.MobileNetDetectionNetwork)

nnOut = pipeline.create(dai.node.XLinkOut)
disparityOut = pipeline.create(dai.node.XLinkOut)
xoutRight = pipeline.create(dai.node.XLinkOut)

disparityOut.setStreamName("disparity")
xoutRight.setStreamName("rectifiedRight")
nnOut.setStreamName("nn")

# Ustawienia kamer mono
monoRight.setCamera("right")
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setCamera("left")
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

# Konfiguracja stereo – wysokiej gęstości
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
stereo.setRectifyEdgeFillColor(0)  # Czarny kolor krawędzi

# Konfiguracja ImageManip – zmiana rozmiaru na 300x300 i konwersja do BGR
manip.initialConfig.setResize(300, 300)
manip.initialConfig.setFrameType(dai.ImgFrame.Type.BGR888p)

# Konfiguracja sieci detekcji
nn.setConfidenceThreshold(0.5)
nn.setBlobPath(nnPath)
nn.setNumInferenceThreads(2)
nn.input.setBlocking(False)

# Łączenie elementów pipeline
monoRight.out.link(stereo.right)
monoLeft.out.link(stereo.left)
stereo.rectifiedRight.link(manip.inputImage)
stereo.disparity.link(disparityOut.input)
manip.out.link(nn.input)
manip.out.link(xoutRight.input)
nn.out.link(nnOut.input)


# Funkcja normalizująca współrzędne ramki wykrycia
def frameNorm(frame, bbox):
    normVals = np.full(len(bbox), frame.shape[0])
    normVals[::2] = frame.shape[1]
    return (np.clip(np.array(bbox), 0, 1) * normVals).astype(int)


# Funkcja, która rysuje wykrycia wraz z informacją o odległości
def show(name, frame, raw_disp, detections):
    color = (255, 0, 0)
    for detection in detections:
        # Filtrujemy tylko wykrycia dla allowed_class_ids
        if detection.label not in allowed_class_ids:
            continue
        bbox = frameNorm(frame, (detection.xmin, detection.ymin, detection.xmax, detection.ymax))

        # Pobieramy region z surowego obrazu dyspersji (upewniamy się, że rozmiar się zgadza)
        region = raw_disp[bbox[1]:bbox[3], bbox[0]:bbox[2]]
        if region.size > 0:
            avg_disp = np.mean(region)
            # Unikamy dzielenia przez zero – jeśli dyspersja jest zbyt mała, nie liczymy odległości
            if avg_disp > 1e-6:
                focal_length = 200  # przybliżona ogniskowa w pikselach (do dostosowania)
                baseline = 0.075  # baza w metrach (75 mm)
                distance = (focal_length * baseline) / avg_disp
            else:
                distance = None
        else:
            distance = None

        # Rysujemy prostokąt i tekst – wyświetlamy etykietę, pewność oraz (jeśli dostępna) odległość
        cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color, 2)
        text = f"{labelMap[detection.label]}: {int(detection.confidence * 100)}%"
        if distance is not None:
            text += f", {distance:.2f} m"
        cv2.putText(frame, text, (bbox[0] + 10, bbox[1] + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color, 1)
    cv2.imshow(name, frame)


# Przelicznik do wizualizacji dyspersji
disparityMultiplier = 255 / stereo.initialConfig.getMaxDisparity()

with dai.Device(pipeline) as device:
    # Kolejki wyjść
    qRight = device.getOutputQueue("rectifiedRight", maxSize=4, blocking=False)
    qDisparity = device.getOutputQueue("disparity", maxSize=4, blocking=False)
    qDet = device.getOutputQueue("nn", maxSize=4, blocking=False)

    rightFrame = None
    raw_disp = None
    detections = []

    while True:
        if qDet.has():
            detections = qDet.get().detections

        if qRight.has():
            rightFrame = qRight.get().getCvFrame()

        if qDisparity.has():
            disp_packet = qDisparity.get()
            # Pobieramy surową dyspersję (float) – nieprzeskalowaną
            raw_disp = disp_packet.getFrame()
            # Dla wizualizacji tworzymy skalowany obraz dyspersji
            disp_vis = (raw_disp * disparityMultiplier).astype(np.uint8)
            disp_vis = cv2.applyColorMap(disp_vis, cv2.COLORMAP_JET)
            cv2.imshow("disparity", disp_vis)

        # Upewnij się, że rightFrame oraz raw_disp są dostępne
        if rightFrame is not None and raw_disp is not None:
            # Jeśli rozmiary się nie zgadzają, zmień rozmiar surowej dyspersji do rozmiaru rightFrame
            if (raw_disp.shape[1], raw_disp.shape[0]) != (rightFrame.shape[1], rightFrame.shape[0]):
                raw_disp_resized = cv2.resize(raw_disp, (rightFrame.shape[1], rightFrame.shape[0]))
            else:
                raw_disp_resized = raw_disp

            show("rectified right", rightFrame, raw_disp_resized, detections)

        if cv2.waitKey(1) == ord('q'):
            break
