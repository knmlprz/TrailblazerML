import serial
import pynmea2
import time
import json
from filelock import FileLock

outputDict = {}
ser = serial.Serial(port="/dev/ttyTHS0", baudrate=9600)

lock = FileLock("untils")

with lock:
    with open("GPSdata.json", 'w') as OutputFile:
        while True:
            newline = ser.readline().decode("utf-8").strip()
            try:
                parsedLine = pynmea2.parse(newline)
                if parsedLine.sentence_type == 'RMC':
                    outputDict['latitude'] = parsedLine.latitude
                    outputDict['longitude'] = parsedLine.longitude
                    print(f"outputDict{outputDict}")
                    OutputFile.seek(0)
                    json.dump(outputDict, OutputFile)
                    OutputFile.flush()
                    OutputFile.truncate()
            except pynmea2.ParseError as e:
                print(f"Parse error: {e}")
