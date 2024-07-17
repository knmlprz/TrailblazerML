import serial
import pynmea2
import time
import string
import json

outputDict = {}
ser = serial.Serial(port="/dev/ttyTHS0", baudrate=9600)
with open("GPSdata.json", 'w') as OutputFile:
    while True:
        try:
            newline = ser.readline().decode("utf-8").strip()
            parsedLine = pynmea2.parse(newline)
            if newline.find("$GPRMC") > -1:
                print(f"outputDict{outputDict }")
                outputDict['latitude'] = parsedLine.latitude
                outputDict['longitude'] = parsedLine.longitude
                json.dump(outputDict, OutputFile)
        except UnicodeDecodeError:
            print("Invalid byte encountered. Skipping.")
        except pynmea2.nmea.ParseError:
            print("Could not parse NMEA sentence. Skipping.")








