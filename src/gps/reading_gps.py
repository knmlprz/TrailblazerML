import serial
import pynmea2
import time
import string
import json

outputDict = {}
ser = serial.Serial(port="dev/ttyTHS0", baudrate=9600)
with open("GPSdata.json", 'w') as OutputFile:
    while True:
        newline = ser.readline().decode("utf-8").split()
        parsedLine = pynmea2.parse(newline)
        if parsedLine.find("GPRMC") > -1:
            outputDict['latitude'] = parsedLine.latitude
            outputDict['longitude'] = parsedLine.longitude
            json.dump(outputDict, OutputFile)






