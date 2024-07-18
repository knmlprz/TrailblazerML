import json
import os
import serial
import pynmea2

outputDict = {}
ser = serial.Serial(port="/dev/ttyTHS0", baudrate=9600)

# Upewnij się, że katalog istnieje
output_path = "/home/jetson/repos/TrailblazerML/utils"
os.makedirs(output_path, exist_ok=True)

while True:
    try:
        newline = ser.readline().decode("utf-8").strip()
        parsedLine = pynmea2.parse(newline)
        if newline.find("$GPRMC") > -1:
            print(f"outputDict: {outputDict}")
            outputDict['latitude'] = parsedLine.latitude
            outputDict['longitude'] = parsedLine.longitude
            with open(os.path.join(output_path, "position.json"), 'w') as OutputFile:
                json.dump(outputDict, OutputFile)
    except UnicodeDecodeError:
        print("Invalid byte encountered. Skipping.")
    except pynmea2.nmea.ParseError:
        print("Could not parse NMEA sentence. Skipping.")
    except FileNotFoundError as e:
        print(f"File not found: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
