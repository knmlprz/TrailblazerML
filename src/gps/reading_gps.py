import serial
import pynmea2
import time
import json

outputDict = {}
ser = serial.Serial(port="/dev/ttyTHS0", baudrate=9600)
with open("GPSdata.json", 'w') as OutputFile:
    while True:
        try:
            # Read and decode the line
            newline = ser.readline().decode("utf-8").strip()

            # Parse the line using pynmea2
            parsedLine = pynmea2.parse(newline)

            # Check if the line is of type GPRMC
            if isinstance(parsedLine, pynmea2.types.talker.GPRMC):
                outputDict['latitude'] = parsedLine.latitude
                outputDict['longitude'] = parsedLine.longitude

                # Write the dictionary to the file as JSON
                json.dump(outputDict, OutputFile)
                OutputFile.write('\n')  # Ensure each record is on a new line

        except pynmea2.ParseError:
            # Handle the case where the line can't be parsed
            continue
        except Exception as e:
            # Handle other exceptions
            print(f"An error occurred: {e}")
            break
