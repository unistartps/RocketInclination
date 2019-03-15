#!/usr/bin/env python3

import serial
import struct
import time

typesDict = {'char': 'c', 'bool': '?',
             'int8': 'b', 'uint8': 'B',
             'int16': 'h', 'uint16': 'H',
             'int32': 'i', 'uint32': 'I',
             'int64': 'l', 'uint64': 'L',
             'float': 'f'}

def computeFormat(structFormat):
    """Compute the format string for struct.(pack/unpack)"""
    structTypes = '='

    for t in structFormat:
        structTypes += typesDict[t]

    return structTypes

def readData(ser, structFormat):
    structTypes = computeFormat(structFormat)
    nbBytes = struct.calcsize(structTypes)
    # Wait until all the data is in the buffer
    while ser.in_waiting < nbBytes:
        pass
    # Read the raw data
    rawData = bytearray(nbBytes)
    ser.readinto(rawData)
    # Convert the raw data
    data = list(struct.unpack(structTypes, rawData))
    return data

def writeData(ser, structFormat, data):
    structTypes = computeFormat(structFormat)
    rawData = struct.pack(structTypes, *data)
    ser.write(rawData)

if __name__ == '__main__':
    portName = '/dev/ttyUSB0'
    baudRate = 38400

    waitTime = 1000

    # Define the format of the structure of data sent
    structFormatMeasure = ['uint32'] + ['int16'] * 10

    with serial.Serial(portName, baudRate, timeout=1) as ser:
        # Wait for the arduino to initilize
        time.sleep(2)
        # Read test connection of the sensor
        testAdxl, testMpu = readData(ser, ['bool'] * 2)
        # Send the wait time if at least one sensor is online
        # if it is not sent the arduino do not start the measures
        if testAdxl or testMpu:
            writeData(ser, ['uint32'], [waitTime])

        while testAdxl or testMpu:
            # Read the raw data
            timestamp, adxlAx, adxlAy, adxlAz, mpuAx, mpuAy, mpuAz, mpuGx, mpuGy, mpuGz, mpuTemp = readData(ser, structFormatMeasure)

            # Conversion and display of the data
            timestamp /= 1000
            print('Timestamp (s)', timestamp)
            adxlAx *= 0.0043
            adxlAy *= 0.0043
            adxlAz *= 0.0043
            if testAdxl:
                print('ADXL345 online')
                print('\tAcceleration (g)', adxlAx, adxlAy, adxlAz)
                mpuAx /= 16384
                mpuAy /= 16384
                mpuAz /= 16384
            else:
                print('ADXL345 offline')
            if testMpu:
                print('MPU6050 online')
                print('\tAcceleration (g)', mpuAx, mpuAy, mpuAz)
                mpuGx /= 131
                mpuGy /= 131
                mpuGz /= 131
                print('\tRotation (w/s)', mpuGx, mpuGy, mpuGz)
                mpuTemp = (mpuTemp+521)/340+35
                print('\tTemperature (C°)', mpuTemp)
            else:
                print('MPU6050 offline')
            print()
