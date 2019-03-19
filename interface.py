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

def computeData(rawData):
    data = rawData.copy()

    # Time
    data[0] /= 1000

    # ADXL Acceleration
    data[1] *= 0.0043
    data[2] *= 0.0043
    data[3] *= 0.0043

    # MPU Acceleration
    data[4] /= 16384
    data[5] /= 16384
    data[6] /= 16384

    # MPU Speed Rotation
    data[7] /= 131
    data[8] /= 131
    data[9] /= 131

    # Temperature
    data[10] = (data[10]+521)/340+35
    return data

def computeOffsetScale(up, down):
    offset = (up + down) / 2
    scale = (up - down) / 2
    return offset, scale

def computeCalibration(data, offsets, scales):
    for i in range(len(data)):
        data[i] = (data[i] - offsets[i]) / scales[i]


if __name__ == '__main__':
    portName = '/dev/ttyUSB0'
    baudRate = 38400

    waitTime = 1000
    nbMeasureCalibration = 10

    # Define the format of the structure of data sent
    structFormatMeasure = ['uint32'] + ['int16'] * 10
    structFormatcalibration = ['uint8']

    with serial.Serial(portName, baudRate, timeout=1) as ser:
        # Wait for the arduino to initilize
        time.sleep(2)
        # Read test connection of the sensor
        testAdxl, testMpu = readData(ser, ['bool'] * 2)

        offsets = [0] * 11
        scales = [1] * 11

        if testAdxl or testMpu:
            # Calibration
            # writeData(ser, structFormatcalibration, [nbMeasureCalibration])
            # for i in range(7, 10):
            #     if i < 7:
            #         upDown = [0] * 2
            #         for j in range(2):
            #             input('{:} {:}'.format(i, j))
            #             writeData(ser, ['bool'], [True])
            #             for k in range(nbMeasureCalibration):
            #                 rawData = readData(ser, structFormatMeasure)
            #                 upDown[j] += computeData(rawData)[i] / nbMeasureCalibration
            #         offsets[i], scales[i] = computeOffsetScale(*upDown)
            #     else:
            #         for k in range(nbMeasureCalibration):
            #             writeData(ser, ['bool'], [True])
            #             rawData = readData(ser, structFormatMeasure)
            #             offsets[i] += computeData(rawData)[i] / nbMeasureCalibration
            #
            # print(offsets, scales)

            offsets = [0, -0.00043000000000004146, 0.05267500000000003, -0.0006449999999998957, 0.04554443359374999, -0.01922607421875, 0.0020751953125, -1.3793893129770993, 1.385496183206107, -2.483969465648855, 0]
            scales = [1, 1.08962, 1.083815, 1.0730650000000002, 0.9987426757812501, 1.01854248046875, 1.0287841796875, 1, 1, 1, 1]

            # Send the wait time if at least one sensor is online
            # if it is not sent the arduino do not start the measures
            writeData(ser, ['uint32'], [waitTime])

        while testAdxl or testMpu:
            # Read the raw data
            rawData = readData(ser, structFormatMeasure)

            data = computeData(rawData)
            computeCalibration(data, offsets, scales)
            timestamp, adxlAx, adxlAy, adxlAz, mpuAx, mpuAy, mpuAz, mpuGx, mpuGy, mpuGz, mpuTemp = data
            # Conversion and display of the data
            print('Timestamp (s)', timestamp)
            if testAdxl:
                print('ADXL345 online')
                print('\tAcceleration (g) x:{:0.2f}, y:{:0.2f}, z:{:0.2f}'.format(adxlAx, adxlAy, adxlAz))
            else:
                print('ADXL345 offline')
            if testMpu:
                print('MPU6050 online')
                print('\tAcceleration (g) x:{:0.2f}, y:{:0.2f}, z:{:0.2f}'.format(mpuAx, mpuAy, mpuAz))
                print('\tRotation (w/s) x:{:0.2f}, y:{:0.2f}, z:{:0.2f}'.format(mpuGx, mpuGy, mpuGz))
                print('\tTemperature (C°) {:0.2f}'.format(mpuTemp))
            else:
                print('MPU6050 offline')
            print()
