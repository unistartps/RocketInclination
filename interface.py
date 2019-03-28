#!/usr/bin/env python3

from binserial import BinSerial
import numpy as np


def compute_data(raw_data):
    data = raw_data.copy()

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


def compute_offset_scale(up, down):
    offset = (up + down) / 2
    scale = (up - down) / 2
    return offset, scale


def compute_calibration(data, offsets, scales):
    for i in range(len(data)):
        data[i] = (data[i] - offsets[i]) / scales[i]


def calibration(bser):
    bser.write(struct_format_calibration, [nb_measure_calibration])
    for i in range(7, 10):
        if i < 7:
            upDown = [0] * 2
            for j in range(2):
                input('{:} {:}'.format(i, j))
                bser.write(['bool'], [True])
                for k in range(nb_measure_calibration):
                    raw_data = bser.read(struct_format_measure)
                    upDown[j] += compute_data(raw_data)[i]\
                        / nb_measure_calibration
            offsets[i], scales[i] = compute_offset_scale(*upDown)
        else:
            for k in range(nb_measure_calibration):
                bser.write(['bool'], [True])
                raw_data = bser.read(struct_format_measure)
                offsets[i] += compute_data(raw_data)[i]\
                    / nb_measure_calibration

    print(offsets, scales)


if __name__ == '__main__':
    port_name = '/dev/ttyUSB0'
    baud_rate = 38400

    bser = BinSerial(port_name, baud_rate)

    wait_time = 1000
    nb_measure_calibration = 10

    # Define the format of the structure of data sent
    struct_format_measure = ['uint32'] + ['int16'] * 10
    struct_format_calibration = ['uint8']

    # Read test connection of the sensor
    test_adxl, test_mpu = bser.read(['bool'] * 2)

    offsets = [0] * 11
    scales = [1] * 11

    if test_adxl or test_mpu:
        # Calibration
        # calibration(bser)

        offsets = [0, -0.00043000000000004146, 0.05267500000000003,
                   -0.0006449999999998957, 0.04554443359374999,
                   -0.01922607421875, 0.0020751953125, -1.3793893129770993,
                   1.385496183206107, -2.483969465648855, 0]
        scales = [1, 1.08962, 1.083815, 1.0730650000000002, 0.9987426757812501,
                  1.01854248046875, 1.0287841796875, 1, 1, 1, 1]

        # Send the wait time if at least one sensor is online
        # if it is not sent the arduino do not start the measures
        bser.write(['uint32'], [wait_time])

    while test_adxl or test_mpu:
        # Read the raw data
        raw_data = bser.read(struct_format_measure)

        data = compute_data(raw_data)
        compute_calibration(data, offsets, scales)
        timestamp, adxl_ax, adxl_ay, adxl_az, mpu_ax, mpu_ay, mpu_az, mpu_gx,\
            mpu_gy, mpu_gz, mpu_temp = data
        # Conversion and display of the data
        print('Timestamp (s)', timestamp)
        if test_adxl:
            print('ADXL345 online')
            print('\tAcceleration (g) x:{:0.2f}, y:{:0.2f}, z:{:0.2f}'.format(
                adxl_ax, adxl_ay, adxl_az))
        else:
            print('ADXL345 offline')
        if test_mpu:
            print('MPU6050 online')
            print('\tAcceleration (g) x:{:0.2f}, y:{:0.2f}, z:{:0.2f}'.format(
                mpu_ax, mpu_ay, mpu_az))
            print('\tRotation (w/s) x:{:0.2f}, y:{:0.2f}, z:{:0.2f}'.format(
                mpu_gx, mpu_gy, mpu_gz))
            print('\tTemperature (C°) {:0.2f}'.format(mpu_temp))
        else:
            print('MPU6050 offline')

        ax = (adxl_ax + mpu_ax) / 2
        ay = (adxl_ay + mpu_ay) / 2
        az = (adxl_az + mpu_az) / 2

        thetaA = np.arctan2(az, ax) / np.pi * 180
        phiA = np.arctan2(az, ay) / np.pi * 180

        print(thetaA, phiA)

        print()
