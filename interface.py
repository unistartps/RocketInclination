#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""interface get sensor data from the experiment board.
Calibrate and correct the measures.
Compute a sensor fusion.

TODO:
- Test calibration method.
- Improve data fusion.
- Documentation.
- Implement other data fusion method.

"""

__version__ = '0.1'
__author__ = 'Jonathan Plasse'

# To communicate with the arduino
from binserial import BinSerial
import numpy as np
import yaml


def process_data(raw_data):
    """Process raw_data into readable data."""
    # Assure raw_data is not modified.
    data = np.array(raw_data.copy(), dtype=float)

    # Time
    data[0] /= 1000

    # ADXL Acceleration
    data[1:4] = data[1:4] * 0.0043

    # MPU Acceleration
    data[4:7] = data[4:7] / 16384

    # MPU Speed Rotation
    data[7:10] = data[7:10] / 131

    # Temperature
    data[10] = (data[10]+521)/340 + 35

    return data


def calculate_offset_scale(up, down):
    """Calculate the offset and scale for the calibration."""
    offset = (up + down) / 2
    scale = (up - down) / 2
    return offset, scale


def apply_correction(data, offsets, scales):
    """Apply the correction on the data (offsets and scales)."""
    for i in range(len(data)):
        data[i] = (data[i] - offsets[i]) / scales[i]


def calibrate(bser, nb_measure_calibration, run=False):
    """Give the offsets and scales for the data correction."""
    # Tell the arduino if the calibration is done
    bser.write(['bool'], [run])

    if run:
        # Initialization of offsets and scales corrections
        offsets = np.zeros(11)
        scales = np.ones(11)

        # Send the number of measures to do for each calibrations
        bser.write(['uint8'], [nb_measure_calibration])

        # Wait for confirmation.
        input('Do not move')
        # Send ok to the board to start sending measures to calibrate.
        bser.write(['bool'], [True])

        # Calibrate gyroscope.
        for i in range(nb_measure_calibration):
            # Receive and process data
            raw_data = bser.read(struct_format_measure)
            data = process_data(raw_data)
            # Calibrate
            offsets[7:10] = offsets[7:10] + data[7:10]\
                / nb_measure_calibration

        # Calibrate accelerometers.
        xyz = {0: 'x', 1: 'y', 2: 'z'}
        ud = {0: 'up', 1: 'down'}
        for i in range(3):
            upDown = np.zeros(4)
            for j in range(2):
                # Wait for confirmation
                input('{} {}'.format(xyz[i], ud[j]))
                # Send ok to the board to start sending measures to calibrate.
                bser.write(['bool'], [True])

                for k in range(nb_measure_calibration):
                    # Receive and process data
                    raw_data = bser.read(struct_format_measure)
                    data = process_data(raw_data)
                    # Calibrate
                    upDown[j::2] = upDown[j::2] + data[i+1:i+5:3]\
                        / nb_measure_calibration

            # Calculate offset and scale
            offsets[i], scales[i] = calculate_offset_scale(*upDown[:2])
            offsets[i+3], scales[i+3] = calculate_offset_scale(*upDown[2:])

        # Save offsets and scales to configuration.csv
        np.savetxt('calibration.csv', [offsets, scales])
    else:
        # Load offsets and scales from configuration.csv
        offsets, scales = np.loadtxt('calibration.csv')

    return offsets, scales


if __name__ == '__main__':
    with open('config.yml', 'r') as f:
        config = yaml.load(f.read())

    # Define the format of the structure of data sent.
    struct_format_measure = ['uint32'] + ['int16'] * 10

    # Initialize connection with the arduino.
    bser = BinSerial(config['port_name'], config['baud_rate'])

    # Read test connection of the sensor.
    test_adxl, test_mpu = bser.read(['bool'] * 2)

    if test_adxl or test_mpu:
        # Calibrate
        offsets, scales = calibrate(bser, config['nb_measure_calibration'],
                                    config['do_calibration'])

        # Send the wait time if at least one sensor is online.
        # If it is not sent the arduino do not start the measures.
        bser.write(['uint32'], [config['wait_time']])

    while test_adxl or test_mpu:
        # Read the raw data.
        raw_data = bser.read(struct_format_measure)
        # Process the raw data into readable data.
        data = process_data(raw_data)
        # Apply data correction.
        apply_correction(data, offsets, scales)

        timestamp, adxl_ax, adxl_ay, adxl_az, mpu_ax, mpu_ay, mpu_az, mpu_gx,\
            mpu_gy, mpu_gz, mpu_temp = data

        # Display the data.
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

        # Sensor Fusion
        ax = (adxl_ax + mpu_ax) / 2
        ay = (adxl_ay + mpu_ay) / 2
        az = (adxl_az + mpu_az) / 2

        thetaA = np.arctan2(az, ax) / np.pi * 180
        phiA = np.arctan2(az, ay) / np.pi * 180

        print(thetaA, phiA)

        print()
