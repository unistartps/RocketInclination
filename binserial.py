#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""BinSerial is a library to transmit data in binary via serial link.

TODO:
- Finish Documentation.
- Catch potential exceptions
- Implement multi-treading.

"""

__all__ = ['BinSerial']
__version__ = '1.0'
__author__ = 'Jonathan Plasse'

# For serial connection
import serial
# To convert binary data
import struct
import time


class BinSerial:
    """Send and Receive serial data in binary."""

    def __init__(self, port_name, baud_rate):
        """Open a serial link at port_name with baud_rate."""
        # Dict to convert explicit types to format caracters
        self.format_dict = {'char': 'c', 'bool': '?',
                            'int8': 'b', 'uint8': 'B',
                            'int16': 'h', 'uint16': 'H',
                            'int32': 'i', 'uint32': 'I',
                            'int64': 'l', 'uint64': 'L',
                            'float': 'f'}
        # Port name of the serial port
        self.port_name = port_name
        # Baud rate of the connection
        self.baud_rate = baud_rate

        # Open serial link
        self.ser = serial.Serial(self.port_name, self.baud_rate, timeout=1)

        # Wait for initialisation
        time.sleep(2)

    def __del__(self):
        """Close serial link."""
        self.ser.close()

    def _compute_format(self, struct_format):
        """Compute the format caracters to convert binary data."""
        # Set byte order to native and no alignement for the binary data
        format_caracters = '='

        # Convert struct_format explicit types to format caracters
        for t in struct_format:
            format_caracters += self.format_dict[t]

        return format_caracters

    def read(self, struct_format):
        """Read data from serial link with struct_format."""
        # Compute the format caracters
        format_caracters = self._compute_format(struct_format)
        # Calculate the number of bytes needed
        nb_bytes = struct.calcsize(format_caracters)

        # Wait until all the data is in the buffer
        while self.ser.in_waiting < nb_bytes:
            pass

        # Read the raw data
        raw_data = bytearray(nb_bytes)
        self.ser.readinto(raw_data)

        # Convert the raw data
        data = list(struct.unpack(format_caracters, raw_data))

        return data

    def write(self, struct_format, data):
        """Write data to serial link with struct_format."""
        # Compute the format caracters
        format_caracters = self._compute_format(struct_format)

        # Convert data in binary
        raw_data = struct.pack(format_caracters, *data)
        # Send raw_data
        self.ser.write(raw_data)


if __name__ == '__main__':
    port_name = '/dev/ttyUSB0'
    baud_rate = 115200

    # Define the format of the structure of the data
    struct_format = ['float']*2+['int16']

    # Initialize connection
    bser = BinSerial(port_name, baud_rate)

    # Test echo
    bser.write(struct_format, [2.718, 3.14, 5203])
    data = bser.read(struct_format)

    print(data)
