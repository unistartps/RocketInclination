#include "binary_serial.hpp"

void readData(void* data, size_t nbBytes) {
  while (Serial.available() < nbBytes);
  Serial.readBytes((byte*) data, nbBytes);
}

void writeData(void* data, size_t nbBytes) {
  byte* byteData = (byte*) data;
  Serial.write(byteData, nbBytes);
}
