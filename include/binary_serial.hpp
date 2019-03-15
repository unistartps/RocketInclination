#ifndef SERIAL_HPP
#define SERIAL_HPP

#include <Arduino.h>

void readData(void* data, size_t nbBytes);
void writeData(void* data, size_t nbBytes);

#endif
