/*
 * For this to work, in cores/arduino/main.cpp "PluggableUSBD().begin();" and 
 * "_SerialUSB.begin(115200);" must be suppressed somehow
 */

#include "mbed.h"
#include "USBSerial.h"

USBSerial usb_serial;

void setup() {
}

void loop() {
    // Send a message over USB CDC every second
    while (true) {
        usb_serial.printf("Hello, USB CDC!\n");
        delay(1000);
    }
}
