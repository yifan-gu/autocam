#include <Arduino.h>

#include "LED_setup.hpp"

void setupLEDPIN(int pin) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
}

void setLEDColor(int red_pin, int green_pin, int blue_pin, int r, int g, int b, int brightness) {
    analogWrite(red_pin, 255 - r * brightness / 100);   // Invert value for common anode
    analogWrite(green_pin, 255 - g * brightness / 100);
    analogWrite(blue_pin, 255 - b * brightness / 100);
}

void setLEDGreen(int red_pin, int green_pin, int brightness) {
    analogWrite(red_pin, 255 - 128 * brightness / 100);   // Invert value for common anode
    analogWrite(green_pin, 255 - 255 * brightness / 100);
}

void setLEDRed(int red_pin, int green_pin, int brightness) {
    analogWrite(red_pin, 255 - 255 * brightness / 100);   // Invert value for common anode
    analogWrite(green_pin, 255 - 0 * brightness / 100);
}
