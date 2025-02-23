#ifndef LED_SETUP_HPP
#define LED_SETUP_HPP

void setupLEDPIN(int pin);
void setLEDColor(int red_pin, int green_pin, int blue_pin, int r, int g, int b, int brightness = 100);
void setLEDGreen(int red_pin, int green_pin, int brightness = 5);
void setLEDRed(int red_pin, int green_pin, int brightness = 5);

#endif
