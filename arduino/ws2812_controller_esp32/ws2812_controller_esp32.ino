/*
* This example works for ESP32 and uses the NeoPixelBus library instead of the one bundle
* Sketch Written by Joey Babcock - https://joeybabcock.me/blog/
* Codebase created by ScottLawsonBC - https://github.com/scottlawsonbc
*/

#include <Arduino.h>
#include <NeoPixelBus.h>
#include "SerialTransfer.h"


// Set to the number of LEDs in your LED strip
#define NUM_LEDS 10
// Toggles FPS output (1 = print FPS over serial, 0 = disable output)
#define PRINT_FPS 1

//NeoPixelBus settings
const uint8_t PixelPin = 3;  // make sure to set this to the correct pin, ignored for Esp8266(set to 3 by default for DMA)

SerialTransfer myTransfer;

NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> ledstrip(NUM_LEDS, PixelPin);

uint16_t N = 0;

struct pixel_data{
  uint8_t pixNum;   //offset
  uint8_t data[90]; //30 pixels of r,g,b i.e. 3*30
} pixelData;

void setup() {
    Serial.begin(115200);
    myTransfer.begin(Serial);
    ledstrip.Begin();//Begin output
    ledstrip.Show();//Clear the strip for use
}

void loop() {
    if (myTransfer.available()) {
        myTransfer.rxObj(pixelData, sizeof(pixelData));
        for (uint8_t i=0; i < sizeof(pixelData.data); i=i+3){
            RgbColor pixel(pixelData.data[i], pixelData.data[i + 1], pixelData.data[i + 2]);
            ledstrip.SetPixelColor(((uint16_t) pixelData.pixNum * 30) + i / 3, pixel);
        }
        ledstrip.Show();
    }
}
