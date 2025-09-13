#include <Wire.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include "DFRobot_GNSS.h"
#include <FastLED.h>

#define NUM_LEDS 16
#define DATA_PIN 15

CRGB leds[NUM_LEDS];

DFRobot_GNSS_I2C gnss(&Wire ,GNSS_DEVICE_ADDR);
Adafruit_LIS3MDL lis3mdl;

void setupMagnitometer(){
  if(!lis3mdl.begin_I2C(0x1C, &Wire1)){
    Serial.println("Failed to find LIS3MDL chip");
    while (1) { delay(10); }
  }
  Serial.println("LIS3MDL Found!");

  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  

  lis3mdl.setIntThreshold(500);
  lis3mdl.configInterrupt(false, false, true, // enable z axis
                          true, // polarity
                          false, // don't latch
                          true); // enabled!
}

void setupGNNS(){
  while(!gnss.begin()){
    Serial.println("NO gps !");
    delay(1000);
  }
  
  gnss.enablePower();      
  gnss.setGnss(eGPS_BeiDou_GLONASS);
  gnss.setRgbOn();
}

void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10); // pause until serial monitor open

  Wire1.setSDA(26);
  Wire1.setSCL(27);

  setupMagnitometer();
  setupGNNS();

  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.clear();
  FastLED.show();
}

void getGpsData(){
  sLonLat_t lat = gnss.getLat();
  sLonLat_t lon = gnss.getLon();

  uint8_t starUsed = gnss.getNumSatUsed();



  Serial.print(lat.latitudeDegree,6);
  Serial.print((char)lon.lonDirection);

  Serial.print(", ");
  Serial.print(lon.lonitudeDegree,6);
  Serial.print((char)lat.latDirection); // shit fucking driver

  Serial.print(" - ");
  Serial.println(starUsed);
}

void getMagnitometerData(){
  sensors_event_t event;
  lis3mdl.getEvent(&event);
  float mx = event.magnetic.x;
  float my = event.magnetic.y;
  float mz = event.magnetic.z;

  // Calculate the heading using the X and Y axes
  float heading_rad = atan2(my, mx);

  // Convert the heading from radians to degrees
  float heading_deg = heading_rad * 180 / PI;

  // Adjust for a 0-360 degree compass
  if (heading_deg < 0) {
    heading_deg += 360;
  }

  // Print the results
  // Serial.print("X: "); Serial.print(mx);
  // Serial.print(" \tY: "); Serial.print(my);
  // Serial.print(" \tZ: "); Serial.print(mz);
  // Serial.print(" uTesla \tHeading: ");
  Serial.print(heading_deg);
  Serial.println(" degrees");

}

void loop() {
  getMagnitometerData();
  getGpsData();

  // Turn the first LED red
  leds[0] = CRGB::Purple;
  FastLED.show();
  delay(500);

  // Turn the first LED off
  leds[0] = CRGB::Black;
  FastLED.show();
}