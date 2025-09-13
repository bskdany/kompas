#include <Wire.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include "DFRobot_GNSS.h"
#include <FastLED.h>
#include <EEPROM.h>   // <-- Added

#define NUM_LEDS 16
#define DATA_PIN 15

CRGB leds[NUM_LEDS];

DFRobot_GNSS_I2C gnss(&Wire1 ,GNSS_DEVICE_ADDR);
Adafruit_LIS3MDL lis3mdl;

// Struct to hold GPS fix
struct GpsFix {
  double lat;
  double lon;
  uint8_t satUsed;
};

// Cached fix in RAM
GpsFix cachedFix;

// Save GPS fix to flash
void saveGpsFix(double lat, double lon, uint8_t satUsed) {
  GpsFix fix = {lat, lon, satUsed};
  EEPROM.put(0, fix);   // write at address 0
  EEPROM.commit();      // flush to flash
  cachedFix = fix;      // also update RAM copy

  Serial.printf("Cached to flash: %.6f, %.6f (%u sats)\n", lat, lon, satUsed);
}

// Load GPS fix from flash
GpsFix loadGpsFix() {
  GpsFix fix;
  EEPROM.get(0, fix);
  return fix;
}

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

  // init EEPROM with 512 bytes reserved
  EEPROM.begin(512);

  // Load last fix on boot
  cachedFix = loadGpsFix();
  Serial.print("Last stored fix: ");
  Serial.print(cachedFix.lat, 6); Serial.print(", ");
  Serial.print(cachedFix.lon, 6);
  Serial.print(" ("); Serial.print(cachedFix.satUsed); Serial.println(" sats)");
}

void getGpsData(){
  sLonLat_t lat = gnss.getLat();
  sLonLat_t lon = gnss.getLon();
  uint8_t starUsed = gnss.getNumSatUsed();

  double latitude  = lat.latitudeDegree;
  double longitude = lon.lonitudeDegree;

  if (starUsed > 0) {
    // use live data
    Serial.printf("GPS fix: %.6f%c, %.6f%c - %u sats\n",
                  latitude, (char)lon.lonDirection,
                  longitude, (char)lat.latDirection,
                  starUsed);
    saveGpsFix(latitude, longitude, starUsed);
  } else {
    // fall back to cached coords
    Serial.printf("No GPS, using cached fix: %.6f, %.6f (%u sats)\n",
                  cachedFix.lat, cachedFix.lon, cachedFix.satUsed);
  }
}

void getMagnitometerData(){
  sensors_event_t event;
  lis3mdl.getEvent(&event);
  float mx = event.magnetic.x;
  float my = event.magnetic.y;

  float heading_rad = atan2(my, mx);
  float heading_deg = heading_rad * 180 / PI;
  if (heading_deg < 0) heading_deg += 360;

  Serial.print(heading_deg);
  Serial.println(" degrees");
}

void loop() {
  getMagnitometerData();
  getGpsData();

  leds[0] = CRGB::Purple;
  FastLED.show();
  delay(500);

  leds[0] = CRGB::Black;
  FastLED.show();
}