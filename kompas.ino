// nclude <Wire.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include "DFRobot_GNSS.h"
#include <FastLED.h>
#include <EEPROM.h>

#define NUM_LEDS 16
#define DATA_PIN 16

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

// Hardcoded target coordinate
const double targetLat = 43.476653;
const double targetLon = -80.539472;

// Calibration offsets - ADJUST THESE VALUES AFTER TESTING
const double LED_OFFSET_DEGREES = 67.5;        // Adjust this: LED index 0 vs magnetometer 0°
const double MAGNETIC_DECLINATION_DEGREES = -260.0;  //djust this: magnetometer 0° vs True North

// Save GPS fix to flash
void saveGpsFix(double lat, double lon, uint8_t satUsed) {
  GpsFix fix = {lat, lon, satUsed};
  EEPROM.put(0, fix);
  EEPROM.commit();
  cachedFix = fix;
  Serial.printf("Cached to flash: %.6f, %.6f (%u sats)\n", lat, lon, satUsed);
}

// Load GPS fix from flash
GpsFix loadGpsFix() {
  GpsFix fix;
  EEPROM.get(0, fix);
  return fix;
}

// Compute bearing from current to target using standard formula
double computeBearing(double lat1, double lon1, double lat2, double lon2) {
    double deltaLat = lat2 - lat1; // in degrees
    double deltaLon = lon2 - lon1; // in degrees

    double x = deltaLon * cos(radians(lat1));
    double y = deltaLat;

    double bearingRad = atan2(x, y);  // note: atan2(x, y), not atan2(y, x)
    double bearingDeg = bearingRad * 180.0 / PI;

    if (bearingDeg < 0) bearingDeg += 360;  // normalize to 0-360°

    return bearingDeg;
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
  lis3mdl.configInterrupt(false, false, true,
                          true, false, true);
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
  while (!Serial) delay(10);

  Wire1.setSDA(26);
  Wire1.setSCL(27);

  setupMagnitometer();
  setupGNNS();

  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.clear();
  FastLED.show();

  EEPROM.begin(512);
  cachedFix = loadGpsFix();
  Serial.printf("Last stored fix: %.6f, %.6f (%u sats)\n",
                cachedFix.lat, cachedFix.lon, cachedFix.satUsed);
                
  Serial.printf("Calibration offsets: LED=%.1f°, Magnetic=%.1f°\n", 
                LED_OFFSET_DEGREES, MAGNETIC_DECLINATION_DEGREES);
}

// Read GPS + update cached coords
void getGpsData(double &latitude, double &longitude){
  sLonLat_t lat = gnss.getLat();
  sLonLat_t lon = gnss.getLon();
  uint8_t starUsed = gnss.getNumSatUsed();

  latitude  = lat.latitudeDegree;
  longitude = lon.lonitudeDegree;

 if(lat.latDirection == 'W'){        // lat field contains longitude direction
    longitude = -longitude;         // negate longitude for West
}

if(lon.lonDirection == 'S'){        // lon field contains latitude direction  
    latitude = -latitude;           // negate latitude for South
}

  if (starUsed > 0) {
    Serial.printf("GPS fix: %.6f%c, %.6f%c - %u sats\n",
                  latitude, (char)lon.lonDirection,
                  longitude, (char)lat.latDirection,
                  starUsed);
    saveGpsFix(latitude, longitude, starUsed);
  } else {
    Serial.printf("No GPS, using cached fix: %.6f, %.6f (%u sats)\n",
                  cachedFix.lat, cachedFix.lon, cachedFix.satUsed);
    latitude = cachedFix.lat;
    longitude = cachedFix.lon;
  }
}

// Get compass heading from magnetometer
double getHeading() {
  sensors_event_t event;
  lis3mdl.getEvent(&event);
  float heading_rad = atan2(event.magnetic.y - 8.60, event.magnetic.x + 22.05);
  double heading_deg = heading_rad * (180.0 / PI);
  if (heading_deg < 0) heading_deg += 360;
  
  // Apply magnetic declination to align with True North
  heading_deg += MAGNETIC_DECLINATION_DEGREES;
  if (heading_deg >= 360) heading_deg -= 360;
  if (heading_deg < 0) heading_deg += 360;
  
  Serial.printf("Heading: %.2f degrees\n", heading_deg);
  return heading_deg;
}

// Update LEDs to point to target
void updateLEDs(double latitude, double longitude, double heading) {
  double bearingToTarget = computeBearing(latitude, longitude, targetLat, targetLon);
  double relativeAngle = bearingToTarget - heading;
  if (relativeAngle < 0) relativeAngle += 360;

  // Apply LED offset to account for physical LED positioning
  relativeAngle += LED_OFFSET_DEGREES;
  if (relativeAngle >= 360) relativeAngle -= 360;

  int ledIndex = int((relativeAngle / 360.0) * NUM_LEDS) % NUM_LEDS;
  FastLED.clear();
  leds[ledIndex] = CRGB::Green;
  FastLED.show();
  
  Serial.printf("Bearing to target: %.2f°, Relative angle: %.2f°, LED index: %d\n", 
                bearingToTarget, relativeAngle, ledIndex);
}

void loop() {
  double lat, lon;
  getGpsData(lat, lon);

  double heading = getHeading();
  
  updateLEDs(lat, lon, heading);

  delay(100);
}