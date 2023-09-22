//TEST de multiples conexiones I2C a los pines 21 (SDA) y 22 (SCL) de un ESP32 DEV KIT
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_APDS9960.h>
#include "MCP3X21.h" 
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme;
Adafruit_APDS9960 apds;

MCP3221 mcp3221(0x4D);

const uint16_t ref_voltage = 3300;

void setup() {
  Serial.begin(115200);

  bool status1 = bme.begin(0x76);
  if (!status1) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1)
      ;
  } else {
    Serial.println("Valid BM280 sensor");
  }

  bool status2 = apds.begin(0x39);
  if (!status2) {
    Serial.println("Could not find a valid IR Gesture, check wiring!");
    while (1)
      ;
  } else {
    Serial.println("Valid IR Gesture sensor");
  }

  mcp3221.init();

  apds.enableProximity(true);
  apds.enableGesture(true);
}


void loop() {
  uint8_t gesture = apds.readGesture();
  uint16_t result = mcp3221.read();

  if (gesture == APDS9960_DOWN) {
    Serial.println("v");
    Serial.print("Temperature = ");
    Serial.print(bme.readTemperature());
    Serial.println(" *C");
  }
  if (gesture == APDS9960_UP) {
    Serial.println("^");
    Serial.print("Pressure = ");
    Serial.print(bme.readPressure() / 100.0F);
    Serial.println(" hPa");
  }
  if (gesture == APDS9960_LEFT) {
    Serial.println("<");
    Serial.print("Approx. Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");
  }
  if (gesture == APDS9960_RIGHT) {
    Serial.print(F("ADC: "));
    Serial.print(result);
    Serial.print(F(", mV: "));
    Serial.println(mcp3221.toVoltage(result, ref_voltage));
  }
}