#include <Wire.h>

#define SENSOR_SDA_PIN 41
#define SENSOR_SCL_PIN 42

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("I2C Scanner Starting...");
  
  // 센서용 I2C 초기화
  Wire.begin(SENSOR_SDA_PIN, SENSOR_SCL_PIN);
  
  Serial.println("I2C Scanner");
  Serial.println("Scanning...");
}

void loop() {
  byte error, address;
  int nDevices;
  
  Serial.println("Scanning I2C addresses...");
  
  nDevices = 0;
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println(" !");
      
      nDevices++;
    }
  }
  
  if (nDevices == 0) {
    Serial.println("No I2C devices found");
  } else {
    Serial.println("Scan complete - " + String(nDevices) + " devices found");
  }
  
  Serial.println("------------------------");
  delay(5000);
}