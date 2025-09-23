#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  uint64_t chipid = ESP.getEfuseMac();
  Serial.printf("DevEUI (전체): %016llX\n", chipid);
  
  // 바이트별 출력
  Serial.print("DevEUI (bytes): ");
  for(int i = 7; i >= 0; i--) {
    Serial.printf("%02X", ((uint8_t*)&chipid)[i]);
    if(i > 0) Serial.print(" ");
  }
  Serial.println();
}

void loop() { 
  delay(1000); 
}