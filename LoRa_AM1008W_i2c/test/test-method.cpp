#include <Arduino.h>
#include <Wire.h>

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("=== Corrected AM1008W-K-P Parsing Test (0x28) ===");
  
  Wire.begin(41, 42);
  Wire.setClock(10000);
  
  delay(1000);
}

void loop() {
  Serial.println("\n--- Reading AM1008W-K-P at 0x28 ---");
  
  Wire.requestFrom(0x28, 25);
  
  if (Wire.available() >= 25) {
    uint8_t response[25];
    
    for (int i = 0; i < 25; i++) {
      response[i] = Wire.read();
    }
    
    Serial.print("Raw data: ");
    for (int i = 0; i < 25; i++) {
      if (response[i] < 16) Serial.print("0");
      Serial.print(response[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
    
    // 다른 바이트 순서로 파싱 시도
    Serial.println("\n--- Parsing attempts ---");
    
    // 방법 1: 리틀 엔디안 (낮은 바이트가 먼저)
    Serial.println("Method 1 (Little Endian):");
    uint16_t co2_1 = response[5] | (response[4] << 8);
    uint16_t voc_1 = response[7] | (response[6] << 8);
    uint16_t hum_raw_1 = response[9] | (response[8] << 8);
    uint16_t temp_raw_1 = response[11] | (response[10] << 8);
    uint16_t pm1_1 = response[13] | (response[12] << 8);
    uint16_t pm25_1 = response[15] | (response[14] << 8);
    uint16_t pm10_1 = response[17] | (response[16] << 8);
    
    float humidity_1 = hum_raw_1 / 10.0;
    float temperature_1 = (temp_raw_1 - 500) / 10.0;
    
    Serial.println("  CO2: " + String(co2_1) + " ppm");
    Serial.println("  VOC: " + String(voc_1) + " level");
    Serial.println("  Humidity: " + String(humidity_1, 1) + " %");
    Serial.println("  Temperature: " + String(temperature_1, 1) + " °C");
    Serial.println("  PM1.0: " + String(pm1_1) + " ug/m³");
    Serial.println("  PM2.5: " + String(pm25_1) + " ug/m³");
    Serial.println("  PM10: " + String(pm10_1) + " ug/m³");
    
    // 방법 2: 단일 바이트 값들
    Serial.println("\nMethod 2 (Single bytes):");
    uint16_t co2_2 = response[4];
    uint16_t voc_2 = response[6];
    uint16_t hum_2 = response[8];
    uint16_t temp_2 = response[10];
    uint16_t pm1_2 = response[12];
    uint16_t pm25_2 = response[14];
    uint16_t pm10_2 = response[16];
    
    Serial.println("  CO2: " + String(co2_2) + " ppm");
    Serial.println("  VOC: " + String(voc_2) + " level");
    Serial.println("  Humidity: " + String(hum_2) + " %");
    Serial.println("  Temperature: " + String(temp_2 - 50) + " °C");
    Serial.println("  PM1.0: " + String(pm1_2) + " ug/m³");
    Serial.println("  PM2.5: " + String(pm25_2) + " ug/m³");
    Serial.println("  PM10: " + String(pm10_2) + " ug/m³");
    
    // 방법 3: 다른 오프셋으로 시도
    Serial.println("\nMethod 3 (Different offset):");
    uint16_t co2_3 = (response[3] << 8) | response[4];
    uint16_t voc_3 = response[5];
    uint16_t hum_raw_3 = (response[7] << 8) | response[8];
    uint16_t temp_raw_3 = (response[9] << 8) | response[10];
    uint16_t pm1_3 = (response[11] << 8) | response[12];
    uint16_t pm25_3 = (response[13] << 8) | response[14];
    uint16_t pm10_3 = (response[15] << 8) | response[16];
    
    float humidity_3 = hum_raw_3 / 10.0;
    float temperature_3 = (temp_raw_3 - 500) / 10.0;
    
    Serial.println("  CO2: " + String(co2_3) + " ppm");
    Serial.println("  VOC: " + String(voc_3) + " level");
    Serial.println("  Humidity: " + String(humidity_3, 1) + " %");
    Serial.println("  Temperature: " + String(temperature_3, 1) + " °C");
    Serial.println("  PM1.0: " + String(pm1_3) + " ug/m³");
    Serial.println("  PM2.5: " + String(pm25_3) + " ug/m³");
    Serial.println("  PM10: " + String(pm10_3) + " ug/m³");
    
    // 유효성 체크
    Serial.println("\n--- Validity Check ---");
    if (co2_1 <= 5000 && humidity_1 >= 0 && humidity_1 <= 100 && temperature_1 >= -40 && temperature_1 <= 85) {
      Serial.println("Method 1: VALID");
    }
    if (co2_2 <= 5000 && hum_2 >= 0 && hum_2 <= 100 && (temp_2-50) >= -40 && (temp_2-50) <= 85) {
      Serial.println("Method 2: VALID");
    }
    if (co2_3 <= 5000 && humidity_3 >= 0 && humidity_3 <= 100 && temperature_3 >= -40 && temperature_3 <= 85) {
      Serial.println("Method 3: VALID");
    }
    
  } else {
    Serial.println("Not enough data received");
  }
  
  delay(5000);
}