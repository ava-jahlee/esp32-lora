#include "config.h" // config.h 파일에 LoRaWAN 설정 및 라디오/노드 객체 정의가 있음

#include <Wire.h>
#include <ArduinoJson.h>  // getDeviceID 함수를 위해 추가
#include <LittleFS.h>     // getDeviceID 함수를 위해 추가
#include "esp_sleep.h" // ESP32 딥 슬립 관련 헤더 파일
#include "driver/rtc_io.h" // RTC GPIO 제어를 위한 헤더 파일
#include <esp_system.h> // ESP.getEfuseMac() 사용을 위해 추가

// OLED 디스플레이 라이브러리 (Heltec 대신 직접 SSD1306 사용)
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// VEXT 및 ADC_BAT 핀 정의 (Heltec V3 핀맵 기준)
#define VEXT     36 // Heltec V3의 외부 전원 제어 핀 (VEXT_EN)
#define ADC_BAT  1  // Heltec V3의 배터리 ADC 입력 핀

// OLED 디스플레이 설정
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET 21
#define OLED_ADDRESS 0x3C

// OLED I2C 핀 설정 (GPIO17, 18)
#define OLED_SDA_PIN 17
#define OLED_SCL_PIN 18

// AM1008W-K-P UART 핀 설정 (Heltec V3 사용 가능한 GPIO)
#define AM1008_RX_PIN 47  // GPIO47 (RX) - AM1008W-K-P TX에 연결
#define AM1008_TX_PIN 48  // GPIO48 (TX) - AM1008W-K-P RX에 연결

// 재연결 관련 설정
#define MAX_REJOIN_ATTEMPTS 3        // 최대 재조인 시도 횟수
#define MAX_SEND_FAILURES 5         // 연속 전송 실패 허용 횟수
#define REJOIN_DELAY_MS 30000       // 재조인 시도 간격 (30초)

String device_id = "";  // Device ID 변수 추가

// 8x8 픽셀 아이콘 정의 (이모지 스타일로 예쁘게)
const unsigned char PROGMEM icon_temp[] = {
  0x10, 0x28, 0x28, 0x28, 0x28, 0x6C, 0x6C, 0x38  // 🌡️ 온도계 (더 둥글고 예쁘게)
};

const unsigned char PROGMEM icon_humidity[] = {
  0x10, 0x38, 0x7C, 0x7C, 0xFE, 0xFE, 0x7C, 0x38  // 💧 물방울 (더 통통하게)
};

const unsigned char PROGMEM icon_co2[] = {
  0x00, 0x3C, 0x42, 0x99, 0x99, 0x42, 0x3C, 0x00  // 🌫️ CO2 (구름 모양)
};

const unsigned char PROGMEM icon_pm[] = {
  0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA  // 🌪️ 미세먼지 (점점이)
};

const unsigned char PROGMEM icon_lora[] = {
  0x10, 0x38, 0x54, 0x92, 0x10, 0x10, 0x10, 0x7C  // 안테나 + 받침
};

const unsigned char PROGMEM icon_paw[] = {
  0x60, 0x90, 0x90, 0x60, 0x00, 0x66, 0x99, 0x66  // 🐾 발자국
};

// AM1008W-K-P 센서 데이터 구조체
struct AM1008Data {
  float temperature;
  float humidity;
  uint16_t co2;
  uint8_t voc_level;
  uint16_t pm1_0;
  uint16_t pm2_5;
  uint16_t pm10;
  bool valid;
};

// 전체 센서 데이터 구조체
struct SensorData {
  AM1008Data am1008;
  
  // 센서 상태
  bool am1008_available;
};

// 연결 상태 enum
enum LoRaWANStatus {
  LORAWAN_DISCONNECTED,
  LORAWAN_CONNECTING,
  LORAWAN_CONNECTED,
  LORAWAN_SEND_FAILED,
  LORAWAN_REJOIN_NEEDED
};

// AM1008W-K-P 센서 객체만 생성
HardwareSerial am1008Serial(1); // UART1 사용 (AM1008W-K-P용)

// OLED 객체 생성
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);

// 상태 변수들 (AM1008W-K-P + OLED)
bool am1008_available = false;
bool oled_available = false;
uint8_t consecutive_send_failures = 0;
uint32_t last_successful_send = 0;
uint32_t last_rejoin_attempt = 0;
LoRaWANStatus lorawan_status = LORAWAN_DISCONNECTED;

// Device ID 가져오기 함수
String getDeviceID() {
  // LittleFS 시작
  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS 시작 실패. 기본 DeviceID 사용");
    return "LoRa-XXX";
  }

  File file = LittleFS.open("/device_registry.json", "r");
  if (!file) {
    Serial.println("device_registry.json 파일 열기 실패");
    LittleFS.end(); // 메모리 누수 방지
    return "LoRa-XXX";
  }

  uint64_t chipid = ESP.getEfuseMac();
  String chipidStr = String((uint32_t)(chipid >> 32), HEX) + String((uint32_t)chipid, HEX);
  chipidStr.toUpperCase();
  
  Serial.println("Chip ID: " + chipidStr);

  // ArduinoJson 7.x 사용
  JsonDocument doc;
  
  DeserializationError error = deserializeJson(doc, file);
  file.close();
  LittleFS.end(); // 메모리 누수 방지

  if (error) {
    Serial.println("JSON 파싱 오류: " + String(error.c_str()));
    return "LoRa-XXX";
  }

  if (doc[chipidStr].is<String>()) {
    String id = doc[chipidStr].as<String>();
    return id;
  } else {
    Serial.println("등록되지 않은 MAC 주소");
    return "LoRa-XXX";
  }
}

// Light Sleep 함수
void enterLightSleep(uint32_t sleepTimeSeconds) {
  Serial.println("Entering light sleep for " + String(sleepTimeSeconds) + " seconds...");
  Serial.flush();
  
  // 화면 끄기 (전력 절약)
  if (oled_available) {
    display.clearDisplay();
    display.display();
  }
  
  // Light sleep 설정 (RAM 메모리 유지 - JOIN 상태 보존)
  esp_sleep_enable_timer_wakeup(sleepTimeSeconds * 1000000ULL);
  esp_light_sleep_start();
  
  Serial.println("Woke up from light sleep - LoRaWAN session preserved!");
}

// 개선된 OLED 업데이트 함수
void updateDisplay(SensorData data, LoRaWANStatus status) {
  if (!oled_available) return;
  
  display.clearDisplay();
  
  // 제목 (기본 폰트 크기)
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(" LoRa:AM1008W ");
  
  // LoRa 상태
  display.setCursor(80, 0);
  switch(status) {
    case LORAWAN_CONNECTED:
      display.println("OK");
      break;
    case LORAWAN_CONNECTING:
      display.println("JOINING...");
      break;
    case LORAWAN_SEND_FAILED:
      display.print("FAIL(");
      display.print(consecutive_send_failures);
      display.println(")");
      break;
    case LORAWAN_REJOIN_NEEDED:
      display.println("REJOINING...");
      break;
    default:
      display.println("DISCONNECTED");
      break;
  }

  // 구분선
  display.drawLine(0, 12, 128, 12, SSD1306_WHITE);
  
  // LoRa device_id
  display.drawBitmap(0, 16, icon_lora, 8, 8, SSD1306_WHITE);
  display.setCursor(12, 16);
  display.println(device_id);

  // AM1008W-K-P 센서 데이터 표시
  if (data.am1008_available && data.am1008.valid) {
    // 온도
    display.drawBitmap(0, 26, icon_temp, 8, 8, SSD1306_WHITE);
    display.setCursor(12, 26);
    display.print("Temp: ");
    if (isnan(data.am1008.temperature)) {
      display.println("N/A");
    } else {
      display.print(data.am1008.temperature, 1);
      display.println(" C");
    }
    
    // 습도
    display.drawBitmap(0, 36, icon_humidity, 8, 8, SSD1306_WHITE);
    display.setCursor(12, 36);
    display.print("Humi: ");
    if (isnan(data.am1008.humidity)) {
      display.println("N/A");
    } else {
      display.print(data.am1008.humidity, 1);
      display.println(" %");
    }
    
    // CO2
    display.drawBitmap(0, 46, icon_co2, 8, 8, SSD1306_WHITE);
    display.setCursor(12, 46);
    display.print("CO2: ");
    display.print(data.am1008.co2);
    display.println(" ppm");
    
    // PM2.5
    display.drawBitmap(0, 56, icon_pm, 8, 8, SSD1306_WHITE);
    display.setCursor(12, 56);
    display.print("PM2.5: ");
    display.print(data.am1008.pm2_5);
    display.println(" ug/m3");
  } else {
    display.setCursor(12, 26);
    display.println("AM1008W-K-P");
    display.setCursor(12, 36);
    display.println("Sensor Error");
    display.setCursor(12, 46);
    display.println("Check Connection");
  }
  
  display.display();
}

// 초기화 화면 (아이콘 포함)
void displayInitScreen(String message) {
  if (!oled_available) return;
  
  display.clearDisplay();
  
  // 제목 (기본 폰트)
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("LoRa-Stair Sensors");
  
  // 구분선
  display.drawLine(0, 12, 128, 12, SSD1306_WHITE);
  
  // 상태 메시지
  display.setCursor(0, 20);
  display.println("Initializing...");
  display.setCursor(0, 35);
  display.println(message);
  
  display.display();
}

// 라디오 하드웨어 완전 재초기화
bool resetRadioHardware() {
  Serial.println("=== RADIO HARDWARE RESET ===");
  
  // SPI 정지
  SPI.end();
  delay(100);
  
  // LoRa 모듈 하드웨어 리셋
  pinMode(12, OUTPUT); // RST 핀
  digitalWrite(12, LOW);
  delay(200);
  digitalWrite(12, HIGH);
  delay(200);
  
  // SPI 재시작
  SPI.begin(9, 11, 10, 8); // SCK, MISO, MOSI, SS
  delay(100);
  
  // 라디오 객체 재생성
  radio.reset();
  delay(100);
  
  // 라디오 재초기화
  int16_t radioState = radio.begin();
  if (radioState != RADIOLIB_ERR_NONE) {
    Serial.println("✗ Radio hardware reset failed: " + stateDecode(radioState));
    return false;
  }
  
  Serial.println("✓ Radio hardware reset successful");
  return true;
}

// LoRaWAN 강제 재조인 함수
bool forceRejoin() {
  Serial.println("=== FORCE REJOIN ATTEMPT ===");
  
  // 라디오 재초기화
  Serial.println("Reinitializing radio...");
  int16_t radioState = radio.begin();
  if (radioState != RADIOLIB_ERR_NONE) {
    Serial.println("Radio reinitialization failed: " + stateDecode(radioState));
    return false;
  }
  
  // 노드 재초기화
  Serial.println("Reinitializing LoRaWAN node...");
  node.beginOTAA(joinEUI, devEUI, nwkKey, appKey);
  // beginOTAA는 RadioLib 6.x에서 void를 반환하므로 오류 체크 불가
  
  // 새로운 조인 시도
  Serial.println("Attempting fresh OTAA join...");
  int16_t joinState = node.activateOTAA();
  
  if (joinState == RADIOLIB_LORAWAN_NEW_SESSION) {
    Serial.println("✓ Successfully rejoined LoRaWAN network!");
    consecutive_send_failures = 0;
    last_successful_send = millis();
    return true;
  } else {
    Serial.println("✗ Rejoin failed: " + stateDecode(joinState));
    return false;
  }
}

// 스마트 재연결 함수
bool smartReconnect() {
  uint32_t currentTime = millis();
  
  // 너무 자주 재조인 시도하지 않도록 제한
  if (currentTime - last_rejoin_attempt < REJOIN_DELAY_MS) {
    Serial.println("Rejoin cooldown active, skipping...");
    return false;
  }
  
  last_rejoin_attempt = currentTime;
  lorawan_status = LORAWAN_CONNECTING;
  
  // CHIP_NOT_FOUND 에러가 지속되면 하드웨어 리셋부터 시도
  if (consecutive_send_failures >= 2) {
    Serial.println("Multiple CHIP_NOT_FOUND errors detected. Resetting hardware...");
    if (!resetRadioHardware()) {
      lorawan_status = LORAWAN_DISCONNECTED;
      return false;
    }
  }
  
  // 먼저 세션 복원 시도
  if (!node.isActivated()) {
    Serial.println("Session not active. Attempting session restore...");
    node.beginOTAA(joinEUI, devEUI, nwkKey, appKey);
    // beginOTAA는 RadioLib 6.x에서 void를 반환하므로 세션 복원 상태 확인 불가
    
    // 활성화 상태 직접 확인
    if (node.isActivated()) {
      Serial.println("✓ Session restored or new session created!");
      consecutive_send_failures = 0;
      last_successful_send = millis();
      lorawan_status = LORAWAN_CONNECTED;
      return true;
    }
  }
  
  // 세션 복원 실패 시 강제 재조인
  Serial.println("Session restore failed. Attempting force rejoin...");
  lorawan_status = LORAWAN_REJOIN_NEEDED;
  
  for (int attempt = 1; attempt <= MAX_REJOIN_ATTEMPTS; attempt++) {
    Serial.println("Rejoin attempt " + String(attempt) + "/" + String(MAX_REJOIN_ATTEMPTS));
    
    if (forceRejoin()) {
      lorawan_status = LORAWAN_CONNECTED;
      return true;
    }
    
    if (attempt < MAX_REJOIN_ATTEMPTS) {
      Serial.println("Waiting before next attempt...");
      delay(10000); // 10초 대기
    }
  }  
  Serial.println("✗ All rejoin attempts failed!");
  lorawan_status = LORAWAN_DISCONNECTED;

  // ==== 중요 추가: 모든 재연결 시도가 실패했을 때 시스템 재부팅 ====
  Serial.println("CRITICAL: All rejoin attempts failed! Initiating system restart...");
  Serial.flush(); // 시리얼 메시지가 모두 전송되도록 합니다.
  ESP.restart(); // ESP32를 소프트웨어적으로 재부팅합니다.
  // ==================================================================

  return false;
}

// AM1008W-K-P 데이터 읽기 함수 (명령-응답 방식)
AM1008Data readAM1008Data() {
  AM1008Data data;
  // 기본값을 NaN으로 설정
  data.temperature = NAN;
  data.humidity = NAN;
  data.co2 = 0;
  data.voc_level = 0;
  data.pm1_0 = 0;
  data.pm2_5 = 0;
  data.pm10 = 0;
  data.valid = false;
  
  const int EXPECTED_RESPONSE_LENGTH = 25;
  byte response_buffer[EXPECTED_RESPONSE_LENGTH] = {0};
  byte read_measurement_cmd[] = {0x11, 0x02, 0x01, 0x01, 0xEB};
  
  // 이전 데이터 비우기
  while(am1008Serial.available()) {
    am1008Serial.read();
  }
  
  // 명령 전송
  Serial.print("Sending command: ");
  for(int i = 0; i < sizeof(read_measurement_cmd); i++) {
    Serial.print("0x");
    if(read_measurement_cmd[i] < 16) Serial.print("0");
    Serial.print(read_measurement_cmd[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  
  am1008Serial.write(read_measurement_cmd, sizeof(read_measurement_cmd));
  delay(200);
  
  // 응답 대기
  unsigned long startTime = millis();
  const unsigned long timeoutDuration = 1000;
  
  while (am1008Serial.available() < EXPECTED_RESPONSE_LENGTH) {
    if (millis() - startTime > timeoutDuration) {
      Serial.print("Timeout! Available bytes: ");
      Serial.println(am1008Serial.available());
      return data; // 타임아웃, NaN 값들 반환
    }
    delay(10);
  }
  
  if (am1008Serial.available() >= EXPECTED_RESPONSE_LENGTH) {
    am1008Serial.readBytes(response_buffer, EXPECTED_RESPONSE_LENGTH);
    
    Serial.print("Received response: ");
    for(int i = 0; i < EXPECTED_RESPONSE_LENGTH; i++) {
      Serial.print("0x");
      if(response_buffer[i] < 16) Serial.print("0");
      Serial.print(response_buffer[i], HEX);
      Serial.print(" ");
      if((i + 1) % 8 == 0) Serial.println(); // 8바이트마다 줄바꿈
    }
    Serial.println();
    
    // 응답 헤더 확인: 16 16 01
    if (response_buffer[0] == 0x16 && response_buffer[1] == 0x16 && response_buffer[2] == 0x01) {
      Serial.println("✓ Valid AM1008W-K-P response detected");
      
      // 데이터시트에 따른 정확한 파싱
      // CO2: [DF1][DF2] (0~5,000 ppm)
      data.co2 = (response_buffer[3] << 8) | response_buffer[4];
      
      // VOC: [DF3][DF4] (0~3 level)
      data.voc_level = (response_buffer[5] << 8) | response_buffer[6];
      
      // 습도: [DF5][DF6] ÷ 10 (5.0~99.0%)
      uint16_t humidity_raw = (response_buffer[7] << 8) | response_buffer[8];
      data.humidity = humidity_raw / 10.0;
      
      // 온도: (DF7 * 256 + DF8 - 500) / 10 (데이터시트 공식)
      uint16_t temp_raw = (response_buffer[9] << 8) | response_buffer[10];
      data.temperature = (temp_raw - 500) / 10.0;
      
      // PM1.0: [DF9][DF10] (0~1,000 ug/m³)
      data.pm1_0 = (response_buffer[11] << 8) | response_buffer[12];
      
      // PM2.5: [DF11][DF12] (0~1,000 ug/m³)
      data.pm2_5 = (response_buffer[13] << 8) | response_buffer[14];
      
      // PM10: [DF13][DF14] (0~1,000 ug/m³)
      data.pm10 = (response_buffer[15] << 8) | response_buffer[16];
      
      data.valid = true;
      
      Serial.println("Parsed data:");
      Serial.println("  CO2: " + String(data.co2) + " ppm");
      Serial.println("  VOC: " + String(data.voc_level) + " level");
      Serial.println("  Humidity: " + String(data.humidity, 1) + " %");
      Serial.println("  Temperature: " + String(data.temperature, 1) + " °C");
      Serial.println("  PM1.0: " + String(data.pm1_0) + " ug/m³");
      Serial.println("  PM2.5: " + String(data.pm2_5) + " ug/m³");
      Serial.println("  PM10: " + String(data.pm10) + " ug/m³");
    } else {
      Serial.println("✗ Invalid response header");
      Serial.print("Expected: 0x16 0x16 0x01, Got: ");
      Serial.print("0x"); if(response_buffer[0] < 16) Serial.print("0"); Serial.print(response_buffer[0], HEX);
      Serial.print(" 0x"); if(response_buffer[1] < 16) Serial.print("0"); Serial.print(response_buffer[1], HEX);
      Serial.print(" 0x"); if(response_buffer[2] < 16) Serial.print("0"); Serial.println(response_buffer[2], HEX);
    }
  }
  
  return data;
}

// 센서 데이터 읽기 함수 (AM1008W-K-P 전용)
SensorData readSensors() {
  SensorData data;
  
  // AM1008W-K-P 센서 상태 설정
  data.am1008_available = am1008_available;
  
  // AM1008W-K-P 데이터 읽기
  if (am1008_available) {
    data.am1008 = readAM1008Data();
  } else {
    // AM1008W-K-P 센서 없으면 기본값
    data.am1008.temperature = NAN;
    data.am1008.humidity = NAN;
    data.am1008.co2 = 0;
    data.am1008.voc_level = 0;
    data.am1008.pm1_0 = 0;
    data.am1008.pm2_5 = 0;
    data.am1008.pm10 = 0;
    data.am1008.valid = false;
  }
  
  return data;
}

// 센서 데이터를 바이트 배열로 변환
void encodeSensorData(SensorData data, uint8_t* buffer) {
  // 16바이트 패킷 구조:
  // [0-1]: 온도 (AM1008W) - NaN이면 0xFFFF
  // [2-3]: 습도 (AM1008W) - NaN이면 0xFFFF  
  // [4-5]: CO2 (AM1008W) - ppm
  // [6-7]: PM2.5 (AM1008W) - ug/m³
  // [8-9]: PM10 (AM1008W) - ug/m³
  // [10-11]: PM1.0 (AM1008W) - ug/m³
  // [12]: VOC Level (AM1008W) - 0~3
  // [13]: 센서 상태 플래그
  // [14]: 연속 실패 횟수
  // [15]: 예약/체크섬
  
  // AM1008W 데이터 (NaN이면 특별값으로 설정)
  uint16_t temp_am = 0xFFFF, hum_am = 0xFFFF, co2_am = 0, pm1_am = 0, pm25_am = 0, pm10_am = 0;
  uint8_t voc_am = 0;
  
  if (data.am1008_available && data.am1008.valid) {
    // 온도 처리 (NaN 체크)
    if (!isnan(data.am1008.temperature)) {
    temp_am = (uint16_t)((data.am1008.temperature + 40) * 10);
    }
    
    // 습도 처리 (NaN 체크)
    if (!isnan(data.am1008.humidity)) {
    hum_am = (uint16_t)(data.am1008.humidity * 10);
    }
    
    // 다른 데이터들
    co2_am = data.am1008.co2;
    pm1_am = data.am1008.pm1_0;
    pm25_am = data.am1008.pm2_5;
    pm10_am = data.am1008.pm10;
    voc_am = data.am1008.voc_level;
  }
  
  // 센서 상태 플래그 (비트마스크)
  uint8_t sensor_status = 0;
  if (data.am1008_available) sensor_status |= 0x01;
  if (data.am1008.valid) sensor_status |= 0x02;
  
  // 패킷 구성
  buffer[0] = temp_am >> 8;           // AM1008W 온도 상위
  buffer[1] = temp_am & 0xFF;         // AM1008W 온도 하위
  buffer[2] = hum_am >> 8;            // AM1008W 습도 상위
  buffer[3] = hum_am & 0xFF;          // AM1008W 습도 하위
  buffer[4] = co2_am >> 8;            // AM1008W CO2 상위
  buffer[5] = co2_am & 0xFF;          // AM1008W CO2 하위
  buffer[6] = pm25_am >> 8;           // AM1008W PM2.5 상위
  buffer[7] = pm25_am & 0xFF;         // AM1008W PM2.5 하위
  buffer[8] = pm10_am >> 8;           // AM1008W PM10 상위
  buffer[9] = pm10_am & 0xFF;         // AM1008W PM10 하위
  buffer[10] = pm1_am >> 8;           // AM1008W PM1.0 상위
  buffer[11] = pm1_am & 0xFF;         // AM1008W PM1.0 하위
  buffer[12] = voc_am;                // AM1008W VOC Level
  buffer[13] = sensor_status;         // 센서 상태 플래그
  buffer[14] = consecutive_send_failures; // 연속 실패 횟수
  buffer[15] = 0x00;                  // 예약/체크섬
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n=== LoRaWAN + Sensors Initializing ===");
  
  // Device ID 가져오기
  device_id = getDeviceID(); 
  Serial.print("Device ID: ");
  Serial.println(device_id);

  // Vext 핀 제어 (GPIO36) - OLED 전원 활성화
  pinMode(VEXT, OUTPUT);
  digitalWrite(VEXT, LOW); // LOW = 전원 ON (Heltec 보드 특성)
  delay(100);
  Serial.println("Vext (OLED power) enabled");

  // OLED RST 핀 설정 (GPIO21)
  pinMode(21, OUTPUT);
  digitalWrite(21, LOW);
  delay(10);
  digitalWrite(21, HIGH);
  delay(100);
  Serial.println("OLED reset completed");

  // OLED용 I2C 초기화 (GPIO17, 18) - Wire1 사용
  Wire1.begin(OLED_SDA_PIN, OLED_SCL_PIN);
  Wire1.setClock(100000); // I2C 클럭 속도 낮춤
  delay(100);
  
  // OLED 초기화 시도
  Serial.println("Attempting OLED initialization...");
  if (display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS, false, false)) {
    oled_available = true;
    Serial.println("OLED display initialized successfully!");
    
    // 예쁜 시작 화면 테스트
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("HELLO!");
    display.setTextSize(1);
    display.setCursor(0, 20);
    display.println("I'm " + device_id + "!");
    display.setCursor(0, 30);
    // 아이콘 미리보기
    display.drawBitmap(50, 45, icon_paw, 8, 8, SSD1306_WHITE);
    display.display();
    delay(3000);
    Serial.println("OLED test screen displayed");
    
    displayInitScreen("Starting...");
    delay(1000);
  } else {
    oled_available = false;
    Serial.println("OLED display initialization failed - continuing without display");
  }
  
  // AM1008W-K-P 초기화 (필수)
  Serial.println("Attempting AM1008W-K-P initialization...");
  displayInitScreen("Init AM1008W-K-P...");
  
  am1008Serial.begin(9600, SERIAL_8N1, AM1008_RX_PIN, AM1008_TX_PIN);
  delay(1000); // 센서 안정화 대기
  
  // AM1008W-K-P 테스트 (3번 시도)
  AM1008Data testData = {0};
  for (int attempt = 1; attempt <= 3; attempt++) {
    Serial.println("AM1008W-K-P test attempt " + String(attempt) + "/3");
    testData = readAM1008Data();
    if (testData.valid) {
      Serial.println("✓ AM1008W-K-P sensor detected and working!");
      am1008_available = true;
      displayInitScreen("AM1008W-K-P OK");
      break;
    } else {
      Serial.println("✗ AM1008W-K-P test failed on attempt " + String(attempt));
      if (attempt < 3) {
        delay(1000);
      }
    }
  }
  
  if (!am1008_available) {
    Serial.println("WARNING: AM1008W-K-P sensor initialization failed!");
    Serial.println("Check connections:");
    Serial.println("- AM1008W-K-P TX -> GPIO47 (ESP32 RX)");
    Serial.println("- AM1008W-K-P RX -> GPIO48 (ESP32 TX)");
    Serial.println("- AM1008W-K-P VCC -> 5V");
    Serial.println("- AM1008W-K-P GND -> GND");
    Serial.println("Continuing without sensor for debugging...");
    displayInitScreen("AM1008W-K-P FAIL!");
    delay(2000);
    // 디버깅을 위해 재시작하지 않고 계속 진행
  }
  
  delay(1000);

  // LoRaWAN 초기화 시작
  displayInitScreen("Init LoRa radio...");
  
  // SPI 핀 명시적 재설정 (딥슬립 후 복구)
  SPI.begin(9, 11, 10, 8); // SCK, MISO, MOSI, SS for Heltec V3
  delay(100);
  
  // LoRa 모듈 전원 확인 및 리셋
  pinMode(12, OUTPUT); // RST 핀
  digitalWrite(12, LOW);
  delay(100);
  digitalWrite(12, HIGH);
  delay(100);
  Serial.println("LoRa module reset completed");
  
  // LoRaWAN 초기화 (config.h에서 정의된 radio 객체 사용)
  Serial.println("Initialise the radio");
  int16_t state = radio.begin();
  debug(state != RADIOLIB_ERR_NONE, F("Initialise radio failed"), state, true);

  displayInitScreen("Init LoRaWAN node...");
  
  // LoRaWAN 노드 설정
  node.beginOTAA(joinEUI, devEUI, nwkKey, appKey);
  // beginOTAA는 이제 void를 반환하므로 debug 체크 불필요

  // LoRaWAN 네트워크 조인 (첫 부팅 시에만)
  Serial.println("Join ('login') the LoRaWAN Network");
  displayInitScreen("Joining LoRaWAN...");
  
  state = node.activateOTAA(); 
  debug(state != RADIOLIB_LORAWAN_NEW_SESSION, F("Join failed"), state, true);

  Serial.println("Ready! LoRaWAN Network Joined Successfully!");
  Serial.println("Sensors + LoRaWAN initialized successfully!");
  
  // 초기 연결 성공
  lorawan_status = LORAWAN_CONNECTED;
  consecutive_send_failures = 0;
  last_successful_send = millis();
  
  displayInitScreen("LoRaWAN Joined!");
  delay(2000);
}

void loop() {
  uint32_t currentTime = millis();
  
  // 센서 데이터 읽기
  SensorData sensorData = readSensors();
  
  // 연결 상태 확인 및 재연결 시도
  if (!node.isActivated() || consecutive_send_failures >= MAX_SEND_FAILURES) {
    Serial.println("=== CONNECTION ISSUE DETECTED ===");
    Serial.println("Activated: " + String(node.isActivated()));
    Serial.println("Consecutive failures: " + String(consecutive_send_failures));
    
    // 스마트 재연결 시도
    if (smartReconnect()) {
      Serial.println("✓ Reconnection successful!");
      lorawan_status = LORAWAN_CONNECTED;
    } else {
      Serial.println("✗ Reconnection failed!");
      lorawan_status = LORAWAN_DISCONNECTED;
    }
  } else {
    lorawan_status = LORAWAN_CONNECTED;
  }
  
  // 개선된 OLED 디스플레이 업데이트 (Device ID + 아이콘)
  updateDisplay(sensorData, lorawan_status);
  
  // 시리얼로 AM1008W-K-P 센서 데이터 출력
  Serial.println("=== AM1008W-K-P Sensor Data ===");
  Serial.println("Device ID: " + device_id);
  
  // AM1008W-K-P 데이터 출력 (NaN 처리 포함)
  if (sensorData.am1008_available && sensorData.am1008.valid) {
    Serial.print("AM1008W-K-P - Temp: ");
    if (isnan(sensorData.am1008.temperature)) {
      Serial.print("N/A");
    } else {
      Serial.print(String(sensorData.am1008.temperature, 1) + "°C");
    }
    
    Serial.print(", Humi: ");
    if (isnan(sensorData.am1008.humidity)) {
      Serial.print("N/A");
    } else {
      Serial.print(String(sensorData.am1008.humidity, 1) + "%");
    }
    
    Serial.println(", CO2: " + String(sensorData.am1008.co2) + "ppm");
    Serial.println("         VOC: " + String(sensorData.am1008.voc_level) + " level");
    Serial.println("         PM1.0: " + String(sensorData.am1008.pm1_0) + "ug/m³, PM2.5: " + String(sensorData.am1008.pm2_5) + "ug/m³, PM10: " + String(sensorData.am1008.pm10) + "ug/m³");
  } else {
    Serial.println("AM1008W-K-P - Sensor not available or invalid data");
  }
  
  // LoRaWAN 전송 시도 (연결된 경우에만)
  if (lorawan_status == LORAWAN_CONNECTED) {
    uint8_t uplinkPayload[16]; // AM1008W-K-P 데이터 16바이트
    encodeSensorData(sensorData, uplinkPayload);
    
    Serial.println("Sending sensor data via LoRaWAN...");
    int16_t sendState = node.sendReceive(uplinkPayload, sizeof(uplinkPayload)); 
    
    if (sendState == RADIOLIB_ERR_NONE || sendState == RADIOLIB_LORAWAN_NEW_SESSION) {
      Serial.println("✓ Data sent successfully! (State: " + stateDecode(sendState) + ")");
      consecutive_send_failures = 0;
      last_successful_send = currentTime;
      lorawan_status = LORAWAN_CONNECTED;
    } else {
      Serial.println("✗ Send failed: " + stateDecode(sendState) + " (" + String(sendState) + ")");
      consecutive_send_failures++;
      lorawan_status = LORAWAN_SEND_FAILED;
      
      Serial.println("Consecutive failures: " + String(consecutive_send_failures) + "/" + String(MAX_SEND_FAILURES));
      
      // 즉시 재연결 시도 (특정 에러의 경우)
      if (sendState == RADIOLIB_ERR_NETWORK_NOT_JOINED || 
          sendState == RADIOLIB_ERR_JOIN_NONCE_INVALID ||
          sendState == RADIOLIB_ERR_CHIP_NOT_FOUND) { // JOIN_NONCE_INVALID로 대체
        Serial.println("Critical network/hardware error detected. Attempting immediate reconnection...");
        smartReconnect();
      }
    }
  } else {
    Serial.println("⚠ LoRaWAN not connected - skipping data transmission");
  }

  // 전송 결과를 반영하여 디스플레이 다시 업데이트
  updateDisplay(sensorData, lorawan_status);

  // 통계 정보 출력
  Serial.println("=== Connection Stats ===");
  Serial.println("Status: " + String(lorawan_status));
  Serial.println("Consecutive failures: " + String(consecutive_send_failures));
  Serial.println("Last successful send: " + String((currentTime - last_successful_send) / 1000) + "s ago");
  Serial.println("Next transmission in " + String(uplinkIntervalSeconds) + " seconds");
  Serial.println("========================");

// 화면 표시 시간 (5초간 켜두기)
Serial.println("Display will stay on for 5 seconds...");
delay(5000); // 5초간 화면 유지

// 화면 끄기 (전력 절약)
if (oled_available) {
  display.clearDisplay();
  display.display();
  Serial.println("Display turned off for power saving");
}

// Light Sleep으로 전환 (메모리 유지 = 재JOIN 방지)
enterLightSleep(uplinkIntervalSeconds - 5);
  
  // 이제 루프가 다시 시작되지만 LoRaWAN 세션이 유지됨!
}