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
#define OLED_RESET 21  // GPIO21 (OLED RST) - 원래대로 복원
#define OLED_ADDRESS 0x3C

// OLED I2C 핀 설정 (내장 OLED - GPIO17, 18 사용)
#define OLED_SDA_PIN 17
#define OLED_SCL_PIN 18

// AM1008W-K-P I2C 핀 설정 (별도 I2C 버스)
#define AM1008_SDA_PIN 41  // GPIO41 (SDA) - AM1008W-K-P SDA에 연결
#define AM1008_SCL_PIN 42  // GPIO42 (SCL) - AM1008W-K-P SCL에 연결
// AM1008_I2C_ADDR 제거 - 동적 감지 시스템 사용

// 재연결 관련 설정
#define MAX_REJOIN_ATTEMPTS 3        // 최대 재조인 시도 횟수
#define MAX_SEND_FAILURES 5         // 연속 전송 실패 허용 횟수
#define REJOIN_DELAY_MS 30000       // 재조인 시도 간격 (30초)

String device_id = "";  // Device ID 변수 추가
uint8_t detected_sensor_address = 0;  // 동적으로 감지된 센서 주소

// 센서 감지 정보 구조체 (메모리 최적화)
struct SensorInfo {
  uint8_t address;
  bool found;
  bool valid_data;
  const char* parsing_method;  // String → const char* (메모리 절약)
};

// 전역 I2C 버퍼 (재사용으로 메모리 효율성 증대)
static uint8_t i2c_buffer[25];

// 성능 최적화 상수
#define I2C_RESPONSE_DELAY_MS 50    // I2C 응답 대기 시간 (최적화됨)
#define I2C_ADDRESS_TEST_DELAY_MS 20 // 주소 테스트 간격 (최적화됨)
#define I2C_SCAN_DELAY_MS 5         // I2C 스캔 지연 시간 (최적화됨)

// 8x8 픽셀 아이콘 정의 (이모지 스타일로 예쁘게)
const unsigned char PROGMEM icon_temp[] = {
  0x10, 0x28, 0x28, 0x28, 0x28, 0x6C, 0x6C, 0x38  // 온도계 (더 둥글고 예쁘게)
};

const unsigned char PROGMEM icon_humidity[] = {
  0x10, 0x38, 0x7C, 0x7C, 0xFE, 0xFE, 0x7C, 0x38  // 물방울 (더 통통하게)
};

const unsigned char PROGMEM icon_co2[] = {
  0x00, 0x3C, 0x42, 0x99, 0x99, 0x42, 0x3C, 0x00  // CO2 (구름 모양)
};

const unsigned char PROGMEM icon_pm[] = {
  0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA  // 미세먼지 (점점이)
};

const unsigned char PROGMEM icon_lora[] = {
  0x10, 0x38, 0x54, 0x92, 0x10, 0x10, 0x10, 0x7C  // 안테나 + 받침
};

const unsigned char PROGMEM icon_paw[] = {
  0x60, 0x90, 0x90, 0x60, 0x00, 0x66, 0x99, 0x66  // 발자국
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

// OLED 객체 생성
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);

// 상태 변수들 (AM1008W-K-P + OLED)
bool am1008_available = false;
bool oled_available = false;
uint8_t consecutive_send_failures = 0;
uint32_t last_successful_send = 0;
uint32_t last_rejoin_attempt = 0;
LoRaWANStatus lorawan_status = LORAWAN_DISCONNECTED;

// 하드웨어 디버깅 함수
void detailedHardwareTest() {
  Serial.println("=== Detailed Hardware Test ===");
  
  // GPIO 상태 확인
  Serial.println("GPIO States:");
  Serial.printf("GPIO41 (SDA): %d\n", digitalRead(41));
  Serial.printf("GPIO42 (SCL): %d\n", digitalRead(42));
  
  // I2C 클럭 속도를 더 낮춤
  Wire.setClock(1000); // 1kHz
  delay(100);
  
  // 여러 주소에서 응답 테스트
  Serial.println("Testing I2C addresses:");
  for (uint8_t addr = 0x20; addr <= 0x30; addr++) {
    Wire.beginTransmission(addr);
    uint8_t error = Wire.endTransmission();
    if (error == 0) {
      Serial.printf("0x%02X: %d (ACK)\n", addr, error);
    }
    delay(10);
  }
  
  // I2C 재초기화
  Wire.begin(AM1008_SDA_PIN, AM1008_SCL_PIN);
  Wire.setClock(10000); // 10kHz로 복원
}

// I2C 주소 스캔 함수 (개선된 버전)
// 센서 데이터 유효성 검사 함수
bool testDataValidity(uint8_t* response, uint8_t address) {
  // 응답 길이 확인 (최소 25바이트)
  if (response == nullptr) {
    Serial.printf("Address 0x%02X: Null response\n", address);
    return false;
  }
  
  // 헤더 확인 (0x16 0x19)
  if (response[0] != 0x16 || response[1] != 0x19) {
    Serial.printf("Address 0x%02X: Invalid header (0x%02X 0x%02X), expected (0x16 0x19)\n", 
                  address, response[0], response[1]);
    return false;
  }
  
  // CO2 데이터 추출 및 검증 (바이트 3-4, 빅엔디안)
  uint16_t co2 = (response[3] << 8) | response[4];
  if (co2 > 5000) {
    Serial.printf("Address 0x%02X: Invalid CO2 value: %d ppm (> 5000)\n", address, co2);
    return false;
  }
  
  // 온도 데이터 추출 및 검증 (바이트 5-6, 빅엔디안, 0.1도 단위)
  int16_t temp_raw = (response[5] << 8) | response[6];
  float temperature = temp_raw / 10.0;
  if (temperature < -40.0 || temperature > 85.0) {
    Serial.printf("Address 0x%02X: Invalid temperature: %.1f°C (range: -40~85°C)\n", 
                  address, temperature);
    return false;
  }
  
  // 습도 데이터 추출 및 검증 (바이트 7-8, 빅엔디안, 0.1% 단위)
  uint16_t humidity_raw = (response[7] << 8) | response[8];
  float humidity = humidity_raw / 10.0;
  if (humidity < 0.0 || humidity > 100.0) {
    Serial.printf("Address 0x%02X: Invalid humidity: %.1f%% (range: 0~100%%)\n", 
                  address, humidity);
    return false;
  }
  
  Serial.printf("Address 0x%02X: Valid data - CO2: %d ppm, Temp: %.1f°C, Humidity: %.1f%%\n", 
                address, co2, temperature, humidity);
  return true;
}

// AM1008W-K-P 센서 동적 감지 함수
SensorInfo detectAM1008Sensor() {
  SensorInfo sensor_info = {0, false, false, "none"};
  
  Serial.println("=== AM1008W-K-P 센서 동적 감지 시작 ===");
  
  // 검색할 주소 범위 정의
  uint8_t address_ranges[][2] = {
    {0x28, 0x2F},  // 범위 1: 0x28~0x2F
    {0x50, 0x57},  // 범위 2: 0x50~0x57  
    {0x30, 0x37}   // 범위 3: 0x30~0x37
  };
  
  uint8_t command[] = {0x16, 0x02, 0x01, 0x01, 0xEB}; // I2C 데이터 읽기 명령
  uint8_t response[25];
  
  // 각 주소 범위에서 검색
  for (int range = 0; range < 3; range++) {
    Serial.printf("범위 %d: 0x%02X~0x%02X 검색 중...\n", 
                  range + 1, address_ranges[range][0], address_ranges[range][1]);
    
    for (uint8_t addr = address_ranges[range][0]; addr <= address_ranges[range][1]; addr++) {
      Serial.printf("주소 0x%02X 테스트 중...\n", addr);
      
      // I2C 연결 테스트
      Wire.beginTransmission(addr);
      uint8_t error = Wire.endTransmission();
      
      if (error == 0) {
        Serial.printf("주소 0x%02X: I2C 응답 있음\n", addr);
        
        // 명령 전송
        Wire.beginTransmission(addr);
        Wire.write(command, sizeof(command));
        error = Wire.endTransmission();
        
        if (error == 0) {
          delay(I2C_RESPONSE_DELAY_MS); // 응답 대기 (최적화됨)
          
          // 데이터 읽기 시도 (전역 버퍼 사용)
          Wire.requestFrom(addr, (uint8_t)25);
          if (Wire.available() >= 25) {
            // 응답 데이터 읽기 (전역 버퍼 재사용)
            for (int i = 0; i < 25; i++) {
              i2c_buffer[i] = Wire.read();
            }
            
            // 데이터 유효성 검사
            if (testDataValidity(i2c_buffer, addr)) {
              sensor_info.address = addr;
              sensor_info.found = true;
              sensor_info.valid_data = true;
              sensor_info.parsing_method = "Standard AM1008W-K-P";
              
              Serial.printf("✅ AM1008W-K-P 센서 발견! 주소: 0x%02X\n", addr);
              return sensor_info;
            }
          } else {
            Serial.printf("주소 0x%02X: 응답 데이터 부족 (%d/25 바이트)\n", 
                          addr, Wire.available());
          }
        } else {
          Serial.printf("주소 0x%02X: 명령 전송 실패 (error: %d)\n", addr, error);
        }
      }
      
      delay(I2C_ADDRESS_TEST_DELAY_MS); // 다음 주소 테스트 전 대기 (최적화됨)
    }
  }
  
  Serial.println("❌ AM1008W-K-P 센서를 찾을 수 없습니다.");
  return sensor_info;
}

void scanI2CDevices() {
  Serial.println("Scanning I2C devices (improved)...");
  byte error, address;
  int nDevices = 0;
  
  // 더 느린 클럭으로 스캔
  Wire.setClock(1000); // 1kHz
  
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println(" !");
      nDevices++;
      
      // AM1008W-K-P 주소인 경우 추가 정보
      if (address == 0x28) {
        Serial.println("  -> This is our AM1008W-K-P at 0x28!");
      }
    }
    delay(I2C_SCAN_DELAY_MS); // 각 주소 테스트 간 지연 (최적화됨)
  }
  
  // 클럭 속도 복원
  Wire.setClock(10000); // 10kHz
  
  if (nDevices == 0) {
    Serial.println("No I2C devices found!");
  } else {
    Serial.printf("Found %d device(s)\n", nDevices);
  }
}

// AM1008W-K-P 초기화 함수 (동적 감지 방식)
bool initializeAM1008PMSensor() {
  Serial.println("=== AM1008W-K-P 센서 동적 초기화 시작 ===");
  
  // 센서 동적 감지 실행
  SensorInfo sensor_info = detectAM1008Sensor();
  
  if (sensor_info.found && sensor_info.valid_data) {
    detected_sensor_address = sensor_info.address;
    Serial.printf("✅ 센서 초기화 성공!\n");
    Serial.printf("   - 주소: 0x%02X\n", detected_sensor_address);
    Serial.printf("   - 파싱 방법: %s\n", sensor_info.parsing_method);
    return true;
  } else {
    Serial.println("❌ 센서 초기화 실패: AM1008W-K-P를 찾을 수 없습니다.");
    detected_sensor_address = 0;
    return false;
  }
}

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
  display.println("LoRa - Suseo Station");
  
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
    Serial.println("Radio hardware reset failed: " + stateDecode(radioState));
    return false;
  }
  
  Serial.println("Radio hardware reset successful");
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
  
  // 새로운 조인 시도
  Serial.println("Attempting fresh OTAA join...");
  int16_t joinState = node.activateOTAA();
  
  if (joinState == RADIOLIB_LORAWAN_NEW_SESSION) {
    Serial.println("Successfully rejoined LoRaWAN network!");
    consecutive_send_failures = 0;
    last_successful_send = millis();
    return true;
  } else {
    Serial.println("Rejoin failed: " + stateDecode(joinState));
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
    
    // 활성화 상태 직접 확인
    if (node.isActivated()) {
      Serial.println("Session restored or new session created!");
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
  Serial.println("All rejoin attempts failed!");
  lorawan_status = LORAWAN_DISCONNECTED;

  // 모든 재연결 시도가 실패했을 때 시스템 재부팅
  Serial.println("CRITICAL: All rejoin attempts failed! Initiating system restart...");
  Serial.flush();
  ESP.restart();

  return false;
}

// AM1008W-K-P 데이터 읽기 함수 (0x28 주소, Method 3 파싱 사용)
AM1008Data readAM1008Data() {
  AM1008Data data;
  // 기본값 설정
  data.temperature = NAN;
  data.humidity = NAN;
  data.co2 = 0;
  data.voc_level = 0;
  data.pm1_0 = 0;
  data.pm2_5 = 0;
  data.pm10 = 0;
  data.valid = false;
  
  // 전역 버퍼 사용으로 메모리 효율성 증대
  memset(i2c_buffer, 0, 25);
  
  // 동적 감지된 센서 주소 확인
  if (detected_sensor_address == 0) {
    Serial.println("❌ 센서 주소가 감지되지 않았습니다. 초기화가 필요합니다.");
    return data;
  }
  
  Serial.printf("Reading AM1008W-K-P via I2C (0x%02X)...\n", detected_sensor_address);
  
  // I2C 읽기: 동적 감지된 주소에서 직접 25바이트 읽기
  Wire.requestFrom(detected_sensor_address, (uint8_t)25);
  
  if (Wire.available() < 25) {
    Serial.print("Not enough data received. Available: ");
    Serial.println(Wire.available());
    return data;
  }
  
  // 데이터 읽기 (전역 버퍼 사용)
  for (int i = 0; i < 25; i++) {
    i2c_buffer[i] = Wire.read();
  }
  
  Serial.print("Received I2C response: ");
  for(int i = 0; i < 25; i++) {
    Serial.print("0x");
    if(i2c_buffer[i] < 16) Serial.print("0");
    Serial.print(i2c_buffer[i], HEX);
    Serial.print(" ");
    if((i + 1) % 8 == 0) Serial.println();
  }
  Serial.println();
  
  // 응답 헤더 확인: 0x16 0x19 (25바이트 데이터)
  if (i2c_buffer[0] == 0x16 && i2c_buffer[1] == 0x19) {
    Serial.println("Valid AM1008W-K-P I2C response detected");
    
    // Method 3 파싱 (테스트에서 유효했던 방법)
    // CO2: [3][4] - Big Endian
    uint16_t co2_raw = (i2c_buffer[3] << 8) | i2c_buffer[4];
    data.co2 = co2_raw;
    
    // VOC: [5] - Single byte
    data.voc_level = i2c_buffer[5];
    
    // Humidity: [7][8] - Big Endian, /10
    uint16_t humidity_raw = (i2c_buffer[7] << 8) | i2c_buffer[8];
    data.humidity = humidity_raw / 10.0;
    
    // Temperature: [9][10] - Big Endian, (value-500)/10
    uint16_t temp_raw = (i2c_buffer[9] << 8) | i2c_buffer[10];
    data.temperature = (temp_raw - 500) / 10.0;
    
    // PM1.0: [11][12] - Big Endian
    uint16_t pm1_raw = (i2c_buffer[11] << 8) | i2c_buffer[12];
    data.pm1_0 = pm1_raw;
    
    // PM2.5: [13][14] - Big Endian
    uint16_t pm25_raw = (i2c_buffer[13] << 8) | i2c_buffer[14];
    data.pm2_5 = pm25_raw;
    
    // PM10: [15][16] - Big Endian
    uint16_t pm10_raw = (i2c_buffer[15] << 8) | i2c_buffer[16];
    data.pm10 = pm10_raw;
    
    // 데이터 유효성 검사
    if (data.co2 <= 5000 && 
        data.humidity >= 0 && data.humidity <= 100 && 
        data.temperature >= -40 && data.temperature <= 85 &&
        data.pm1_0 <= 1000 && data.pm2_5 <= 1000 && data.pm10 <= 1000 &&
        data.voc_level <= 3) {
      
      data.valid = true;
      
      Serial.println("Parsed I2C data:");
      Serial.printf("  CO2: %d ppm\n", data.co2);
      Serial.printf("  VOC: %d level\n", data.voc_level);
      Serial.printf("  Humidity: %.1f %%\n", data.humidity);
      Serial.printf("  Temperature: %.1f °C\n", data.temperature);
      Serial.printf("  PM1.0: %d ug/m³\n", data.pm1_0);
      Serial.printf("  PM2.5: %d ug/m³\n", data.pm2_5);
      Serial.printf("  PM10: %d ug/m³\n", data.pm10);
    } else {
      Serial.println("Sensor data validation failed - values out of range");
      data.valid = false;
    }
  } else {
    Serial.println("Invalid I2C response header");
    Serial.print("Expected: 0x16 0x19, Got: ");
    Serial.print("0x"); if(i2c_buffer[0] < 16) Serial.print("0"); Serial.print(i2c_buffer[0], HEX);
    Serial.print(" 0x"); if(i2c_buffer[1] < 16) Serial.print("0"); Serial.println(i2c_buffer[1], HEX);
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

// Light Sleep 모드 실행 함수 (기존 코드)
void enterLightSleep(unsigned long sleep_duration_ms) {
  Serial.printf("💤 Light Sleep 모드 진입 (%lums)...\n", sleep_duration_ms);
  
  // 🔋 Light Sleep 개선: 불필요한 주변장치 비활성화
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
  
  // Light Sleep 설정 및 실행
  esp_sleep_enable_timer_wakeup(sleep_duration_ms * 1000);  // μs 단위로 변환
  esp_light_sleep_start();
  
  Serial.println("⏰ Light Sleep에서 깨어남 (개선된 절전 모드)");
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n=== LoRaWAN + AM1008W-K-P Sensor Initializing ===");
  
  // 🔋 1단계: CPU 클록 최적화 (240MHz → 80MHz, 안전함)
  Serial.printf("CPU 클록 변경 전: %dMHz\n", getCpuFrequencyMhz());
  setCpuFrequencyMhz(80);  // 240MHz → 80MHz
  Serial.printf("CPU 클록 변경 후: %dMHz (67%% 전력 절약!)\n", getCpuFrequencyMhz());
  
  // 🔋 2단계: LoRa TX 출력 최적화 (22dBm → 14dBm, 안전함)
  Serial.println("LoRa TX 출력을 14dBm으로 최적화 (기본 22dBm)");
  
  // 🔋 3단계: Light Sleep 개선 (더 깊은 절전 모드)
  Serial.println("Light Sleep 모드 개선 설정");
  
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
  
  // AM1008W-K-P I2C 초기화 (필수)
  Serial.println("\n=== AM1008W-K-P Sensor Initialization ===");
  displayInitScreen("Init AM1008W-K-P I2C...");
  
  // AM1008W-K-P I2C 모드 대기 (전원 공급 후 10초 대기)
  Serial.println("Waiting 5 seconds for AM1008W-K-P initialization...");
  displayInitScreen("Wait 5s for I2C...");
  delay(5000); // 센서가 이미 I2C 모드이므로 5초로 단축
  
  // AM1008W-K-P용 I2C 초기화 (GPIO41, 42) - Wire0 사용
  Serial.println("Initializing I2C on GPIO41 (SDA), GPIO42 (SCL)...");
  Wire.begin(AM1008_SDA_PIN, AM1008_SCL_PIN);
  Wire.setClock(10000); // 10kHz - 안전한 속도
  delay(100);
  Serial.println("AM1008W-K-P I2C (Wire0) initialized");
  Serial.printf("SDA: GPIO%d, SCL: GPIO%d\n", AM1008_SDA_PIN, AM1008_SCL_PIN);
  Serial.println("Clock: 10kHz, Address: 0x28");
  
  // 하드웨어 상세 테스트 실행
  Serial.println("\n=== Hardware Diagnostic Tests ===");
  detailedHardwareTest();
  
  // I2C 주소 스캔 (디버깅용)
  scanI2CDevices();
  
  // AM1008W-K-P 특정 주소 테스트
  Serial.println("\n=== AM1008W-K-P Detection ===");
  
  // PM 센서 동적 감지 및 초기화
  if (initializeAM1008PMSensor()) {
    Serial.println("AM1008W-K-P sensor initialized successfully!");
  } else {
    Serial.println("AM1008W-K-P sensor initialization failed!");
  }
  
  // AM1008W-K-P 데이터 읽기 테스트 (3번 시도)
  Serial.println("\n=== AM1008W-K-P Data Test ===");
  AM1008Data testData = {0};
  bool sensor_working = false;
  
  for (int attempt = 1; attempt <= 3; attempt++) {
    Serial.printf("AM1008W-K-P data read test attempt %d/3\n", attempt);
    testData = readAM1008Data();
    
    if (testData.valid) {
      Serial.println("AM1008W-K-P sensor data valid and working!");
      am1008_available = true;
      sensor_working = true;
      displayInitScreen("AM1008W-K-P OK");
      break;
    } else {
      Serial.println("AM1008W-K-P data test failed on attempt " + String(attempt));
      if (attempt < 3) {
        Serial.println("Waiting 2 seconds before retry...");
        delay(2000);
      }
    }
  }
  
  if (!sensor_working) {
    Serial.println("WARNING: AM1008W-K-P sensor data validation failed!");
    Serial.println("Continuing without valid sensor data...");
    displayInitScreen("Sensor Data Invalid!");
    delay(3000);
  }
  
  Serial.println("\n=== LoRaWAN Network Initialization ===");
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
  Serial.println("Initializing LoRa radio...");
  int16_t state = radio.begin();
  debug(state != RADIOLIB_ERR_NONE, F("Radio initialization failed"), state, true);
  Serial.println("LoRa radio initialized successfully");
  
  // 🔋 LoRa TX 출력 설정 (22dBm → 14dBm, 약 50% 전력 절약)
  radio.setOutputPower(14);  // 14dBm (기본값: 22dBm)
  Serial.printf("LoRa TX 출력: 14dBm으로 설정 완료\n");

  displayInitScreen("Init LoRaWAN node...");
  
  // LoRaWAN 노드 설정
  Serial.println("Setting up LoRaWAN node...");
  node.beginOTAA(joinEUI, devEUI, nwkKey, appKey);
  Serial.println("LoRaWAN node configured");

  // LoRaWAN 네트워크 조인
  Serial.println("Joining LoRaWAN network...");
  Serial.println("This may take 10-30 seconds...");
  displayInitScreen("Joining LoRaWAN...");
  
  state = node.activateOTAA(); 
  debug(state != RADIOLIB_LORAWAN_NEW_SESSION, F("LoRaWAN join failed"), state, true);

  Serial.println("LoRaWAN network joined successfully!");
  Serial.println("Ready for operation!");
  
  // 초기 연결 성공
  lorawan_status = LORAWAN_CONNECTED;
  consecutive_send_failures = 0;
  last_successful_send = millis();
  
  displayInitScreen("System Ready!");
  delay(2000);
  
  Serial.println("\n" + String("=").substring(0, 50));
  Serial.println("INITIALIZATION COMPLETE");
  Serial.println("Device ID: " + device_id);
  Serial.println("AM1008W-K-P: " + String(am1008_available ? "Available" : "Not Available"));
  Serial.println("OLED Display: " + String(oled_available ? "Available" : "Not Available"));
  Serial.println("LoRaWAN: Connected");
  Serial.println("Transmission Interval: " + String(uplinkIntervalSeconds) + " seconds");
  Serial.println(String("=").substring(0, 50));
}

void loop() {
  uint32_t currentTime = millis();
  
  Serial.println("\n=== SENSOR CYCLE ===");
  
  // 센서 데이터 읽기
  SensorData sensorData = readSensors();
  
  // 연결 상태 확인 및 재연결 시도
  if (!node.isActivated() || consecutive_send_failures >= MAX_SEND_FAILURES) {
    Serial.println("=== CONNECTION ISSUE DETECTED ===");
    Serial.println("LoRaWAN Activated: " + String(node.isActivated()));
    Serial.println("Consecutive failures: " + String(consecutive_send_failures));
    
    // 스마트 재연결 시도
    if (smartReconnect()) {
      Serial.println("Reconnection successful!");
      lorawan_status = LORAWAN_CONNECTED;
    } else {
      Serial.println("Reconnection failed!");
      lorawan_status = LORAWAN_DISCONNECTED;
    }
  } else {
    lorawan_status = LORAWAN_CONNECTED;
  }
  
  // OLED 디스플레이 업데이트
  updateDisplay(sensorData, lorawan_status);
  
  // 센서 데이터 시리얼 출력
  Serial.println("=== AM1008W-K-P Sensor Data ===");
  Serial.println("Device ID: " + device_id);
  Serial.println("Timestamp: " + String(millis() / 1000) + "s");
  
  if (sensorData.am1008_available && sensorData.am1008.valid) {
    Serial.print("Temperature: ");
    if (isnan(sensorData.am1008.temperature)) {
      Serial.println("N/A");
    } else {
      Serial.println(String(sensorData.am1008.temperature, 1) + "C");
    }
    
    Serial.print("Humidity: ");
    if (isnan(sensorData.am1008.humidity)) {
      Serial.println("N/A");
    } else {
      Serial.println(String(sensorData.am1008.humidity, 1) + "%");
    }
    
    Serial.printf("CO2: %d ppm\n", sensorData.am1008.co2);
    Serial.printf("VOC Level: %d\n", sensorData.am1008.voc_level);
    Serial.printf("PM1.0: %d ug/m3\n", sensorData.am1008.pm1_0);
    Serial.printf("PM2.5: %d ug/m3\n", sensorData.am1008.pm2_5);
    Serial.printf("PM10: %d ug/m3\n", sensorData.am1008.pm10);
  } else {
    Serial.println("AM1008W-K-P sensor not available or invalid data");
  }
  
  // LoRaWAN 전송 시도
  if (lorawan_status == LORAWAN_CONNECTED) {
    Serial.println("=== LoRaWAN Transmission ===");
    uint8_t uplinkPayload[16];
    encodeSensorData(sensorData, uplinkPayload);
    
    Serial.println("Sending sensor data via LoRaWAN...");
    int16_t sendState = node.sendReceive(uplinkPayload, sizeof(uplinkPayload)); 
    
    if (sendState == RADIOLIB_ERR_NONE || sendState == RADIOLIB_LORAWAN_NEW_SESSION) {
      Serial.println("Data sent successfully! (State: " + stateDecode(sendState) + ")");
      consecutive_send_failures = 0;
      last_successful_send = currentTime;
      lorawan_status = LORAWAN_CONNECTED;
    } else {
      Serial.println("Transmission failed: " + stateDecode(sendState) + " (" + String(sendState) + ")");
      consecutive_send_failures++;
      lorawan_status = LORAWAN_SEND_FAILED;
      
      Serial.println("Consecutive failures: " + String(consecutive_send_failures) + "/" + String(MAX_SEND_FAILURES));
      
      // 즉시 재연결 시도 (특정 에러의 경우)
      if (sendState == RADIOLIB_ERR_NETWORK_NOT_JOINED || 
          sendState == RADIOLIB_ERR_JOIN_NONCE_INVALID ||
          sendState == RADIOLIB_ERR_CHIP_NOT_FOUND) {
        Serial.println("Critical network/hardware error detected. Attempting immediate reconnection...");
        smartReconnect();
      }
    }
  } else {
    Serial.println("LoRaWAN not connected - skipping data transmission");
  }

  // 전송 결과를 반영하여 디스플레이 다시 업데이트
  updateDisplay(sensorData, lorawan_status);

  // 시스템 상태 및 통계 정보 출력
  Serial.println("=== System Status ===");
  Serial.println("LoRaWAN Status: " + String(
    lorawan_status == LORAWAN_CONNECTED ? "Connected" :
    lorawan_status == LORAWAN_CONNECTING ? "Connecting" :
    lorawan_status == LORAWAN_SEND_FAILED ? "Send Failed" :
    lorawan_status == LORAWAN_REJOIN_NEEDED ? "Rejoining" : "Disconnected"
  ));
  Serial.println("Consecutive failures: " + String(consecutive_send_failures));
  Serial.println("Last successful send: " + String((currentTime - last_successful_send) / 1000) + "s ago");
  Serial.println("Next transmission in " + String(uplinkIntervalSeconds) + " seconds");

  // 화면 표시 시간 (5초간 켜두기)
  Serial.println("Display will stay on for 5 seconds...");
  delay(5000);

  // 화면 끄기 (전력 절약)
  if (oled_available) {
    display.clearDisplay();
    display.display();
    Serial.println("Display turned off for power saving");
  }

  // Light Sleep으로 전환 (메모리 유지 = 재JOIN 방지)
  uint32_t sleepTime = uplinkIntervalSeconds - 5;
  Serial.println("Entering light sleep for " + String(sleepTime) + " seconds...");
  Serial.println("LoRaWAN session will be preserved during sleep.");
  
  enterLightSleep(sleepTime);
  
  // 깨어남 후 다음 루프 시작
  Serial.println("System wake-up - Starting next sensor cycle...");
}