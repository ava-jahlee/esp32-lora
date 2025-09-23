#include "config.h" // config.h 파일에 LoRaWAN 설정 및 라디오/노드 객체 정의가 있음

#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BMP3XX.h>
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

// 센서 I2C 핀 설정 (GPIO41, 42)
#define SENSOR_SDA_PIN 41
#define SENSOR_SCL_PIN 42

// OLED I2C 핀 설정 (GPIO17, 18)
#define OLED_SDA_PIN 17
#define OLED_SCL_PIN 18

// 센서 I2C 주소
#define BME280_ADDRESS 0x76
#define BMP390_ADDRESS 0x77

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

const unsigned char PROGMEM icon_pressure[] = {
  0x3C, 0x42, 0x99, 0xA5, 0xA5, 0x99, 0x42, 0x3C  // 🎈 풍선 (압력 대신)
};

const unsigned char PROGMEM icon_altitude[] = {
  0x10, 0x38, 0x7C, 0xFE, 0x44, 0x28, 0x10, 0x00  // 🏔️ 산 (더 선명하게)
};

const unsigned char PROGMEM icon_lora[] = {
  0x10, 0x38, 0x54, 0x92, 0x10, 0x10, 0x10, 0x7C  // 안테나 + 받침
};

const unsigned char PROGMEM icon_paw[] = {
  0x60, 0x90, 0x90, 0x60, 0x00, 0x66, 0x99, 0x66  // 🐾 발자국
};

// 센서 데이터 구조체
struct SensorData {
  float temperature_bme;  // BME280 온도
  float humidity;         // BME280 습도
  float pressure_bme;     // BME280 압력
  float temperature_bmp;  // BMP390 온도
  float pressure_bmp;     // BMP390 압력
  float altitude;         // BMP390 고도
};

// 연결 상태 enum
enum LoRaWANStatus {
  LORAWAN_DISCONNECTED,
  LORAWAN_CONNECTING,
  LORAWAN_CONNECTED,
  LORAWAN_SEND_FAILED,
  LORAWAN_REJOIN_NEEDED
};

// 센서 객체 생성
Adafruit_BME280 bme;
Adafruit_BMP3XX bmp;

// OLED 객체 생성
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);

// 상태 변수들
bool bme280_available = false;  // BME280 가용성 추가
bool bmp390_available = false;
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

  // 파일 크기 확인하여 적절한 JSON 문서 크기 설정
  size_t fileSize = file.size();
  DynamicJsonDocument doc(fileSize + 512); // 여유분 추가
  
  DeserializationError error = deserializeJson(doc, file);
  file.close();
  LittleFS.end(); // 메모리 누수 방지

  if (error) {
    Serial.println("JSON 파싱 오류: " + String(error.c_str()));
    return "LoRa-XXX";
  }

  if (doc.containsKey(chipidStr)) {
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
  display.println(" LoRa:Stair ");
  
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

  // 온도 (왼쪽 - 아이콘 + 텍스트) - BME280 사용
  display.drawBitmap(0, 26, icon_temp, 8, 8, SSD1306_WHITE);
  display.setCursor(12, 26);
  display.print("Temp: ");
  display.print(data.temperature_bme, 1);
  display.println(" C");
  
  // 습도 (왼쪽) - BME280 사용
  display.drawBitmap(0, 36, icon_humidity, 8, 8, SSD1306_WHITE);
  display.setCursor(12, 36);
  display.print("Humi: ");
  display.print(data.humidity, 1);
  display.println(" %");
  
  // 압력 (왼쪽) - BMP390 우선, 없으면 BME280
  display.drawBitmap(0, 46, icon_pressure, 8, 8, SSD1306_WHITE);
  display.setCursor(12, 46);
  display.print("Press: ");
  if (bmp390_available) {
    display.print(data.pressure_bmp, 1);
  } else {
    display.print(data.pressure_bme, 1);
  }
  display.println(" hPa");
  
  // 고도 - BMP390만 (더 정확함)
  display.drawBitmap(0, 56, icon_altitude, 8, 8, SSD1306_WHITE);
  display.setCursor(12, 56);
  display.print("Alt: ");
  if (bmp390_available) {
    display.print(data.altitude, 0);
    display.println(" m");
  } else {
    display.println("N/A");
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
  int16_t nodeState = node.beginOTAA(joinEUI, devEUI, nwkKey, appKey);
  if (nodeState != RADIOLIB_ERR_NONE) {
    Serial.println("Node reinitialization failed: " + stateDecode(nodeState));
    return false;
  }
  
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
    int16_t restoreState = node.beginOTAA(joinEUI, devEUI, nwkKey, appKey);
    
    if (restoreState == RADIOLIB_LORAWAN_SESSION_RESTORED) {
      Serial.println("✓ Session restored successfully!");
      consecutive_send_failures = 0;
      last_successful_send = millis();
      lorawan_status = LORAWAN_CONNECTED;
      return true;
    } else if (restoreState == RADIOLIB_LORAWAN_NEW_SESSION) {
      Serial.println("✓ New session created!");
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

// 센서 데이터 읽기 함수 (데이터 유효성 검증 추가)
SensorData readSensors() {
  SensorData data;
  
  // 기본값 설정 (센서 오류 시 대비)
  data.temperature_bme = 0.0;
  data.humidity = 0.0;
  data.pressure_bme = 1013.25;
  data.temperature_bmp = 0.0;
  data.pressure_bmp = 1013.25;
  data.altitude = 0.0;
  
  // BME280 데이터 읽기
  if (bme280_available) {
    data.temperature_bme = bme.readTemperature();
    data.humidity = bme.readHumidity();
    data.pressure_bme = bme.readPressure() / 100.0F; // Pa to hPa
    
    // 데이터 유효성 검증
    if (isnan(data.temperature_bme) || data.temperature_bme < -40 || data.temperature_bme > 85) {
      Serial.println("Warning: Invalid BME280 temperature reading");
      data.temperature_bme = 0.0;
    }
    if (isnan(data.humidity) || data.humidity < 0 || data.humidity > 100) {
      Serial.println("Warning: Invalid BME280 humidity reading");
      data.humidity = 0.0;
    }
    if (isnan(data.pressure_bme) || data.pressure_bme < 800 || data.pressure_bme > 1200) {
      Serial.println("Warning: Invalid BME280 pressure reading");
      data.pressure_bme = 1013.25;
    }
  }
  
  // BMP390 사용 가능 여부에 따라 분기
  if (bmp390_available) {
    // BMP390 새 데이터 읽기
    if (bmp.performReading()) {
      data.temperature_bmp = bmp.temperature;
      data.pressure_bmp = bmp.pressure / 100.0F; // Pa to hPa
      data.altitude = bmp.readAltitude(1013.25); // 해수면 기압 기준
      
      // 데이터 유효성 검증
      if (isnan(data.temperature_bmp) || data.temperature_bmp < -40 || data.temperature_bmp > 85) {
        Serial.println("Warning: Invalid BMP390 temperature reading");
        data.temperature_bmp = data.temperature_bme;
      }
      if (isnan(data.pressure_bmp) || data.pressure_bmp < 800 || data.pressure_bmp > 1200) {
        Serial.println("Warning: Invalid BMP390 pressure reading");
        data.pressure_bmp = data.pressure_bme;
      }
      if (isnan(data.altitude) || data.altitude < -500 || data.altitude > 4000) {
        Serial.println("Warning: Invalid BMP390 altitude reading");
        data.altitude = 0.0;
      }
    } else {
      // BMP390 읽기 실패 시 BME280 값 사용
      Serial.println("Warning: BMP390 reading failed, using BME280 data");
      data.temperature_bmp = data.temperature_bme;
      data.pressure_bmp = data.pressure_bme;
      data.altitude = 0.0;
    }
  } else {
    // BMP390 없으면 BME280 값으로 대체
    data.temperature_bmp = data.temperature_bme;   // BME280 온도 사용
    data.pressure_bmp = data.pressure_bme;         // BME280 압력 사용
    data.altitude = 0.0;                           // 고도 0으로 설정
  }
  
  return data;
}

// 센서 데이터를 바이트 배열로 변환
void encodeSensorData(SensorData data, uint8_t* buffer) {
  // 온도: -40~85°C를 0~1250으로 매핑 (0.1°C 정밀도)
  uint16_t temp_bme = (uint16_t)((data.temperature_bme + 40) * 10);
  uint16_t temp_bmp = (uint16_t)((data.temperature_bmp + 40) * 10);
  
  // 습도: 0~100%를 0~1000으로 매핑 (0.1% 정밀도)
  uint16_t hum = (uint16_t)(data.humidity * 10);
  
  // 압력: 800~1200hPa를 0~4000으로 매핑 (0.1hPa 정밀도)
  uint16_t press_bme = (uint16_t)((data.pressure_bme - 800) * 10);
  uint16_t press_bmp = (uint16_t)((data.pressure_bmp - 800) * 10);
  
  // 고도: -500~4000m를 0~4500으로 매핑 (1m 정밀도)
  uint16_t alt = (uint16_t)(data.altitude + 500);
  
  // 12바이트 패킷 구성 + 상태 정보 추가 (13바이트)
  buffer[0] = temp_bme >> 8;        // BME280 온도 상위
  buffer[1] = temp_bme & 0xFF;      // BME280 온도 하위
  buffer[2] = hum >> 8;             // 습도 상위
  buffer[3] = hum & 0xFF;           // 습도 하위
  buffer[4] = press_bme >> 8;       // BME280 압력 상위
  buffer[5] = press_bme & 0xFF;     // BME280 압력 하위
  buffer[6] = temp_bmp >> 8;        // BMP390 온도 상위
  buffer[7] = temp_bmp & 0xFF;      // BMP390 온도 하위
  buffer[8] = press_bmp >> 8;       // BMP390 압력 상위
  buffer[9] = press_bmp & 0xFF;     // BMP390 압력 하위
  buffer[10] = alt >> 8;            // 고도 상위
  buffer[11] = alt & 0xFF;          // 고도 하위
  buffer[12] = consecutive_send_failures; // 연속 실패 횟수 (디버깅용)
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

  // 센서용 I2C 초기화 (GPIO41, 42) - Wire 사용
  Wire.begin(SENSOR_SDA_PIN, SENSOR_SCL_PIN);
  Serial.println("Sensor I2C initialized");
  displayInitScreen("I2C initialized");
  delay(500);
  
  // BME280 초기화 (주소 0x76)
  Serial.println("Attempting BME280 initialization...");
  if (!bme.begin(BME280_ADDRESS, &Wire)) {
    Serial.println("Critical: BME280 sensor not found at 0x76!");
    displayInitScreen("BME280 FAIL!");
    bme280_available = false;
    delay(3000);
    
    // BME280 없이는 동작 불가 - 재시작
    Serial.println("BME280 is required sensor. Restarting...");
    ESP.restart();
  } else {
    Serial.println("BME280 initialized successfully (0x76)");
    bme280_available = true;
    displayInitScreen("BME280 OK");
  }
  delay(500);

  // BMP390 초기화 전 지연
  delay(1000);
  Serial.println("Attempting BMP390 initialization...");
  displayInitScreen("Checking BMP390...");
  
  // BMP390 초기화 (주소 0x77)
  if (!bmp.begin_I2C(BMP390_ADDRESS, &Wire)) {
    Serial.println("BMP390 sensor not found at 0x77!");
    Serial.println("Continuing with BME280 only...");
    bmp390_available = false;
    displayInitScreen("BMP390 not found");
  } else {
    Serial.println("BMP390 initialized successfully (0x77)");
    bmp390_available = true;   // BMP390 사용 가능 표시
    
    // BMP390 설정
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    Serial.println("BMP390 configured");
    displayInitScreen("BMP390 OK");
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
  state = node.beginOTAA(joinEUI, devEUI, nwkKey, appKey);
  debug(state != RADIOLIB_ERR_NONE, F("Initialise node failed"), state, true);

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
  
  // 시리얼로 센서 데이터 출력
  Serial.println("=== Sensor Data ===");
  Serial.println("Device ID: " + device_id);
  Serial.println("BME280 - Temp: " + String(sensorData.temperature_bme, 1) + "°C, Humidity: " + String(sensorData.humidity, 1) + "%, Pressure: " + String(sensorData.pressure_bme, 1) + "hPa");
  
  if (bmp390_available) {
    Serial.println("BMP390 - Temp: " + String(sensorData.temperature_bmp, 1) + "°C, Pressure: " + String(sensorData.pressure_bmp, 1) + "hPa, Altitude: " + String(sensorData.altitude, 0) + "m");
  } else {
    Serial.println("BMP390 - Not available (using BME280 data)");
  }
  
  // LoRaWAN 전송 시도 (연결된 경우에만)
  if (lorawan_status == LORAWAN_CONNECTED) {
    uint8_t uplinkPayload[13]; // 상태 정보 포함하여 13바이트
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
          sendState == RADIOLIB_ERR_NO_JOIN_ACCEPT ||
          sendState == RADIOLIB_ERR_CHIP_NOT_FOUND) { // CHIP_NOT_FOUND 추가
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