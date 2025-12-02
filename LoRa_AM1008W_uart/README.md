# LoRa AM1008W-K-P 센서 프로젝트

**Heltec WiFi LoRa 32 V3 보드**에 **AM1008W-K-P 센서**(UART)를 연결하여 **LoRaWAN**을 통해 **TTN**으로 데이터를 전송하는 파이프라인입니다.

## 하드웨어 구성

### 메인 보드
- **Heltec WiFi LoRa 32 V3** (ESP32-S3 + SX1262)

### 센서
- **AM1008W-K-P**: CO2, 온도, 습도, PM1.0/2.5/10, VOC 측정
- **BME280** (선택사항): 온도, 습도, 압력 측정
- **BMP390** (선택사항): 온도, 압력, 고도 측정

### 연결 핀맵

#### AM1008W-K-P (UART 통신)
- RX: GPIO47
- TX: GPIO48
- VCC: 5V
- GND: GND

#### BME280/BMP390 (I2C 통신)
- SDA: GPIO41
- SCL: GPIO42
- VCC: 3.3V
- GND: GND

#### OLED 디스플레이 (I2C 통신)
- SDA: GPIO17
- SCL: GPIO18
- VCC: 3.3V
- GND: GND

## 기능

### 센서 데이터 수집
- **AM1008W-K-P**: CO2(ppm), 온도(°C), 습도(%), PM1.0/2.5/10(μg/m³), VOC 레벨
- **BME280**: 온도(°C), 습도(%), 기압(hPa)
- **BMP390**: 온도(°C), 기압(hPa), 고도(m)

### LoRaWAN 전송
- **주파수 대역**: KR920 (한국)
- **전송 간격**: 60초
- **페이로드 크기**: 20바이트
- **네트워크**: TTN (The Things Network)

### 페이로드 구조 (20바이트)
```
[0-1]:   BME280 온도 (0.1°C 정밀도, -40~85°C)
[2-3]:   BME280 습도 (0.1% 정밀도, 0~100%)  
[4-5]:   BME280 압력 (0.1hPa 정밀도, 800~1200hPa)
[6-7]:   AM1008W 온도 (0.1°C 정밀도, -40~85°C)
[8-9]:   AM1008W 습도 (0.1% 정밀도, 0~100%)
[10-11]: AM1008W CO2 (ppm)
[12-13]: AM1008W PM2.5 (μg/m³)
[14-15]: AM1008W PM10 (μg/m³)
[16]:    AM1008W VOC Level (0-255)
[17]:    센서 상태 플래그 (bit0:BME280, bit1:BMP390, bit2:AM1008W)
[18]:    연속 실패 횟수
[19]:    예약/체크섬
```

### OLED 디스플레이
- 실시간 센서 데이터 표시
- LoRaWAN 연결 상태 표시
- 디바이스 ID 표시
- 아이콘 기반 직관적 UI

### 전력 관리
- Light Sleep 모드 사용
- LoRaWAN 세션 유지
- OLED 자동 꺼짐
- 배터리 효율적 운영

## 설정 방법

### 1. 디바이스 등록
`data/device_registry.json` 파일에서 MAC 주소와 디바이스 ID 매핑:
```json
{
  "3C61052DA3F0": "LoRa-AM-001",
  "3C61052DA3F1": "LoRa-AM-002"
}
```

### 2. LoRaWAN 설정
`src/config.h` 파일에서 TTN 키 설정:
- `RADIOLIB_LORAWAN_DEV_EUI`
- `RADIOLIB_LORAWAN_APP_KEY`  
- `RADIOLIB_LORAWAN_NWK_KEY`

### 3. 컴파일 및 업로드
```bash
pio run --target upload
pio run --target uploadfs  # 파일시스템 업로드
```

## 특징

### 센서 우선순위
1. **AM1008W-K-P**: 주요 센서 (필수)
2. **BME280/BMP390**: 보조 센서 (선택사항)

### 오류 처리
- 센서 통신 실패 시 자동 재시도
- LoRaWAN 연결 실패 시 스마트 재연결
- 하드웨어 리셋 기능
- 시스템 자동 재시작

### 디버깅
- 상세한 시리얼 로그
- 센서 상태 모니터링
- 연결 통계 정보
- 오류 코드 해석

## 사용법

1. 하드웨어 연결
2. TTN에서 디바이스 등록
3. 설정 파일 수정
4. 펌웨어 업로드
5. 시리얼 모니터로 상태 확인
6. TTN 콘솔에서 데이터 확인

## 문제 해결

### AM1008W-K-P 통신 오류
- 핀 연결 확인 (GPIO47, 48)
- 전원 공급 확인 (5V)
- 보드레이트 확인 (9600)

### LoRaWAN 연결 실패
- TTN 키 설정 확인
- 주파수 대역 확인 (KR920)
- 안테나 연결 확인

### OLED 표시 안됨
- I2C 연결 확인 (GPIO17, 18)
- VEXT 핀 상태 확인
- 주소 설정 확인 (0x3C)

## 라이선스

MIT License

## 작성자

EAN RU 이정아 연구원
