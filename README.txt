|- 📁 LoRa_DevEUI

|- 📁 I2C_Scanner

|- 📁 LoRa_Stabilize

|- 📁 LoRa_Stabilize_v2

|- 📁 LoRa_AM1008W


- LoRa_DevEUI : ESP32 DevEUI 확인용 .ino

- I2C_Scanner : 현재 LoRa에 연결된 센서의 점유 I2C 주소 확인 및 센서 자체의 연결 확인 .ino

- LoRa_Stabilize : LoRa BME280, BMP390, Display 표시 및 재연결 로직을 포함한 코드 .cpp
    - config.h
        - eui, key values나 핀설정 등
    - /data/device_registry.json
        - littlefs로 업로드 필요하므로 vscode에서 진행하는 게 젤 편함
- LoRa_Stabilize_v2 : LoRa_Stabilize 기본 코드에 배터리 잔량 표시하는 코드 추가

- LoRa_AM1008w : LoRa에 AM1008W 센서 및 Level Shifter 연결. Display 표시 및 재연결 로직을 포함한 코드 .cpp
