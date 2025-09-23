#ifndef _RADIOLIB_EX_LORAWAN_CONFIG_H
#define _RADIOLIB_EX_LORAWAN_CONFIG_H

#include <RadioLib.h>

// first you have to set your radio model and pin configuration
// this is provided just as a default example
// Heltec WiFi LoRa 32 V3 핀맵 (SX1262)
SX1262 radio = new Module(8, 14, 12, 13);
//                       NSS, DIO1, RST, BUSY

// if you have RadioBoards (https://github.com/radiolib-org/RadioBoards)
// and are using one of the supported boards, you can do the following:
/*
#define RADIO_BOARD_AUTO
#include <RadioBoards.h>

Radio radio = new RadioModule();
*/

// how often to send an uplink - consider legal & FUP constraints - see notes
const uint32_t uplinkIntervalSeconds = 1UL * 10UL; // 10초 단위
// joinEUI - previous versions of LoRaWAN called this AppEUI
// for development purposes you can use all zeros - see wiki for details
#define RADIOLIB_LORAWAN_JOIN_EUI  0x22BC781E8AD1DD69

#ifndef RADIOLIB_LORAWAN_DEV_EUI
#define RADIOLIB_LORAWAN_DEV_EUI   0x0000489A6ABA2010
#endif
// LoRa-001: 0x0000249B6ABA2010
// LoRa-002: 0x000038FD66BA2010
// LoRa-003: 0x0000500367BA2010
// LoRa-004: 0x0000489A6ABA2010
// LoRa-005: 0x0000F8F966BA2010

#ifndef RADIOLIB_LORAWAN_APP_KEY
#define RADIOLIB_LORAWAN_APP_KEY   0x82, 0x2E, 0x38, 0x25, 0x6D, 0x85, 0xC1, 0x1E, 0x58, 0x5F, 0xCF, 0xA3, 0x8A, 0xD8, 0xF7, 0xEB
#endif
// LoRa-001: 0x4B, 0x5C, 0x5D, 0x83, 0x36, 0xB7, 0x8B, 0xBB, 0xB0, 0x33, 0x8F, 0xE6, 0xEF, 0x33, 0x9E, 0x08
// LoRa-002: 0x9E, 0x3D, 0x8E, 0x3E, 0x06, 0x06, 0xD8, 0xE7, 0x86, 0x73, 0x4B, 0x3F, 0xDC, 0x1C, 0x94, 0x78
// LoRa-003: 0x0F, 0xD6, 0x2B, 0x64, 0xA1, 0x2E, 0x9B, 0x9E, 0x4C, 0xA2, 0x1B, 0xA0, 0x2D, 0x1C, 0x12, 0x43
// LoRa-004: 0x82, 0x2E, 0x38, 0x25, 0x6D, 0x85, 0xC1, 0x1E, 0x58, 0x5F, 0xCF, 0xA3, 0x8A, 0xD8, 0xF7, 0xEB
// LoRa-005: 0xBC, 0xFC, 0x5D, 0xD2, 0xDD, 0xFA, 0xD4, 0x54, 0x08, 0x4A, 0xDB, 0xB5, 0x50, 0xF0, 0x03, 0xCC

#ifndef RADIOLIB_LORAWAN_NWK_KEY
#define RADIOLIB_LORAWAN_NWK_KEY   0x71, 0xA8, 0x7F, 0x25, 0xD3, 0x9F, 0x47, 0x55, 0xAA, 0x3C, 0x6B, 0x82, 0x40, 0xA1, 0x75, 0x67
#endif
// LoRa-001: 0x29, 0x32, 0x5B, 0x81, 0x66, 0x1D, 0x2D, 0x5C, 0xB1, 0x54, 0xE4, 0x2C, 0x00, 0x11, 0x28, 0x1C
// LoRa-002: 0x14, 0x06, 0x40, 0xBE, 0x2E, 0x8E, 0xA0, 0x55, 0xF5, 0xE1, 0x02, 0x2C, 0x0B, 0x71, 0xEC, 0x44
// LoRa-003: 0xEF, 0x26, 0x1F, 0x91, 0x64, 0x12, 0x29, 0xE3, 0x62, 0x2C, 0xC4, 0xDC, 0x70, 0xCB, 0x17, 0x5B
// LoRa-004: 0x71, 0xA8, 0x7F, 0x25, 0xD3, 0x9F, 0x47, 0x55, 0xAA, 0x3C, 0x6B, 0x82, 0x40, 0xA1, 0x75, 0x67
// LoRa-005: 0xCF, 0xA5, 0xA9, 0xB6, 0xBA, 0xC7, 0x95, 0xFD, 0x43, 0x74, 0x88, 0x3C, 0xBE, 0x06, 0xD7, 0x23

// for the curious, the #ifndef blocks allow for automated testing &/or you can
// put your EUI & keys in to your platformio.ini - see wiki for more tips

// regional choices: EU868, US915, AU915, AS923, AS923_2, AS923_3, AS923_4, IN865, KR920, CN470
const LoRaWANBand_t Region = KR920;

// subband choice: for US915/AU915 set to 2, for CN470 set to 1, otherwise leave on 0
const uint8_t subBand = 0;

// ============================================================================
// Below is to support the sketch - only make changes if the notes say so ...

// copy over the EUI's & keys in to the something that will not compile if incorrectly formatted
uint64_t joinEUI =   RADIOLIB_LORAWAN_JOIN_EUI;
uint64_t devEUI  =   RADIOLIB_LORAWAN_DEV_EUI;
uint8_t appKey[] = { RADIOLIB_LORAWAN_APP_KEY };
uint8_t nwkKey[] = { RADIOLIB_LORAWAN_NWK_KEY };

// create the LoRaWAN node
LoRaWANNode node(&radio, &Region, subBand);

// result code to text - these are error codes that can be raised when using LoRaWAN
// however, RadioLib has many more - see https://jgromes.github.io/RadioLib/group__status__codes.html for a complete list
String stateDecode(const int16_t result) {
  switch (result) {
  case RADIOLIB_ERR_NONE:
    return "ERR_NONE";
  case RADIOLIB_ERR_CHIP_NOT_FOUND:
    return "ERR_CHIP_NOT_FOUND";
  case RADIOLIB_ERR_PACKET_TOO_LONG:
    return "ERR_PACKET_TOO_LONG";
  case RADIOLIB_ERR_RX_TIMEOUT:
    return "ERR_RX_TIMEOUT";
  case RADIOLIB_ERR_MIC_MISMATCH:
    return "ERR_MIC_MISMATCH";
  case RADIOLIB_ERR_INVALID_BANDWIDTH:
    return "ERR_INVALID_BANDWIDTH";
  case RADIOLIB_ERR_INVALID_SPREADING_FACTOR:
    return "ERR_INVALID_SPREADING_FACTOR";
  case RADIOLIB_ERR_INVALID_CODING_RATE:
    return "ERR_INVALID_CODING_RATE";
  case RADIOLIB_ERR_INVALID_FREQUENCY:
    return "ERR_INVALID_FREQUENCY";
  case RADIOLIB_ERR_INVALID_OUTPUT_POWER:
    return "ERR_INVALID_OUTPUT_POWER";
  case RADIOLIB_ERR_NETWORK_NOT_JOINED:
	  return "RADIOLIB_ERR_NETWORK_NOT_JOINED";
  case RADIOLIB_ERR_DOWNLINK_MALFORMED:
    return "RADIOLIB_ERR_DOWNLINK_MALFORMED";
  case RADIOLIB_ERR_INVALID_REVISION:
    return "RADIOLIB_ERR_INVALID_REVISION";
  case RADIOLIB_ERR_INVALID_PORT:
    return "RADIOLIB_ERR_INVALID_PORT";
  case RADIOLIB_ERR_NO_RX_WINDOW:
    return "RADIOLIB_ERR_NO_RX_WINDOW";
  case RADIOLIB_ERR_INVALID_CID:
    return "RADIOLIB_ERR_INVALID_CID";
  case RADIOLIB_ERR_UPLINK_UNAVAILABLE:
    return "RADIOLIB_ERR_UPLINK_UNAVAILABLE";
  case RADIOLIB_ERR_COMMAND_QUEUE_FULL:
    return "RADIOLIB_ERR_COMMAND_QUEUE_FULL";
  case RADIOLIB_ERR_COMMAND_QUEUE_ITEM_NOT_FOUND:
    return "RADIOLIB_ERR_COMMAND_QUEUE_ITEM_NOT_FOUND";
  case RADIOLIB_ERR_JOIN_NONCE_INVALID:
    return "RADIOLIB_ERR_JOIN_NONCE_INVALID";
  case RADIOLIB_ERR_DWELL_TIME_EXCEEDED:
    return "RADIOLIB_ERR_DWELL_TIME_EXCEEDED";
  case RADIOLIB_ERR_CHECKSUM_MISMATCH:
    return "RADIOLIB_ERR_CHECKSUM_MISMATCH";
  case RADIOLIB_ERR_NO_JOIN_ACCEPT:
    return "RADIOLIB_ERR_NO_JOIN_ACCEPT";
  case RADIOLIB_LORAWAN_SESSION_RESTORED:
    return "RADIOLIB_LORAWAN_SESSION_RESTORED";
  case RADIOLIB_LORAWAN_NEW_SESSION:
    return "RADIOLIB_LORAWAN_NEW_SESSION";
  case RADIOLIB_ERR_NONCES_DISCARDED:
    return "RADIOLIB_ERR_NONCES_DISCARDED";
  case RADIOLIB_ERR_SESSION_DISCARDED:
    return "RADIOLIB_ERR_SESSION_DISCARDED";
  }
  return "See https://jgromes.github.io/RadioLib/group__status__codes.html";
}

// helper function to display any issues
void debug(bool failed, const __FlashStringHelper* message, int state, bool halt) {
  if(failed) {
    Serial.print(message);
    Serial.print(" - ");
    Serial.print(stateDecode(state));
    Serial.print(" (");
    Serial.print(state);
    Serial.println(")");
    while(halt) { delay(1); }
  }
}

// helper function to display a byte array
void arrayDump(uint8_t *buffer, uint16_t len) {
  for(uint16_t c = 0; c < len; c++) {
    char b = buffer[c];
    if(b < 0x10) { Serial.print('0'); }
    Serial.print(b, HEX);
  }
  Serial.println();
}

#endif
