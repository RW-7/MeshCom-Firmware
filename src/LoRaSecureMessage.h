#pragma once
#include <Arduino.h>

#if defined(ESP32)
  #include <mbedtls/md.h>
  #define LORA_SEC_USE_MBEDTLS
#elif defined(NRF52_SERIES)
  #include <Adafruit_nRFCrypto.h>
  #define LORA_SEC_USE_NRFCRYPTO
#else
  #error "Unsupported platform: LoRaSecureMessage requires ESP32 or nRF52"
#endif

struct LoRaPacket {
    String src;       // Source callsign
    String dst;       // Destination callsign
    uint8_t cmd;      // Command zB 1= set pin
    uint8_t pin;      // Pin number MCP Chip A0 to B7 = 0 to 15
    uint8_t val;      // Pin value 0 or 1
    uint32_t nonce;   // Monotonic counter to prevent replay attacks. Save it in NVS Flash. Checking is not done in verifyPacket
    uint8_t mac[16];  // Truncated HMAC
};

class LoRaSecureMessage {
public:
    LoRaSecureMessage(const uint8_t *tdk, size_t keyLen);

    void signPacket(LoRaPacket &pkt);
    bool verifyPacket(const LoRaPacket &pkt, uint32_t lastNonce);

private:
    const uint8_t *_key;
    size_t _keyLen;

    void computeHmac(LoRaPacket &pkt);
};
