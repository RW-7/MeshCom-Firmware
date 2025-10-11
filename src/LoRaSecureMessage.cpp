#include "LoRaSecureMessage.h"

LoRaSecureMessage::LoRaSecureMessage(const uint8_t *tdk, size_t keyLen)
: _key(tdk), _keyLen(keyLen) {}

void LoRaSecureMessage::computeHmac(LoRaPacket &pkt) {
  uint8_t fullMac[32];

#if defined(LORA_SEC_USE_MBEDTLS)
  // --- ESP32: use mbedTLS backend ---
  mbedtls_md_context_t ctx;
  const mbedtls_md_info_t *info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);

  mbedtls_md_init(&ctx);
  mbedtls_md_setup(&ctx, info, 1);
  mbedtls_md_hmac_starts(&ctx, _key, _keyLen);

  mbedtls_md_hmac_update(&ctx, (const unsigned char*)pkt.src.c_str(), pkt.src.length());
  mbedtls_md_hmac_update(&ctx, (const unsigned char*)"\0", 1);
  mbedtls_md_hmac_update(&ctx, (const unsigned char*)pkt.dst.c_str(), pkt.dst.length());
  mbedtls_md_hmac_update(&ctx, (const unsigned char*)"\0", 1);
  mbedtls_md_hmac_update(&ctx, &pkt.cmd, sizeof(pkt.cmd));
  mbedtls_md_hmac_update(&ctx, &pkt.pin, sizeof(pkt.pin));
  mbedtls_md_hmac_update(&ctx, &pkt.val, sizeof(pkt.val));
  mbedtls_md_hmac_update(&ctx, (const unsigned char*)&pkt.nonce, sizeof(pkt.nonce));

  mbedtls_md_hmac_finish(&ctx, fullMac);
  mbedtls_md_free(&ctx);

#elif defined(LORA_SEC_USE_NRFCRYPTO)
  // --- nRF52: use Adafruit_nRFCrypto backend ---
  nRFCrypto.begin();

  nRFCrypto_Hmac hmac;
  hmac.begin(CRYS_HASH_SHA256_mode, (uint8_t*)_key, _keyLen);

  hmac.update((uint8_t*)pkt.src.c_str(), pkt.src.length());
  hmac.update((uint8_t*)"\0", 1);
  hmac.update((uint8_t*)pkt.dst.c_str(), pkt.dst.length());
  hmac.update((uint8_t*)"\0", 1);
  hmac.update(&pkt.cmd, sizeof(pkt.cmd));
  hmac.update(&pkt.pin, sizeof(pkt.pin));
  hmac.update(&pkt.val, sizeof(pkt.val));
  hmac.update((uint8_t*)&pkt.nonce, sizeof(pkt.nonce));
  hmac.end((uint32_t*)fullMac);

  nRFCrypto.end();
#endif

  // Truncate to 16 bytes for compact LoRa payloads
  memcpy(pkt.mac, fullMac, 16);
}

void LoRaSecureMessage::signPacket(LoRaPacket &pkt) {
    computeHmac(pkt);
}

bool LoRaSecureMessage::verifyPacket(const LoRaPacket &pkt, uint32_t lastNonce) {
    if (pkt.nonce <= lastNonce) {
        Serial.println(F("[SEC] Reject: replayed nonce"));
        return false;
    }

    // Recompute expected MAC
    LoRaPacket tmp = pkt;
    computeHmac(tmp);

    if (memcmp(pkt.mac, tmp.mac, 16) == 0) {
        Serial.println(F("[SEC] Packet verified OK"));
        return true;
    } else {
        Serial.println(F("[SEC] Reject: MAC mismatch"));
        return false;
    }
}
