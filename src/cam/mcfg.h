// ============================================================
//  MantisCam flash-time config blob ("mcfg" partition)
//
//  The web flasher writes this 4 KB sector at MCFG_OFFSET alongside the
//  firmware. On boot the camera validates it, applies it to NVS, and wipes it.
//
//  Layout (little-endian):
//    0   char[4]  magic   "MCFG"
//    4   u16      version  MCFG_VERSION
//    6   u16      len      length of JSON payload in bytes
//    8   u32      crc      CRC-32 (zlib/IEEE) of the JSON payload
//    12  u32      nonce    random per flash; lets us tell configs apart
//    16  u8[len]  JSON     UTF-8, e.g. {"name":"Porch","ap_ssid":"Mantis_2_Hotspot",...}
//    ..           0xFF padding to 4096
//
//  KEEP IN SYNC with website/index.html (buildConfigBlob) and
//  partitions/mantis_cam.csv. This header is plain C so it can be unit-tested
//  on a desktop compiler (see tools/test_mcfg.cpp).
// ============================================================
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define MCFG_OFFSET      0x310000u
#define MCFG_SIZE        0x1000u
#define MCFG_HDR_SIZE    16u
#define MCFG_VERSION     1u
#define MCFG_MAX_JSON    (MCFG_SIZE - MCFG_HDR_SIZE)

typedef struct {
  uint16_t version;
  uint16_t len;
  uint32_t crc;
  uint32_t nonce;
} mcfg_header_t;

// Standard CRC-32 (same as zlib.crc32 / JavaScript implementation on the page).
static inline uint32_t mcfg_crc32(const uint8_t *data, size_t len) {
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
  }
  return ~crc;
}

static inline uint16_t mcfg_rd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static inline uint32_t mcfg_rd32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

// Result codes so the boot log says exactly why a blob was ignored.
typedef enum {
  MCFG_OK = 0,
  MCFG_EMPTY,        // erased sector (all 0xFF) - normal after first apply
  MCFG_BAD_MAGIC,
  MCFG_BAD_VERSION,
  MCFG_BAD_LEN,
  MCFG_BAD_CRC,
} mcfg_result_t;

static inline const char *mcfg_result_str(mcfg_result_t r) {
  switch (r) {
    case MCFG_OK:          return "ok";
    case MCFG_EMPTY:       return "empty";
    case MCFG_BAD_MAGIC:   return "bad magic";
    case MCFG_BAD_VERSION: return "unsupported version";
    case MCFG_BAD_LEN:     return "bad length";
    case MCFG_BAD_CRC:     return "crc mismatch";
  }
  return "?";
}

// Validate a full 4 KB sector image. On MCFG_OK, *json points into buf.
static inline mcfg_result_t mcfg_parse(const uint8_t *buf, size_t bufLen,
                                       mcfg_header_t *hdr, const uint8_t **json) {
  if (bufLen < MCFG_HDR_SIZE) return MCFG_BAD_LEN;
  if (buf[0] == 0xFF && buf[1] == 0xFF && buf[2] == 0xFF && buf[3] == 0xFF) return MCFG_EMPTY;
  if (memcmp(buf, "MCFG", 4) != 0) return MCFG_BAD_MAGIC;
  hdr->version = mcfg_rd16(buf + 4);
  hdr->len     = mcfg_rd16(buf + 6);
  hdr->crc     = mcfg_rd32(buf + 8);
  hdr->nonce   = mcfg_rd32(buf + 12);
  if (hdr->version != MCFG_VERSION) return MCFG_BAD_VERSION;
  if (hdr->len == 0 || hdr->len > MCFG_MAX_JSON || MCFG_HDR_SIZE + hdr->len > bufLen) return MCFG_BAD_LEN;
  if (mcfg_crc32(buf + MCFG_HDR_SIZE, hdr->len) != hdr->crc) return MCFG_BAD_CRC;
  *json = buf + MCFG_HDR_SIZE;
  return MCFG_OK;
}
