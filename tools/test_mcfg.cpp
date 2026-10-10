// Host test: the firmware's blob parser (src/cam/mcfg.h) must accept exactly
// what website/mcfg.js produces and reject corrupted variants.
// Usage: test_mcfg <blob.bin> <expected-json-file> <expected-nonce-hex>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <string>
#include "../src/cam/mcfg.h"

static std::vector<uint8_t> rd(const char *p) {
  FILE *f = fopen(p, "rb"); if (!f) { perror(p); exit(2); }
  std::vector<uint8_t> v; int c; while ((c = fgetc(f)) != EOF) v.push_back((uint8_t)c);
  fclose(f); return v;
}
static int fails = 0;
#define CHECK(c, m) do { if (!(c)) { printf("FAIL: %s\n", m); fails++; } else printf("ok   %s\n", m); } while (0)

int main(int argc, char **argv) {
  if (argc != 4) { fprintf(stderr, "usage\n"); return 2; }
  std::vector<uint8_t> blob = rd(argv[1]);
  std::vector<uint8_t> exp  = rd(argv[2]);
  uint32_t nonce = (uint32_t) strtoul(argv[3], NULL, 16);

  CHECK(blob.size() == MCFG_SIZE, "blob is exactly one 4 KB sector");
  mcfg_header_t h; const uint8_t *json = NULL;
  mcfg_result_t r = mcfg_parse(blob.data(), blob.size(), &h, &json);
  CHECK(r == MCFG_OK, "valid blob parses");
  CHECK(h.nonce == nonce, "nonce round-trips");
  CHECK(h.len == exp.size() && memcmp(json, exp.data(), exp.size()) == 0, "JSON payload byte-identical");
  bool padded = true;
  for (size_t i = MCFG_HDR_SIZE + h.len; i < blob.size(); i++) if (blob[i] != 0xFF) padded = false;
  CHECK(padded, "padding is 0xFF (erased flash)");

  { auto b = blob; b[MCFG_HDR_SIZE + 3] ^= 0x01;
    CHECK(mcfg_parse(b.data(), b.size(), &h, &json) == MCFG_BAD_CRC, "flipped payload bit -> crc mismatch"); }
  { auto b = blob; b[0] = 'X';
    CHECK(mcfg_parse(b.data(), b.size(), &h, &json) == MCFG_BAD_MAGIC, "bad magic rejected"); }
  { auto b = blob; b[4] = 9;
    CHECK(mcfg_parse(b.data(), b.size(), &h, &json) == MCFG_BAD_VERSION, "future version rejected"); }
  { auto b = blob; b[6] = 0xFF; b[7] = 0x7F;
    CHECK(mcfg_parse(b.data(), b.size(), &h, &json) == MCFG_BAD_LEN, "oversize length rejected"); }
  { std::vector<uint8_t> b(MCFG_SIZE, 0xFF);
    CHECK(mcfg_parse(b.data(), b.size(), &h, &json) == MCFG_EMPTY, "erased sector = empty (not an error)"); }

  // Known-answer CRC test: CRC-32("123456789") = 0xCBF43926
  CHECK(mcfg_crc32((const uint8_t *)"123456789", 9) == 0xCBF43926u, "CRC-32 known answer");

  printf(fails ? "\n%d FAILED\n" : "\nall passed\n", fails);
  return fails ? 1 : 0;
}
