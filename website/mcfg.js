// MantisCam flash-time settings blob.
// Builds the 4 KB sector the web flasher writes to the camera's "mcfg"
// partition. Format contract: src/cam/mcfg.h (keep both in sync).
(function (root) {
  'use strict';

  var MCFG_OFFSET = 0x310000;
  var MCFG_SIZE = 0x1000;
  var MCFG_HDR = 16;
  var MCFG_VERSION = 1;

  var CRC_TABLE = (function () {
    var t = new Uint32Array(256);
    for (var n = 0; n < 256; n++) {
      var c = n;
      for (var k = 0; k < 8; k++) c = (c & 1) ? (0xEDB88320 ^ (c >>> 1)) : (c >>> 1);
      t[n] = c >>> 0;
    }
    return t;
  })();

  function crc32(bytes) {
    var crc = 0xFFFFFFFF;
    for (var i = 0; i < bytes.length; i++) crc = CRC_TABLE[(crc ^ bytes[i]) & 0xFF] ^ (crc >>> 8);
    return (crc ^ 0xFFFFFFFF) >>> 0;
  }

  function randomNonce() {
    var a = new Uint32Array(1);
    (root.crypto || require('crypto').webcrypto).getRandomValues(a);
    return a[0] >>> 0;
  }

  // settings: plain object, e.g. {name, ap_ssid, ap_pass, st_ssid, st_pass,
  // admin, board, framesize, quality, brightness, vflip, hmirror}
  function buildConfigBlob(settings, nonce) {
    var json = new TextEncoder().encode(JSON.stringify(settings));
    if (json.length > MCFG_SIZE - MCFG_HDR) throw new Error('settings too large');
    var buf = new Uint8Array(MCFG_SIZE);
    buf.fill(0xFF);
    var dv = new DataView(buf.buffer);
    buf.set([0x4D, 0x43, 0x46, 0x47], 0);              // "MCFG"
    dv.setUint16(4, MCFG_VERSION, true);
    dv.setUint16(6, json.length, true);
    dv.setUint32(8, crc32(json), true);
    dv.setUint32(12, (nonce === undefined ? randomNonce() : nonce) >>> 0, true);
    buf.set(json, MCFG_HDR);
    return buf;
  }

  var api = { MCFG_OFFSET: MCFG_OFFSET, MCFG_SIZE: MCFG_SIZE, crc32: crc32, buildConfigBlob: buildConfigBlob };
  root.MantisMcfg = api;
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
})(typeof window !== 'undefined' ? window : globalThis);
