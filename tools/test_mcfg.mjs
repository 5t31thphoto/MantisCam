// Generates blobs with the exact code the web page uses, then runs the C parser on them.
import { createRequire } from 'module';
import { writeFileSync } from 'fs';
import { execFileSync } from 'child_process';
import { fileURLToPath } from 'url';
import path from 'path';
import zlib from 'zlib';
const here = path.dirname(fileURLToPath(import.meta.url));
const require = createRequire(import.meta.url);
const M = require(path.join(here, '../website/mcfg.js'));

// zlib cross-check of the JS CRC
const probe = Buffer.from('MantisCam ✓ ünïcode');
if (M.crc32(probe) !== zlib.crc32(probe)) { console.error('JS crc32 != zlib'); process.exit(1); }

const cases = [
  { name: 'Porch', ap_ssid: 'Mantis_2_Hotspot', ap_pass: 'mantis33', st_ssid: '', st_pass: '',
    admin: 'Xy7#kq2LmZ9p', board: 'xiao_s3', framesize: 'VGA', quality: 12, brightness: 0, vflip: false, hmirror: true },
  { name: 'Café “quotes” \\ slash', ap_ssid: 'Garage ✓ cam', ap_pass: 'p@ss "with" \\ chars',
    st_ssid: 'Home WiFi 5G', st_pass: 'hunter22hunter22', admin: 'adminadmin', board: 'auto',
    framesize: 'UXGA', quality: 6, brightness: -2, vflip: true, hmirror: false },
];
let i = 0;
for (const c of cases) {
  const nonce = (0xC0FFEE00 + i) >>> 0;
  const blob = M.buildConfigBlob(c, nonce);
  const b = path.join('/tmp', `mcfg_${i}.bin`), j = path.join('/tmp', `mcfg_${i}.json`);
  writeFileSync(b, blob);
  writeFileSync(j, Buffer.from(new TextEncoder().encode(JSON.stringify(c))));
  console.log(`--- case ${i}`);
  console.log(execFileSync(path.join('/tmp', 'test_mcfg'), [b, j, nonce.toString(16)]).toString());
  i++;
}
// Too-large settings must throw rather than produce a truncated sector.
try { M.buildConfigBlob({ x: 'a'.repeat(5000) }); console.error('expected throw'); process.exit(1); }
catch (e) { console.log('ok   oversize settings refused:', e.message); }
