"""Sanity-check a merged camera image before it is published to the web flasher."""
import sys, os
img, boot_off = sys.argv[1], int(sys.argv[2], 16)
d = open(img, 'rb').read()
assert d[boot_off] == 0xE9, f"{img}: no bootloader at {boot_off:#x}"
assert d[0x8000:0x8002] == b'\xaa\x50', f"{img}: no partition table at 0x8000"
assert b'mcfg' in d[0x8000:0x9000], f"{img}: partition table has no mcfg slot"
assert d[0x10000] == 0xE9, f"{img}: no app at 0x10000"
assert len(d) <= 0x310000, f"{img}: image ({len(d):#x}) overlaps settings slot at 0x310000"
print(f"OK {os.path.basename(img)} {len(d)} bytes")
