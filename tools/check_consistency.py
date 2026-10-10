"""Fails if the web flasher and camera firmware disagree on board ids or the settings offset."""
import re, sys, pathlib
root = pathlib.Path(__file__).resolve().parent.parent
fw   = (root / 'src/cam/main.cpp').read_text()
html = (root / 'website/index.html').read_text()
js   = (root / 'website/mcfg.js').read_text()
err = []
hdr  = (root / 'src/cam/mcfg.h').read_text()
csv  = (root / 'partitions/mantis_cam.csv').read_text()
fw_ids = set(re.findall(r'#define CAM_ID "([a-z0-9_]+)"', fw))
env_ids = set(re.findall(r'^\[env:cam-([a-z0-9_]+)\]', (root / 'platformio.ini').read_text(), re.M))
if fw_ids != env_ids:
    err.append(f"firmware boards vs platformio envs differ: {sorted(fw_ids ^ env_ids)}")
page_ids = set(re.findall(r'<option value="([a-z0-9_]+)"', html)) - {'', 'QVGA','VGA','SVGA','XGA','HD','SXGA','UXGA'}
if fw_ids != page_ids:
    err.append(f"board ids differ: only firmware={sorted(fw_ids-page_ids)} only page={sorted(page_ids-fw_ids)}")
off = lambda pat, txt: int(re.search(pat, txt).group(1), 16)
offs = {'mcfg.h': off(r'MCFG_OFFSET\s+(0x[0-9A-Fa-f]+)', hdr),
        'mcfg.js': off(r'MCFG_OFFSET = (0x[0-9A-Fa-f]+)', js),
        'csv': int(re.search(r'^mcfg\s*,[^,]*,[^,]*,\s*(0x[0-9A-Fa-f]+)', csv, re.M).group(1), 16)}
if len(set(offs.values())) != 1: err.append(f"mcfg offset mismatch: {offs}")
print(f"{len(fw_ids)} board ids match; mcfg offset {hex(offs['csv'])} consistent" if not err else "\n".join(err))
sys.exit(1 if err else 0)
