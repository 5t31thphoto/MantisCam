Import("env")
import os
from os.path import join, isfile

def after_build(source, target, env):
    """Merge bootloader + partitions + app into a single 0x0 flashable image."""
    build_dir = env.subst("$BUILD_DIR")
    project_dir = env.subst("$PROJECT_DIR")
    out_bin = join(project_dir, "firmware_merged.bin")

    # Locate the standard PlatformIO outputs
    bootloader = join(build_dir, "bootloader.bin")
    partitions = join(build_dir, "partitions.bin")
    firmware   = join(build_dir, "firmware.bin")
    boot_app0  = join(env.subst("$PROJECT_PACKAGES_DIR"),
                      "framework-arduinoespressif32", "tools", "partitions", "boot_app0.bin")

    # Fallbacks for different platform versions
    if not isfile(boot_app0):
        candidates = [
            join(env.subst("$PROJECT_PACKAGES_DIR"), "framework-arduinoespressif32",
                 "tools", "partitions", "boot_app0.bin"),
            join(build_dir, "boot_app0.bin"),
        ]
        for c in candidates:
            if isfile(c):
                boot_app0 = c
                break

    if not (isfile(bootloader) and isfile(partitions) and isfile(firmware)):
        print("[merge] Skipping merge – required bins not found yet")
        return

    # Typical offsets for M5Core2 / ESP32 Arduino
    # 0x1000 bootloader, 0x8000 partitions, 0xe000 boot_app0, 0x10000 app
    cmd = [
        env.subst("$PYTHONEXE"), "-m", "esptool",
        "--chip", "esp32",
        "merge_bin",
        "-o", out_bin,
        "--flash_mode", "qio",
        "--flash_freq", "80m",
        "--flash_size", "16MB",
        "0x1000", bootloader,
        "0x8000", partitions,
        "0xe000", boot_app0 if isfile(boot_app0) else partitions,  # harmless fallback
        "0x10000", firmware,
    ]
    print("[merge] Running:", " ".join(cmd))
    env.Execute(" ".join('"%s"' % c if " " in c else c for c in cmd))
    if isfile(out_bin):
        size = os.path.getsize(out_bin)
        print(f"[merge] Created {out_bin} ({size // 1024} KB)")
    else:
        print("[merge] WARNING: merge_bin did not produce output")

env.AddPostAction("$BUILD_DIR/firmware.bin", after_build)
