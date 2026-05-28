"""
PlatformIO post-script: Redirect -Map to a temp ASCII path.
Workaround for ESP32 GCC ld.exe failing on non-ASCII (Korean) paths.
"""
Import("env")

import tempfile, os

# Replace the map file path with one in the temp directory (always ASCII)
map_path = os.path.join(tempfile.gettempdir(), "firmware.map")

# Filter existing LINKFLAGS and replace the -Map flag
new_flags = []
for f in env.get("LINKFLAGS", []):
    f_str = str(f)
    if ".map" in f_str or "-Map" in f_str:
        continue
    new_flags.append(f)

new_flags.append('-Wl,-Map,' + map_path)
env.Replace(LINKFLAGS=new_flags)
