from pathlib import Path
import shutil, time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".html.bak_fix_ptz_samples_recursion_{int(time.time())}")
shutil.copy2(p, backup)
print("Backup:", backup)

old1 = "  window.initPtzSpeedTuneSamples = async () => initPtzSpeedTuneSamples();"
new1 = "  window.initPtzSpeedTuneSamples = initPtzSpeedTuneSamples;"

old2 = "  window.refreshPtzTuneZoomSamples = async function refreshPtzTuneZoomSamples(){ return initPtzSpeedTuneSamples?.(); };"
new2 = "  window.refreshPtzTuneZoomSamples = initPtzSpeedTuneSamples;"

changed = False

if old1 in s:
    s = s.replace(old1, new1, 1)
    print("OK: fixed window.initPtzSpeedTuneSamples recursion")
    changed = True
else:
    print("WARN: old initPtzSpeedTuneSamples wrapper not found")

if old2 in s:
    s = s.replace(old2, new2, 1)
    print("OK: fixed window.refreshPtzTuneZoomSamples recursion risk")
    changed = True
else:
    print("WARN: old refreshPtzTuneZoomSamples wrapper not found")

if not changed:
    raise SystemExit("ERROR: nothing patched")

p.write_text(s, encoding="utf-8")
print("DONE")
