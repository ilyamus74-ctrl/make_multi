from pathlib import Path
import shutil, time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".html.bak_disable_autoclear_single_run_{int(time.time())}")
shutil.copy2(p, backup)
print("Backup:", backup)

s2 = s.replace(
'''    status(`starting fresh calibration for ${activeLabel} id=${a.tag_id}...`);
    await save();
    await requestFreshZoomCalibSession();
    const res = await postJson('/api/zoom_calibration', payload());
''',
'''    status(`starting calibration for ${activeLabel} id=${a.tag_id}...`);
    await save();
    const res = await postJson('/api/zoom_calibration', payload());
'''
)

if s2 == s:
    print("SKIP/WARN: single-run autoclear block not found; maybe already clean")
else:
    p.write_text(s2, encoding="utf-8")
    print("OK: single RUN buttons no longer clear old profiles")
