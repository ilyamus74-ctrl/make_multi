from pathlib import Path
import shutil

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(".html.bak_remove_duplicate_zoomtest")
if not backup.exists():
    shutil.copy2(p, backup)
    print(f"backup: {backup}")

old = '''  async function zoomTest(cmd, holdMs) {
    const res = await apiPostJson('/api/zoom_test', { cmd, hold_ms: holdMs });
    ptzLog('ZOOM TEST', res);
    $('zoomAprilTagCalibStatus').textContent = `test cmd=${cmd} hold=${holdMs}`;
  }
'''

new = '''  // zoomTest(cmd, holdMs) is defined above and uses browser_ws/sendLine().
  // Do not redefine it here; the old /api/zoom_test path does not control zoom reliably.
'''

if old not in s:
    raise SystemExit("duplicate old zoomTest block not found")

s = s.replace(old, new, 1)

p.write_text(s, encoding="utf-8")
print("patched web/index.html")
