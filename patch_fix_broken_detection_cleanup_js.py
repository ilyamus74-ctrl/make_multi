from pathlib import Path
import shutil
import time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".html.bak_fix_broken_detection_cleanup_{int(time.time())}")
shutil.copy2(p, backup)
print(f"backup: {backup}")

bad = """
    syncOperatorModelSelect(normalizedModels, select.value);
  }

  function syncOperatorModelSelect(models, currentModel) {
"""

good = """
  function syncOperatorModelSelect(models, currentModel) {
"""

if bad not in s:
    raise SystemExit("ERROR: broken dangling syncOperatorModelSelect block not found")

s = s.replace(bad, good, 1)

p.write_text(s, encoding="utf-8")
print("OK: removed dangling broken JS fragment before syncOperatorModelSelect()")
