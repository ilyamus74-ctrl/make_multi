from pathlib import Path
import re
import shutil
import time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".html.bak_remove_dead_legacy_handlers_{int(time.time())}")
shutil.copy2(p, backup)
print(f"backup: {backup}")

# 1) Remove orphan settingsToggleBtn/settingsCollapseBtn/controlsSpoiler handlers.
# These crash JS because the visible controlsSpoiler UI was removed.
patterns = [
    r"\n\s*\$\('settingsToggleBtn'\)\.onclick\s*=\s*\(\)\s*=>\s*\{[\s\S]*?\n\s*\};",
    r"\n\s*\$\('settingsCollapseBtn'\)\.onclick\s*=\s*\(\)\s*=>\s*\{[\s\S]*?\n\s*\};",
    r"\n\s*\$\('controlsSpoiler'\)\.addEventListener\('toggle',\s*\(\)\s*=>\s*\{[\s\S]*?\n\s*\}\);",
]

for pat in patterns:
    s2, n = re.subn(pat, "\n", s, count=1)
    if n:
        print(f"OK: removed legacy handler pattern: {pat[:45]}...")
        s = s2
    else:
        print(f"WARN: pattern not found/already removed: {pat[:45]}...")

# 2) Remove remaining #touchControls CSS guard block.
s2, n = re.subn(
    r"\n\s*/\* Touch overlay removed:[\s\S]*?#touchControls,\s*\n\s*\.touch-controls\s*\{[^}]*\}\s*",
    "\n",
    s,
    count=1
)
if n:
    s = s2
    print("OK: removed touchControls CSS guard block")
else:
    print("WARN: touchControls CSS guard block not found")

# Also remove bare #touchControls/.touch-controls guard if comment was already changed/removed.
s2, n = re.subn(
    r"\n\s*#touchControls,\s*\n\s*\.touch-controls\s*\{[^}]*\}\s*",
    "\n",
    s,
    count=1
)
if n:
    s = s2
    print("OK: removed bare touchControls CSS guard")

# 3) Remove leftover comment that makes grep noisy.
s = s.replace(
    "  // Legacy keyboardZoomPulse block removed. Q/A are handled by keyboardZoomSampleStep().\n",
    ""
)

p.write_text(s, encoding="utf-8")
print("DONE")
