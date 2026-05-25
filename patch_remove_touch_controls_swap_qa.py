from pathlib import Path
import re
import shutil
import time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".html.bak_remove_touch_controls_swap_qa_{int(time.time())}")
shutil.copy2(p, backup)
print(f"backup: {backup}")

# 1) Remove visible on-screen touch arrows block.
touch_re = re.compile(
    r'\n\s*<div class="touch-controls" id="touchControls">.*?</div>\s*',
    re.S
)

s2, n = touch_re.subn("\n", s, count=1)
if n != 1:
    raise SystemExit(f"ERROR: touchControls block not removed, replacements={n}")

s = s2
print("OK: removed #touchControls HTML block")

# 2) Add hard CSS guard in case old browser/cache or duplicate DOM appears.
css_marker = "</style>"
guard_css = """
/* Touch overlay removed: keep forced hidden if stale cached DOM exists. */
#touchControls,
.touch-controls {
  display: none !important;
  visibility: hidden !important;
  pointer-events: none !important;
  opacity: 0 !important;
}
"""

if "#touchControls," not in s:
    if css_marker not in s:
        print("WARN: </style> not found, CSS guard not inserted")
    else:
        s = s.replace(css_marker, guard_css + "\n" + css_marker, 1)
        print("OK: added #touchControls CSS guard")
else:
    print("SKIP: #touchControls CSS guard already present")

# 3) Swap Q/A sample directions.
old_q = """    if (isQ) {
      keyboardZoomSampleStep(-1, 'Q_WIDE');
      return;
    }
    if (isA) {
      keyboardZoomSampleStep(1, 'A_TELE');
      return;
    }
"""

new_q = """    if (isQ) {
      keyboardZoomSampleStep(1, 'Q_TELE');
      return;
    }
    if (isA) {
      keyboardZoomSampleStep(-1, 'A_WIDE');
      return;
    }
"""

if old_q not in s:
    raise SystemExit("ERROR: Q/A sample direction block not found")

s = s.replace(old_q, new_q, 1)
print("OK: swapped Q/A sample directions")

p.write_text(s, encoding="utf-8")
print("DONE")
