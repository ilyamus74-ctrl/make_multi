from pathlib import Path
import shutil, time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".html.bak_keyboard_focus_ptz_mode_{int(time.time())}")
shutil.copy2(p, backup)
print("Backup:", backup)

old = '''  function shouldIgnoreKeyboardEvent(e) {
    const tag = e.target?.tagName;
    if (!tag) return false;
    const editableTags = ['INPUT', 'TEXTAREA', 'SELECT'];
    if (editableTags.includes(tag)) return true;
    return Boolean(e.target?.isContentEditable);
  }
'''

new = '''  function shouldIgnoreKeyboardEvent(e) {
    const key = String(e.key || '').toLowerCase();
    const code = String(e.code || '');

    const isArrow =
      key === 'arrowleft' ||
      key === 'arrowright' ||
      key === 'arrowup' ||
      key === 'arrowdown';

    /*
     * In PTZ mode arrows are safety/operator controls.
     * After editing ZOOM CALIB/PTZ fields, focus may stay on INPUT/SELECT.
     * Do not let focused inputs steal Arrow keys from PTZ control.
     */
    if (isArrow && $('controlMode')?.value === 'ptz') {
      try {
        const el = document.activeElement;
        const tag = String(el?.tagName || '').toUpperCase();
        if (tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT' || el?.isContentEditable) {
          el.blur?.();
        }
      } catch (_) {}
      return false;
    }

    /*
     * Q/A zoom hotkeys should not fire while typing/editing fields.
     */
    const isZoomHotkey =
      key === 'q' ||
      key === 'a' ||
      code === 'KeyQ' ||
      code === 'KeyA';

    const tag = String(e.target?.tagName || '').toUpperCase();
    const editableTags = ['INPUT', 'TEXTAREA', 'SELECT'];
    if (editableTags.includes(tag) || Boolean(e.target?.isContentEditable)) {
      return true;
    }

    return false;
  }
'''

if old not in s:
    raise SystemExit("ERROR: shouldIgnoreKeyboardEvent exact block not found")

s = s.replace(old, new, 1)
p.write_text(s, encoding="utf-8")

print("OK: keyboard focus guard patched")
