from pathlib import Path
import time
import re

ROOT = Path('/root/new_yolo8')
WEB = ROOT / 'web' / 'index.html'

if not WEB.exists():
    raise SystemExit(f'missing: {WEB}')

s = WEB.read_text(encoding='utf-8', errors='ignore')
backup = WEB.with_suffix(WEB.suffix + f'.bak_ui_select_dynamic_options_v2_{int(time.time())}')
backup.write_text(s, encoding='utf-8')

helper = r'''
<script>
// PTZ_SELECT_DYNAMIC_OPTION_FIX_START
(function () {
  window.ptzEnsureSelectOption = window.ptzEnsureSelectOption || function (id, value, label) {
    const el = document.getElementById(id);

    if (!el) return null;

    const v = String(value ?? '');
    const txt = String(label ?? value ?? '');

    if (v !== '') {
      let found = false;

      for (const opt of Array.from(el.options || [])) {
        if (String(opt.value) === v) {
          found = true;
          break;
        }
      }

      if (!found) {
        const opt = document.createElement('option');
        opt.value = v;
        opt.textContent = txt || v;
        opt.dataset.ptzDynamicOption = '1';
        el.appendChild(opt);
      }
    }

    el.value = v;
    return el.value;
  };
})();
// PTZ_SELECT_DYNAMIC_OPTION_FIX_END
</script>
'''

changed = []

if 'PTZ_SELECT_DYNAMIC_OPTION_FIX_START' not in s:
    idx = s.find('<script>')
    if idx < 0:
        raise SystemExit('no <script> tag found')
    s = s[:idx] + helper + '\n' + s[idx:]
    changed.append('inserted global dynamic select option helper')
else:
    changed.append('helper already present')

# Patch direct select assignments so custom values like LIMIT=8 do not render as blank.
target_ids = [
    'operatorDetectionLimit',
    'operatorDetectEvery',
    'operatorDetectionAreaMode',
]

lines = s.splitlines()
out = []
replace_count = 0

for line in lines:
    original = line
    stripped = line.strip()

    if 'ptzEnsureSelectOption' in line:
        out.append(line)
        continue

    replaced = False

    for target in target_ids:
        token = f"$('{target}').value"

        if token in line and '.value =' in line:
            indent = re.match(r'^(\s*)', line).group(1)
            rhs = line.split('.value =', 1)[1].strip()

            # Preserve only simple one-line assignments.
            if rhs.endswith(';'):
                rhs = rhs[:-1].strip()

            if rhs:
                line = (
                    f"{indent}window.ptzEnsureSelectOption "
                    f"? window.ptzEnsureSelectOption('{target}', {rhs}, {rhs}) "
                    f": ($('{target}').value = {rhs});"
                )
                replace_count += 1
                replaced = True
                break

    out.append(line)

s2 = '\n'.join(out) + ('\n' if s.endswith('\n') else '')

WEB.write_text(s2, encoding='utf-8')

print('OK patched web/index.html dynamic select options v2')
print('Backup:', backup)
print('Changes:')
for item in changed:
    print(' -', item)
print('Replaced select assignments:', replace_count)
print('Targets:', ', '.join(target_ids))