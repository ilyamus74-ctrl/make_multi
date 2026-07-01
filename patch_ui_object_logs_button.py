from pathlib import Path
import time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8", errors="ignore")

bak = p.with_suffix(p.suffix + f".bak_object_logs_button_{int(time.time())}")
bak.write_text(s, encoding="utf-8")

script = r'''
<script>
// OBJECT_TRACKING_LOGS_BUTTON_START
(function () {
  const $ = (id) => document.getElementById(id);

  function ensureLogsButton() {
    if ($('objectTrackingLogsBtn')) return;

    const panel = $('objectPresetPanel') || $('ptzPanel') || document.querySelector('.controls-panel');
    if (!panel) return;

    const btn = document.createElement('button');
    btn.id = 'objectTrackingLogsBtn';
    btn.className = 'small-btn';
    btn.textContent = 'LOGS';
    btn.title = 'Open object tracking logs in a new window';

    btn.addEventListener('click', (e) => {
      e.preventDefault();

      const url = `${location.protocol}//${location.hostname}:8091/`;
      window.open(url, '_blank', 'noopener,noreferrer');
    });

    panel.appendChild(btn);
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', ensureLogsButton, { once: true });
  } else {
    ensureLogsButton();
  }

  setTimeout(ensureLogsButton, 500);
  setTimeout(ensureLogsButton, 1500);
  setTimeout(ensureLogsButton, 3000);
})();
// OBJECT_TRACKING_LOGS_BUTTON_END
</script>
'''

if "OBJECT_TRACKING_LOGS_BUTTON_START" not in s:
    if "</body>" not in s:
        raise SystemExit("ERROR: </body> not found")
    s = s.replace("</body>", script + "\n</body>", 1)
    p.write_text(s, encoding="utf-8")
    print("OK patched web/index.html")
    print("Backup:", bak)
else:
    print("OK already patched")
