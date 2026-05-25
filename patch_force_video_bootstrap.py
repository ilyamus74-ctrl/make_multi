from pathlib import Path
import shutil
import re
import time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".html.bak_force_video_bootstrap_{int(time.time())}")
shutil.copy2(p, backup)
print(f"backup: {backup}")

# Remove previous bootstrap if already inserted.
s = re.sub(
    r"\n<!-- FORCE_VIDEO_BOOTSTRAP_START -->.*?<!-- FORCE_VIDEO_BOOTSTRAP_END -->\n",
    "\n",
    s,
    flags=re.S
)

bootstrap = r'''
<!-- FORCE_VIDEO_BOOTSTRAP_START -->
<script>
(function () {
  function log(msg, obj) {
    try { console.log('[video-bootstrap]', msg, obj || ''); } catch (_) {}
  }

  function streamUrl() {
    return 'http://' + location.hostname + ':8080/stream?ts=' + Date.now();
  }

  function pickImage() {
    var ids = ['mjpegFrame', 'mjpegImg'];
    var candidates = [];

    for (var i = 0; i < ids.length; i++) {
      var el = document.getElementById(ids[i]);
      if (el && String(el.tagName || '').toUpperCase() === 'IMG') {
        candidates.push(el);
      }
    }

    if (!candidates.length) return null;

    for (var j = 0; j < candidates.length; j++) {
      var e = candidates[j];
      var r = e.getBoundingClientRect ? e.getBoundingClientRect() : null;
      if (r && r.width > 10 && r.height > 10) return e;
    }

    return candidates[0];
  }

  function setStatus(text) {
    var ids = ['streamStatus', 'streamStatusChip', 'mjpegStatus'];
    for (var i = 0; i < ids.length; i++) {
      var el = document.getElementById(ids[i]);
      if (el) {
        el.textContent = text;
        return;
      }
    }
  }

  function bootVideo() {
    var img = pickImage();
    if (!img) {
      log('no image element yet');
      return;
    }

    var current = String(img.getAttribute('src') || img.src || '');
    if (!current || current.indexOf('/stream') < 0) {
      var url = streamUrl();
      img.src = url;
      setStatus('STREAM BOOT');
      log('stream src set', url);
    }

    img.onerror = function () {
      setStatus('STREAM RETRY');
      setTimeout(function () {
        img.src = streamUrl();
      }, 1200);
    };

    img.onload = function () {
      setStatus('LIVE');
    };
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', bootVideo);
  } else {
    bootVideo();
  }

  setTimeout(bootVideo, 500);
  setTimeout(bootVideo, 1500);

  // Safety: if some JS later clears src, restore it.
  setInterval(function () {
    var img = pickImage();
    if (!img) return;
    var current = String(img.getAttribute('src') || img.src || '');
    if (!current || current.indexOf('/stream') < 0) bootVideo();
  }, 5000);
})();
</script>
<!-- FORCE_VIDEO_BOOTSTRAP_END -->
'''

if '</body>' not in s:
    raise SystemExit("ERROR: </body> not found")

s = s.replace('</body>', bootstrap + '\n</body>', 1)

p.write_text(s, encoding="utf-8")
print("OK: inserted force video bootstrap")
