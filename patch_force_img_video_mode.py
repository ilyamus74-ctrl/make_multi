from pathlib import Path
import re
import shutil
import time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".html.bak_force_img_video_mode_{int(time.time())}")
shutil.copy2(p, backup)
print(f"backup: {backup}")

# Remove old bootstrap if exists.
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
  function streamUrl() {
    return 'http://' + location.hostname + ':8080/stream?ts=' + Date.now();
  }

  function setStatus(text) {
    var ids = ['streamStateChip', 'streamStatus', 'streamStatusChip', 'mjpegStatus'];
    for (var i = 0; i < ids.length; i++) {
      var el = document.getElementById(ids[i]);
      if (el) {
        el.textContent = text;
        return;
      }
    }
  }

  function forceImgStream() {
    var img = document.getElementById('mjpegImg');
    var frame = document.getElementById('mjpegFrame');
    var mode = document.getElementById('streamMode');
    var urlInput = document.getElementById('mjpegUrl');

    if (mode) mode.value = 'img';
    if (urlInput && !urlInput.value) urlInput.value = streamUrl();

    if (frame) {
      frame.removeAttribute('src');
      frame.classList.add('hidden');
      frame.style.display = 'none';
      frame.style.visibility = 'hidden';
      frame.style.opacity = '0';
      frame.style.pointerEvents = 'none';
    }

    if (!img) return;

    img.classList.remove('hidden');
    img.style.display = 'block';
    img.style.visibility = 'visible';
    img.style.opacity = '1';
    img.style.width = '100%';
    img.style.height = '100%';
    img.style.objectFit = 'contain';
    img.style.background = '#111';

    var current = String(img.getAttribute('src') || img.src || '');
    if (!current || current.indexOf('/stream') < 0) {
      img.src = streamUrl();
      setStatus('STREAM IMG');
      try { console.log('[video-bootstrap] img src set', img.src); } catch (_) {}
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
    document.addEventListener('DOMContentLoaded', forceImgStream);
  } else {
    forceImgStream();
  }

  setTimeout(forceImgStream, 300);
  setTimeout(forceImgStream, 1000);
  setTimeout(forceImgStream, 2500);

  setInterval(function () {
    var img = document.getElementById('mjpegImg');
    if (!img) return;

    var current = String(img.getAttribute('src') || img.src || '');
    var hidden = img.classList.contains('hidden') || img.style.display === 'none';

    if (!current || current.indexOf('/stream') < 0 || hidden) {
      forceImgStream();
    }
  }, 5000);
})();
</script>
<!-- FORCE_VIDEO_BOOTSTRAP_END -->
'''

if '</body>' not in s:
    raise SystemExit("ERROR: </body> not found")

s = s.replace('</body>', bootstrap + '\n</body>', 1)

p.write_text(s, encoding="utf-8")
print("OK: forced IMG stream mode bootstrap inserted")
