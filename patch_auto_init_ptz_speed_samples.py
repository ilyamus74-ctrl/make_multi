from pathlib import Path
import shutil, time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".html.bak_auto_init_ptz_speed_samples_{int(time.time())}")
shutil.copy2(p, backup)
print("Backup:", backup)

block = r'''
<script>
// PTZ_SPEED_AUTO_INIT_REPAIR_START
(function () {
  if (window.__ptzSpeedAutoInitRepairInstalled) return;
  window.__ptzSpeedAutoInitRepairInstalled = true;

  const $ = (id) => document.getElementById(id);
  let busy = false;

  function hasSampleButtons() {
    const row = $('ptzTuneSamplesRow');
    return !!(row && row.querySelector('button'));
  }

  async function runPtzSpeedSampleInit(reason) {
    if (busy) return;
    busy = true;

    try {
      const fn = window.initPtzSpeedTuneSamples || globalThis.initPtzSpeedTuneSamples;

      if (typeof fn !== 'function') {
        console.warn('[ptz-speed-auto-init] initPtzSpeedTuneSamples not ready', reason);
        return;
      }

      await fn();

      console.log('[ptz-speed-auto-init] done', {
        reason,
        has_buttons: hasSampleButtons()
      });
    } catch (e) {
      console.error('[ptz-speed-auto-init] failed', reason, e);
    } finally {
      busy = false;
    }
  }

  function schedule(reason, delay) {
    setTimeout(() => runPtzSpeedSampleInit(reason), delay);
  }

  function install() {
    schedule('dom-ready', 300);
    schedule('late-1', 1000);
    schedule('late-2', 2500);

    const menu = $('ptzSpeedTuneMenu');
    if (menu && !menu.__ptzSpeedAutoInitBound) {
      menu.__ptzSpeedAutoInitBound = true;
      menu.addEventListener('toggle', () => {
        if (menu.open) schedule('menu-open', 50);
      });
    }

    const btn = $('ptzTuneBuildFromZoomMasterBtn');
    if (btn && !btn.__ptzSpeedAutoRefreshBound) {
      btn.__ptzSpeedAutoRefreshBound = true;
      btn.addEventListener('click', () => {
        schedule('after-build-click-1', 1200);
        schedule('after-build-click-2', 2500);
      }, true);
    }

    window.ptzSpeedSamplesAutoInit = () => runPtzSpeedSampleInit('manual');
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', install, { once: true });
  } else {
    install();
  }

  window.addEventListener('load', () => schedule('window-load', 300));
  window.addEventListener('pageshow', () => schedule('pageshow', 300));
})();
// PTZ_SPEED_AUTO_INIT_REPAIR_END
</script>
'''

if "PTZ_SPEED_AUTO_INIT_REPAIR_START" in s:
    print("SKIP: auto init repair already installed")
else:
    if "</body>" in s:
        s = s.replace("</body>", block + "\n</body>", 1)
    elif "</html>" in s:
        s = s.replace("</html>", block + "\n</html>", 1)
    else:
        s += "\n" + block + "\n"

    p.write_text(s, encoding="utf-8")
    print("OK: inserted PTZ speed samples auto init repair")
