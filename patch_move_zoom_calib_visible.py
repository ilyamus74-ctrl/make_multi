from pathlib import Path
import shutil

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(".html.bak_zoom_calib_visible")
if not backup.exists():
    shutil.copy2(p, backup)
    print(f"backup: {backup}")

block = '''        <div class="row">
          <b>ZOOM CALIB:</b>
          <button id="zoomRunAprilTagCalibBtn" class="small-btn">RUN APRILTAG ZOOM CALIB</button>
          <label>Tag mm <input id="zoomTagSizeMm" type="number" value="160"></label>
          <label>Near m <input id="zoomNearM" type="number" value="1" step="0.1"></label>
          <label>Far m <input id="zoomFarM" type="number" value="10" step="0.5"></label>
          <label>Impulse ms <input id="zoomImpulseMs" type="number" value="100"></label>
          <label>Samples <input id="zoomSamples" type="number" value="20"></label>
          <code id="zoomAprilTagCalibStatus">idle</code>
        </div>
'''

if block not in s:
    raise SystemExit("zoom apriltag calibration block not found")

# Remove from hidden Advanced block.
s = s.replace(block, "", 1)

# Insert after PTZ config panel, before PTZ log.
anchor = '''      <div class="row">
        <details><summary>PTZ log</summary><textarea id="ptzDebugLog" style="width:100%;height:48px;font-family:monospace;font-size:12px"></textarea></details>
      </div>
'''

visible_block = '''      <div class="row" id="zoomAprilTagCalibPanel">
        <b>ZOOM CALIB:</b>
        <button id="zoomRunAprilTagCalibBtn" class="small-btn">RUN APRILTAG ZOOM CALIB</button>
        <label>Tag mm <input id="zoomTagSizeMm" type="number" value="160"></label>
        <label>Near m <input id="zoomNearM" type="number" value="1" step="0.1"></label>
        <label>Far m <input id="zoomFarM" type="number" value="10" step="0.5"></label>
        <label>Impulse ms <input id="zoomImpulseMs" type="number" value="100"></label>
        <label>Samples <input id="zoomSamples" type="number" value="20"></label>
        <code id="zoomAprilTagCalibStatus">idle</code>
      </div>
'''

if anchor not in s:
    raise SystemExit("PTZ log anchor not found")

s = s.replace(anchor, visible_block + anchor, 1)

p.write_text(s, encoding="utf-8")
print("patched web/index.html")
