from pathlib import Path
import shutil, time

p = Path("web/index.html")
s = p.read_text(encoding="utf-8")

backup = p.with_suffix(f".html.bak_fix_zoom_anchor_defs_order_{int(time.time())}")
shutil.copy2(p, backup)
print("Backup:", backup)

early_block = r'''  // ZOOM_ANCHOR_DEFS_EARLY_START
  let zoomActiveAnchorLabel = 'near';

  const zoomAnchorDefs = [
    { label: 'near', prefix: 'Near', defaultDistanceM: 1, defaultTagId: 7 },
    { label: 'mid', prefix: 'Mid', defaultDistanceM: 5, defaultTagId: -1 },
    { label: 'far', prefix: 'Far', defaultDistanceM: 10, defaultTagId: 12 },
  ];

  function zoomAnchorInputId(def, field) {
    return `zoomAnchor${def.prefix}${field}`;
  }

  function readZoomAnchorFromUi(def) {
    return {
      label: def.label,
      enabled: !!$(zoomAnchorInputId(def, 'Enabled'))?.checked,
      tag_id: Number($(zoomAnchorInputId(def, 'TagId'))?.value ?? def.defaultTagId),
      distance_mm: Number($(zoomAnchorInputId(def, 'DistanceM'))?.value || def.defaultDistanceM) * 1000,
      tag_size_mm: Number($(zoomAnchorInputId(def, 'SizeMm'))?.value || 160),
    };
  }

  function getActiveZoomAnchorFromUi() {
    const anchors = zoomAnchorDefs.map(readZoomAnchorFromUi);
    return anchors.find(a => a.label === zoomActiveAnchorLabel && a.enabled) ||
      anchors.find(a => a.enabled) ||
      anchors.find(a => a.label === zoomActiveAnchorLabel) ||
      anchors[0];
  }
  // ZOOM_ANCHOR_DEFS_EARLY_END

'''

insert_anchor = '''  // PTZ speed tune sample init must never block video stream startup.
'''

if 'ZOOM_ANCHOR_DEFS_EARLY_START' not in s:
    if insert_anchor not in s:
        raise SystemExit("ERROR: insert anchor not found")
    s = s.replace(insert_anchor, early_block + insert_anchor, 1)
    print("OK: inserted early zoomAnchorDefs block")
else:
    print("SKIP: early zoomAnchorDefs already inserted")

old_block_start = "  let zoomActiveAnchorLabel = 'near';\n"
old_block_end = "  function updateZoomActiveAnchorSummary() {\n"

start = s.find(old_block_start)
end = s.find(old_block_end, start)

if start < 0 or end < 0:
    print("SKIP: old late zoomAnchorDefs block not found")
else:
    block = s[start:end]
    if 'const zoomAnchorDefs' in block and 'function getActiveZoomAnchorFromUi' in block:
        s = s[:start] + "  // zoomAnchorDefs moved earlier to avoid TDZ initialization error.\n\n" + s[end:]
        print("OK: removed late zoomAnchorDefs duplicate block")
    else:
        print("SKIP: matched block does not look like zoomAnchorDefs block")

p.write_text(s, encoding="utf-8")
print("DONE")
