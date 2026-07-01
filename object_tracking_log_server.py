#!/usr/bin/env python3
import argparse
import json
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse, unquote

from object_tracking_log_paths import (
    RUNTIME_DIR,
    ARCHIVE_DIR,
    CURRENT_DIR,
    ensure_log_dirs,
)

ROOT = Path(__file__).resolve().parent


def read_text_tail(path, lines=200):
    path = Path(path)

    if not path.exists():
        return []

    q = deque(maxlen=max(1, int(lines)))

    with path.open("r", encoding="utf-8", errors="replace") as f:
        for line in f:
            q.append(line.rstrip("\n"))

    return list(q)


def read_json(path, default=None):
    try:
        path = Path(path)
        if path.exists():
            return json.loads(path.read_text(encoding="utf-8", errors="replace"))
    except Exception:
        pass
    return default


def read_jsonl_tail(path, lines=200):
    out = []

    for line in read_text_tail(path, lines=lines):
        try:
            out.append(json.loads(line))
        except Exception:
            out.append({"raw": line})

    return out


def list_archive_files():
    ensure_log_dirs()

    out = []

    for p in sorted(ARCHIVE_DIR.rglob("*"), key=lambda x: x.stat().st_mtime if x.exists() else 0, reverse=True):
        if not p.is_file():
            continue

        try:
            rel = p.relative_to(ARCHIVE_DIR)
        except Exception:
            continue

        out.append({
            "name": str(rel),
            "size": p.stat().st_size,
            "mtime": p.stat().st_mtime
        })

    return out[:300]


def safe_archive_path(name):
    name = unquote(str(name or "")).strip().lstrip("/")
    p = (ARCHIVE_DIR / name).resolve()

    if not str(p).startswith(str(ARCHIVE_DIR.resolve())):
        return None

    if not p.exists() or not p.is_file():
        return None

    return p


HTML = r'''<!doctype html>
<html>
<head>
<meta charset="utf-8">
<title>Object Tracking Logs</title>
<style>
body { background:#0b0b0b; color:#ddd; font-family:Arial, sans-serif; margin:14px; }
button, select, input { background:#222; color:#eee; border:1px solid #555; border-radius:6px; padding:5px 9px; }
pre { background:#111; border:1px solid #333; border-radius:8px; padding:10px; overflow:auto; max-height:420px; }
.card { border:1px solid #333; border-radius:8px; padding:10px; margin:10px 0; background:#111; }
.good { color:#00ff66; }
.warn { color:#ffcc00; }
.bad { color:#ff5555; }
.grid { display:grid; grid-template-columns: 1fr 1fr; gap:10px; }
.event { border-bottom:1px solid #222; padding:4px 0; font-family:monospace; white-space:pre-wrap; }
.small { font-size:12px; opacity:.8; }
a { color:#72a7ff; }
</style>
</head>
<body>
<h2>Object Tracking Logs</h2>

<div class="card">
  <button onclick="refresh()">Refresh</button>
  <label><input id="auto" type="checkbox" checked> auto refresh</label>
  <label>tail <input id="tail" type="number" value="120" min="20" max="1000"></label>
  <span id="status" class="small"></span>
</div>

<div class="grid">
  <div class="card">
    <h3>Current tracking state</h3>
    <pre id="trackingState">loading...</pre>
  </div>
  <div class="card">
    <h3>Daemon state</h3>
    <pre id="daemonState">loading...</pre>
  </div>
</div>

<div class="card">
  <h3>Events</h3>
  <div id="events"></div>
</div>

<div class="card">
  <h3>Daemon log</h3>
  <pre id="daemonLog"></pre>
</div>

<div class="card">
  <h3>Archive</h3>
  <div id="archive"></div>
</div>

<script>
function clsForEvent(ev) {
  if (!ev) return '';
  if (ev.includes('tracking_started') || ev === 'tracking') return 'good';
  if (ev.includes('lost') || ev.includes('skip') || ev.includes('scan')) return 'warn';
  if (ev.includes('error') || ev.includes('failed')) return 'bad';
  return '';
}

function compactEvent(e) {
  const keys = ['ts','event','preset','track_id','cls','score','center','mode','ptz','cmd_pan','cmd_tilt','reason','cmd','hold_ms','skipped'];
  const o = {};
  for (const k of keys) if (e[k] !== undefined) o[k] = e[k];
  return JSON.stringify(o, null, 0);
}

async function refresh() {
  const tail = Number(document.getElementById('tail').value || 120);
  const r = await fetch('/api/logs?tail=' + encodeURIComponent(tail), {cache:'no-store'});
  const j = await r.json();

  document.getElementById('status').textContent = new Date().toLocaleTimeString();

  document.getElementById('trackingState').textContent = JSON.stringify(j.tracking_state, null, 2);
  document.getElementById('daemonState').textContent = JSON.stringify(j.daemon_state, null, 2);
  document.getElementById('daemonLog').textContent = (j.daemon_log || []).join('\n');

  const ev = document.getElementById('events');
  ev.innerHTML = '';
  for (const e of (j.events || [])) {
    const div = document.createElement('div');
    div.className = 'event ' + clsForEvent(e.event || '');
    div.textContent = compactEvent(e);
    ev.appendChild(div);
  }

  const ar = document.getElementById('archive');
  ar.innerHTML = '';
  for (const f of (j.archive || []).slice(0, 80)) {
    const a = document.createElement('a');
    a.href = '/archive?file=' + encodeURIComponent(f.name);
    a.target = '_blank';
    a.textContent = f.name + ' (' + f.size + ' bytes)';
    const div = document.createElement('div');
    div.appendChild(a);
    ar.appendChild(div);
  }
}

setInterval(() => {
  if (document.getElementById('auto').checked) refresh().catch(console.error);
}, 2000);

refresh().catch(console.error);
</script>
</body>
</html>
'''


class Handler(BaseHTTPRequestHandler):
    def send_json(self, obj, status=200):
        raw = json.dumps(obj, ensure_ascii=False, indent=2).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Content-Length", str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)

    def send_text(self, text, status=200, content_type="text/plain; charset=utf-8"):
        raw = str(text).encode("utf-8", errors="replace")
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Content-Length", str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)

    def do_GET(self):
        ensure_log_dirs()

        u = urlparse(self.path)
        qs = parse_qs(u.query)

        if u.path == "/":
            return self.send_text(HTML, content_type="text/html; charset=utf-8")

        if u.path == "/api/logs":
            tail = int((qs.get("tail") or ["200"])[0])

            return self.send_json({
                "ok": True,
                "runtime_dir": str(RUNTIME_DIR),
                "archive_dir": str(ARCHIVE_DIR),
                "tracking_state": read_json(RUNTIME_DIR / "object_tracking_state.json", {}),
                "daemon_state": read_json(RUNTIME_DIR / "object_tracking_daemon_state.json", {}),
                "events": read_jsonl_tail(RUNTIME_DIR / "object_tracking_events.jsonl", tail),
                "daemon_log": read_text_tail(RUNTIME_DIR / "object_tracking_daemon.log", min(tail, 300)),
                "archive": list_archive_files()
            })

        if u.path == "/api/archive":
            return self.send_json({
                "ok": True,
                "archive_dir": str(ARCHIVE_DIR),
                "files": list_archive_files()
            })

        if u.path == "/archive":
            name = (qs.get("file") or [""])[0]
            p = safe_archive_path(name)

            if p is None:
                return self.send_text("not found", status=404)

            return self.send_text(p.read_text(encoding="utf-8", errors="replace"))

        return self.send_text("not found", status=404)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8091)
    args = ap.parse_args()

    ensure_log_dirs()

    srv = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"Object log server: http://{args.host}:{args.port}", flush=True)
    srv.serve_forever()


if __name__ == "__main__":
    main()
