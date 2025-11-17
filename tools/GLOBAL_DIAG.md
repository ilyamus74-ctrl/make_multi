# Global Diagnostics Viewer

`global_diag` is a standalone utility that polls each camera's `/tracks` endpoint, logs local-to-global mapping changes, and serves a lightweight web UI plus a `/state` JSON feed for monitoring.

## Running

```
./global_diag \
  --config ./config.json \
  --host 127.0.0.1 \
  --base-port 5000 \
  --pull-interval-ms 250 \
  --http-port 7090 \
  --max-tracks-per-cam 50
```

* Cameras are contacted at `http://<host>:<base_port + index>` unless an explicit `http_port` is present in `config.json`.
* Polling is resilient: unresponsive cameras are marked **DOWN** but do not stop the server.
* Logs are written to `logs/global_diag.log` only when track/global assignments change, when tracks appear/disappear, and via periodic heartbeats.

## Web interface

* `GET /` renders a simple HTML table that auto-refreshes every second.
* `GET /state` returns a JSON summary such as:
  ```json
  {
    "ts_ms": 1731580800456,
    "cams": [
      {"id":"cam01","status":"UP","tracks":23,"with_global":9,"without_global":14,"last_pull_ms":1731580800430}
    ]
  }
  ```

## Camera endpoint requirement

Each camera must expose `GET /tracks` returning the current tracks:
```json
{
  "camera_id": "cam01",
  "ts_ms": 1731580800123,
  "tracks": [
    {"local_id": 12, "global_id": 1, "class": "person", "conf": 0.86,
     "age_ms": 950, "bbox": [x,y,w,h] }
  ]
}
```
A missing `global_id` is treated as `-1`.
