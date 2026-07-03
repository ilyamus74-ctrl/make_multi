#!/usr/bin/env python3
import json
import urllib.request
import time

PTZ = "http://127.0.0.1:8090"

def get(url):
    with urllib.request.urlopen(url, timeout=3) as r:
        return json.loads(r.read().decode("utf-8", errors="replace"))

def post(url, body):
    data = json.dumps(body).encode("utf-8")
    req = urllib.request.Request(
        url,
        data=data,
        headers={"Content-Type": "application/json"},
        method="POST"
    )
    with urllib.request.urlopen(req, timeout=3) as r:
        raw = r.read().decode("utf-8", errors="replace")
    return json.loads(raw) if raw.strip() else {}

def show(label):
    st = get(f"{PTZ}/api/autopilot/state")
    print("")
    print(label)
    print("speed_profile_source =", st.get("speed_profile_source"))
    print("active_zoom_sample_idx =", st.get("active_zoom_sample_idx"))
    print("kp/kd/max =", st.get("kp"), st.get("kd"), st.get("max_pan"), st.get("max_tilt"), st.get("max_accel"))
    print("target/autozoom =", st.get("target_x"), st.get("target_y"), st.get("auto_zoom_target_h"), st.get("auto_zoom_deadzone"))

show("BEFORE")

print("")
print("POST speed_profile/apply_nearest")
print(post(f"{PTZ}/api/autopilot/speed_profile/apply_nearest", {
    "profile_idx": 0
}))

time.sleep(1)
show("AFTER APPLY_NEAREST")

print("")
print("POST framing-only /api/autopilot/config")
print(post(f"{PTZ}/api/autopilot/config", {
    "target_x": 0.5,
    "target_y": 0.5,
    "auto_zoom_enable": True,
    "auto_zoom_target_h": 0.42,
    "auto_zoom_deadzone": 0.12,
    "auto_zoom_cmd": 8,
    "auto_zoom_sign": 1,
    "auto_zoom_period_ms": 500
}))

time.sleep(1)
show("AFTER FRAMING CONFIG")
