import json
import os
import signal
import subprocess
import time
import urllib.request

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

def ps_lines():
    out = subprocess.run(
        ["ps", "ax", "-o", "pid=,args="],
        text=True,
        capture_output=True
    ).stdout
    return [x.strip() for x in out.splitlines() if x.strip()]

def find_processes(pattern):
    result = []

    for line in ps_lines():
        parts = line.split(None, 1)

        if len(parts) != 2:
            continue

        pid = int(parts[0])
        args = parts[1]

        if pid == os.getpid():
            continue

        if pattern in args:
            result.append((pid, args))

    return result

def kill_watch_children():
    for pid, args in find_processes("apply_ptz_object_preset.py"):
        if "--watch" in args:
            print("kill watch", pid, args)
            try:
                os.kill(pid, signal.SIGTERM)
            except Exception as e:
                print("kill failed", pid, e)

def print_state(label):
    print("")
    print("=" * 80)
    print(label)
    print("=" * 80)

    try:
        settings = get("http://127.0.0.1:8080/api/settings")
    except Exception as e:
        settings = {"error": str(e)}

    try:
        ap = get("http://127.0.0.1:8090/api/autopilot/state")
    except Exception as e:
        ap = {"error": str(e)}

    daemon = find_processes("object_tracking_daemon.py")
    watches = [
        x
        for x in find_processes("apply_ptz_object_preset.py")
        if "--watch" in x[1]
    ]

    print("settings.activeObjectPreset =", settings.get("activeObjectPreset"))
    print("settings.activeSearchPreset =", settings.get("activeSearchPreset"))
    print("settings.ptzArmed =", settings.get("ptzArmed"))
    print("settings.controlMode =", settings.get("controlMode"))
    print("autopilot.enabled =", ap.get("enabled"))
    print("autopilot.mode =", ap.get("mode"))
    print("daemon_count =", len(daemon))
    for pid, args in daemon:
        print(" daemon", pid, args)

    print("watch_count =", len(watches))
    for pid, args in watches:
        print(" watch", pid, args)

print_state("initial")

print("")
print("STEP 1: force disarmed/manual")
settings = get("http://127.0.0.1:8080/api/settings")
settings["activeObjectPreset"] = "car_single"
settings["activeSearchPreset"] = settings.get("activeSearchPreset") or "lost_step_wait"
settings["ptzArmed"] = False
settings["controlMode"] = "manual"
post("http://127.0.0.1:8080/api/settings", settings)

try:
    post("http://127.0.0.1:8090/api/autopilot/stop", {})
except Exception as e:
    print("autopilot stop warning:", e)

try:
    post("http://127.0.0.1:8090/api/control/stop", {})
except Exception as e:
    print("control stop warning:", e)

kill_watch_children()

time.sleep(3)
print_state("after disarm wait")

print("")
print("STEP 2: arm via settings")
settings = get("http://127.0.0.1:8080/api/settings")
settings["activeObjectPreset"] = "car_single"
settings["activeSearchPreset"] = "lost_step_wait"
settings["ptzArmed"] = True
settings["controlMode"] = "ptz"
post("http://127.0.0.1:8080/api/settings", settings)

time.sleep(6)
print_state("after arm wait")

print("")
print("STEP 3: disarm again")
settings = get("http://127.0.0.1:8080/api/settings")
settings["ptzArmed"] = False
settings["controlMode"] = "manual"
post("http://127.0.0.1:8080/api/settings", settings)

try:
    post("http://127.0.0.1:8090/api/autopilot/stop", {})
except Exception as e:
    print("autopilot stop warning:", e)

time.sleep(4)
print_state("after final disarm wait")
