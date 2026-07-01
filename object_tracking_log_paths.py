from pathlib import Path
import os
import shutil
import time

ROOT = Path(__file__).resolve().parent

RUNTIME_DIR = Path(
    os.environ.get(
        "OBJECT_TRACKING_LOG_RUNTIME_DIR",
        "/dev/shm/new_yolo8_object_tracking"
    )
)

ARCHIVE_DIR = Path(
    os.environ.get(
        "OBJECT_TRACKING_LOG_ARCHIVE_DIR",
        str(ROOT / "logs")
    )
)

CURRENT_DIR = ARCHIVE_DIR / "current"

KEEP_SESSIONS = int(os.environ.get("OBJECT_TRACKING_LOG_KEEP_SESSIONS", "20"))


def ensure_log_dirs():
    RUNTIME_DIR.mkdir(parents=True, exist_ok=True)
    ARCHIVE_DIR.mkdir(parents=True, exist_ok=True)
    CURRENT_DIR.mkdir(parents=True, exist_ok=True)


def runtime_path(name):
    ensure_log_dirs()
    return RUNTIME_DIR / name


def archive_sessions():
    ensure_log_dirs()

    sessions = [
        p for p in ARCHIVE_DIR.iterdir()
        if p.is_dir() and p.name.startswith("session_")
    ]

    sessions.sort(key=lambda p: p.stat().st_mtime, reverse=True)

    for p in sessions[KEEP_SESSIONS:]:
        shutil.rmtree(p, ignore_errors=True)


def archive_current_logs(reason="stop"):
    ensure_log_dirs()

    files = [
        p for p in RUNTIME_DIR.iterdir()
        if p.is_file()
    ]

    if not files:
        return None

    stamp = time.strftime("%Y%m%d_%H%M%S")
    safe_reason = "".join(
        ch if ch.isalnum() or ch in "-_" else "_"
        for ch in str(reason or "stop")
    )

    session_dir = ARCHIVE_DIR / f"session_{stamp}_{safe_reason}"
    session_dir.mkdir(parents=True, exist_ok=True)

    for f in files:
        try:
            shutil.copy2(f, session_dir / f.name)
            shutil.copy2(f, CURRENT_DIR / f.name)
        except Exception:
            pass

    archive_sessions()
    return str(session_dir)
