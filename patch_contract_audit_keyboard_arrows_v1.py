#!/usr/bin/env python3
from pathlib import Path
import time

ROOT = Path('/root/new_yolo8')
AUDIT = ROOT / 'ptz_contract_audit.py'
MARKER = 'PTZ_AUDIT_KEYBOARD_ARROWS_V1'

block = r'''

    # PTZ_AUDIT_KEYBOARD_ARROWS_V1
    print("")
    section("LAYER 1D — KEYBOARD ARROW PTZ JS CONTRACT")

    try:
        _arrow_web_text = web_text
    except Exception:
        _arrow_web_text = (ROOT / "web/index.html").read_text(encoding="utf-8", errors="ignore")

    arrow_start = _arrow_web_text.find("function ptzKeyboardCaptureDirection")
    arrow_end = _arrow_web_text.find("QA_SWEEP_GLOBAL_HELPERS_START", arrow_start)
    if arrow_start >= 0 and arrow_end > arrow_start:
        arrow_block = _arrow_web_text[arrow_start:arrow_end]
    elif arrow_start >= 0:
        arrow_block = _arrow_web_text[arrow_start:arrow_start + 9000]
    else:
        arrow_block = ""

    arrow_tokens = [
        "ptzKeyboardCaptureDirection",
        "ptzKeyboardCaptureEnabled",
        "window.__ptzKeyCaptureInstalled",
        "ArrowLeft",
        "ArrowRight",
        "ArrowUp",
        "ArrowDown",
        "keyboard_left",
        "keyboard_right",
        "keyboard_up",
        "keyboard_down",
    ]

    for token in arrow_tokens:
        cnt = _arrow_web_text.count(token)
        print(token, "=", cnt)
        add(f"Keyboard arrow marker {token}", cnt >= 1, f"count={cnt}")

    has_arrow_block = bool(arrow_block)
    has_all_directions = all(t in arrow_block for t in ["ArrowLeft", "ArrowRight", "ArrowUp", "ArrowDown"])
    has_ptz_mode_guard = "ptzKeyboardCaptureEnabled" in arrow_block and "controlMode" in arrow_block and "ptz" in arrow_block
    has_capture_keydown = "window.addEventListener('keydown'" in arrow_block and "e.preventDefault()" in arrow_block and "e.stopImmediatePropagation()" in arrow_block and "}, true)" in arrow_block
    has_capture_keyup = "window.addEventListener('keyup'" in arrow_block and "manualKeyState.delete" in arrow_block and "}, true)" in arrow_block
    has_motion_loop = "keyboardMotionLoop" in arrow_block or "manualInput" in _arrow_web_text or "manual_j_pulse" in _arrow_web_text
    has_key_release_stop = "scheduleKeyboardHoldStop" in arrow_block or "stopManualHold" in arrow_block or "manualInputStop" in _arrow_web_text

    print("Arrow scoped block length =", len(arrow_block))
    print("Arrow owns all directions =", has_all_directions)
    print("Arrow PTZ mode guard =", has_ptz_mode_guard)
    print("Arrow capture keydown =", has_capture_keydown)
    print("Arrow capture keyup =", has_capture_keyup)
    print("Arrow backend/manual motion path =", has_motion_loop)
    print("Arrow key release stop path =", has_key_release_stop)

    add(
        "Keyboard arrows own PTZ pan/tilt directions",
        has_arrow_block and has_all_directions,
        f"block={has_arrow_block} all_directions={has_all_directions}"
    )
    add(
        "Keyboard arrows are gated to PTZ mode",
        has_ptz_mode_guard,
        f"ptz_mode_guard={has_ptz_mode_guard}"
    )
    add(
        "Keyboard arrows capture keydown before browser/page handlers",
        has_capture_keydown,
        f"capture_keydown={has_capture_keydown}"
    )
    add(
        "Keyboard arrows capture keyup and release movement",
        has_capture_keyup and has_key_release_stop,
        f"capture_keyup={has_capture_keyup} release_stop={has_key_release_stop}"
    )
    add(
        "Keyboard arrows have backend/manual motion path",
        has_motion_loop,
        f"motion_path={has_motion_loop}"
    )
'''

if not AUDIT.exists():
    raise SystemExit(f'Missing {AUDIT}')

text = AUDIT.read_text(encoding='utf-8', errors='ignore')
if MARKER in text:
    print('Already patched:', MARKER)
    raise SystemExit(0)

backup = AUDIT.with_name(AUDIT.name + f'.bak_keyboard_arrows_v1_{int(time.time())}')
backup.write_text(text, encoding='utf-8')

layer_pos = text.find('LAYER 1 — BROWSER UI')
if layer_pos < 0:
    raise SystemExit('Could not find LAYER 1 — BROWSER UI in ptz_contract_audit.py')

next_def = text.find('\ndef ', layer_pos)
if next_def < 0:
    raise SystemExit('Could not find insertion boundary after LAYER 1 function')

new_text = text[:next_def] + block + text[next_def:]
AUDIT.write_text(new_text, encoding='utf-8')

print('OK patched ptz_contract_audit.py keyboard arrows audit v1')
print('Backup:', backup)
print('Changes:')
print(' - added LAYER 1D — KEYBOARD ARROW PTZ JS CONTRACT')
print(' - checks Arrow keys capture before browser/page handlers')
print(' - checks PTZ-mode guard, keyup release, and backend/manual motion path')