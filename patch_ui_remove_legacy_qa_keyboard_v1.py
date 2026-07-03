#!/usr/bin/env python3
from pathlib import Path
import re
import time

ROOT = Path('/root/new_yolo8')
WEB = ROOT / 'web' / 'index.html'

START_CANON = 'PTZ_QA_SWEEP_PROFILE_IDX_BACKEND_START'
END_CANON = 'PTZ_QA_SWEEP_PROFILE_IDX_BACKEND_END'

LEGACY_FORBIDDEN = [
    'QA_SWEEP_GLOBAL_HELPERS_START',
    'QA_SWEEP_GLOBAL_HELPERS_END',
    'keyboardQaSweepStart',
    'keyboardQaSweepStop',
    'keyboardQaIsSweepMode',
    'KEYBOARD_QA_SWEEP_HOLD_ZOOM_START',
    'KEYBOARD_QA_SWEEP_HOLD_ZOOM_END',
    'keyboard-qa-sweep',
    'QA_SWEEP_BACKEND_JOG_HARD_OVERRIDE_START',
    'QA_SWEEP_BACKEND_JOG_HARD_OVERRIDE_END',
    'qa-hard-jog',
    'QA HARD JOG',
    'KEYBOARD QA SWEEP JOG START',
    'KEYBOARD QA SWEEP JOG STOP',
    'keyboard_qa_sweep_hold',
]

KEEP_REQUIRED = [
    'PTZ_QA_SWEEP_PROFILE_IDX_BACKEND_START',
    'PTZ_QA_SWEEP_PROFILE_IDX_BACKEND_END',
    'PTZ_QA_SAMPLE_STEP_MOVEMENT_MODE_V4',
    'ptzQaSweepProfileIdxStep',
    'keyboard_Q_profile_idx_sweep',
    'keyboard_A_profile_idx_sweep',
    'ptzKeyboardCaptureDirection',
    'ptzKeyboardCaptureEnabled',
    'window.__ptzKeyCaptureInstalled',
]


def backup(path: Path) -> Path:
    stamp = int(time.time())
    dst = path.with_name(path.name + f'.bak_remove_legacy_qa_keyboard_v1_{stamp}')
    dst.write_text(path.read_text(encoding='utf-8', errors='ignore'), encoding='utf-8')
    return dst


def remove_between_markers(text: str, start: str, end: str, label: str) -> tuple[str, bool]:
    a = text.find(start)
    b = text.find(end)
    if a < 0 and b < 0:
        return text, False
    if a < 0 or b < 0 or b < a:
        raise SystemExit(f'Cannot remove {label}: broken markers start={a} end={b}')

    # Expand to whole lines.
    line_start = text.rfind('\n', 0, a)
    if line_start < 0:
        line_start = 0
    else:
        line_start += 1

    line_end = text.find('\n', b)
    if line_end < 0:
        line_end = len(text)
    else:
        line_end += 1

    return text[:line_start] + text[line_end:], True


def remove_script_block(text: str, start_marker: str, end_marker: str, label: str) -> tuple[str, bool]:
    # Prefer deleting the whole standalone <script> that contains the legacy block.
    pattern = re.compile(
        r'\n?\s*<script>\s*\n\s*//\s*' + re.escape(start_marker) +
        r'.*?//\s*' + re.escape(end_marker) + r'\s*\n\s*</script>\s*\n?',
        re.S,
    )
    new, n = pattern.subn('\n', text, count=1)
    if n:
        return new, True

    # Fallback: delete only marker region.
    return remove_between_markers(text, start_marker, end_marker, label)


def scoped_replace_after_canon(text: str, pattern: re.Pattern, repl: str, label: str) -> tuple[str, int]:
    end_pos = text.find(END_CANON)
    if end_pos < 0:
        raise SystemExit(f'Cannot find canonical end marker {END_CANON}')
    prefix = text[:end_pos]
    suffix = text[end_pos:]
    suffix2, n = pattern.subn(repl, suffix, count=1)
    return prefix + suffix2, n


def main():
    if not WEB.exists():
        raise SystemExit(f'Missing {WEB}')

    text = WEB.read_text(encoding='utf-8', errors='ignore')

    for token in KEEP_REQUIRED:
        if token not in text:
            raise SystemExit(f'Refusing cleanup: required canonical/arrow token missing before patch: {token}')

    bak = backup(WEB)
    changes = []

    # 1) Remove old global helper block that manually jogged zoom and then faked zoom state.
    text, ok = remove_between_markers(
        text,
        'QA_SWEEP_GLOBAL_HELPERS_START',
        'QA_SWEEP_GLOBAL_HELPERS_END',
        'QA_SWEEP_GLOBAL_HELPERS',
    )
    if ok:
        changes.append('removed QA_SWEEP_GLOBAL_HELPERS block')

    # 2) Remove legacy Q/A branch from the old combined keyboard handler, but keep arrows/Escape.
    old_keydown_qa = re.compile(
        r"\n\s*if \(isQ \|\| isA\) \{\s*\n"
        r"\s*e\.preventDefault\(\);\s*\n"
        r"\s*if \(e\.repeat\) return;\s*\n"
        r"\s*keyboardState\[isQ \? 'zin' : 'zout'\] = true;\s*\n"
        r"\s*if \(keyboardQaIsSweepMode\(\)\) \{\s*\n"
        r"\s*keyboardQaSweepStart\(isQ, isQ \? 'Q_TELE_SWEEP_HOLD' : 'A_WIDE_SWEEP_HOLD'\)\s*\n"
        r"\s*\.catch\(e => \{\s*\n"
        r"\s*apiPostJson\('/api/zoom/jog', \{ action: 'stop', cmd: 0 \}\)\.catch\(\(\) => sendZoomOnly\(0\)\);\s*\n"
        r"\s*keyboardQaSweepHold = null;\s*\n"
        r"\s*ptzLog\('KEYBOARD QA SWEEP START error', \{ error: String\(e\.message \|\| e\) \}\);\s*\n"
        r"\s*\}\);\s*\n"
        r"\s*\} else \{\s*\n"
        r"\s*if \(isQ\) keyboardZoomSampleStep\(1, 'Q_TELE'\);\s*\n"
        r"\s*if \(isA\) keyboardZoomSampleStep\(-1, 'A_WIDE'\);\s*\n"
        r"\s*\}\s*\n"
        r"\s*return;\s*\n"
        r"\s*\}\s*\n",
        re.S,
    )
    text, n = scoped_replace_after_canon(
        text,
        old_keydown_qa,
        "\n    // PTZ_LEGACY_QA_KEYDOWN_REMOVED_V1: canonical Q/A handler owns Q/A.\n    if (isQ || isA) return;\n",
        'legacy keydown Q/A branch',
    )
    if n:
        changes.append('removed legacy Q/A branch from old keydown handler')

    # 3) Remove old blur/visibility cleanup for removed keyboardQaSweepHold state.
    blur_pat = re.compile(
        r"\n\s*window\.addEventListener\('blur', \(\) => \{\s*\n"
        r"\s*if \(keyboardQaSweepHold\) keyboardQaSweepStop\('window_blur'\)\.catch\(\(\) => sendZoomOnly\(0\)\);\s*\n"
        r"\s*\}\);\s*\n\s*\n"
        r"\s*document\.addEventListener\('visibilitychange', \(\) => \{\s*\n"
        r"\s*if \(document\.hidden && keyboardQaSweepHold\) keyboardQaSweepStop\('visibility_hidden'\)\.catch\(\(\) => sendZoomOnly\(0\)\);\s*\n"
        r"\s*\}\);\s*\n",
        re.S,
    )
    text, n = scoped_replace_after_canon(
        text,
        blur_pat,
        "\n  // PTZ_LEGACY_QA_BLUR_VISIBILITY_REMOVED_V1: canonical Q/A is discrete sample-step and has no hold state.\n",
        'legacy blur/visibility Q/A cleanup',
    )
    if n:
        changes.append('removed legacy Q/A blur/visibility cleanup')

    # 4) Remove legacy Q/A branch from old keyup handler, but keep arrow key release logic.
    old_keyup_qa = re.compile(
        r"\n\s*if \(isQ \|\| isA\) \{\s*\n"
        r"\s*e\.preventDefault\(\);\s*\n"
        r"\s*keyboardState\[isQ \? 'zin' : 'zout'\] = false;\s*\n"
        r"\s*if \(keyboardQaIsSweepMode\(\)\) \{\s*\n"
        r"\s*keyboardQaSweepStop\('keyup'\)\.catch\(e => \{\s*\n"
        r"\s*sendZoomOnly\(0\);\s*\n"
        r"\s*keyboardQaSweepHold = null;\s*\n"
        r"\s*ptzLog\('KEYBOARD QA SWEEP STOP error', \{ error: String\(e\.message \|\| e\) \}\);\s*\n"
        r"\s*\}\);\s*\n"
        r"\s*\}\s*\n"
        r"\s*return;\s*\n"
        r"\s*\}\s*\n",
        re.S,
    )
    text, n = scoped_replace_after_canon(
        text,
        old_keyup_qa,
        "\n    // PTZ_LEGACY_QA_KEYUP_REMOVED_V1: canonical Q/A handler owns Q/A.\n    if (isQ || isA) return;\n",
        'legacy keyup Q/A branch',
    )
    if n:
        changes.append('removed legacy Q/A branch from old keyup handler')

    # 5) Remove standalone legacy script blocks added by older experiments.
    for start, end, label in [
        ('KEYBOARD_QA_SWEEP_HOLD_ZOOM_START', 'KEYBOARD_QA_SWEEP_HOLD_ZOOM_END', 'KEYBOARD_QA_SWEEP_HOLD_ZOOM'),
        ('QA_SWEEP_BACKEND_JOG_HARD_OVERRIDE_START', 'QA_SWEEP_BACKEND_JOG_HARD_OVERRIDE_END', 'QA_SWEEP_BACKEND_JOG_HARD_OVERRIDE'),
    ]:
        text, ok = remove_script_block(text, start, end, label)
        if ok:
            changes.append(f'removed {label} script/block')

    # Safety checks.
    missing_keep = [t for t in KEEP_REQUIRED if t not in text]
    if missing_keep:
        raise SystemExit('Cleanup would remove required canonical/arrow tokens: ' + ', '.join(missing_keep))

    remaining = {t: text.count(t) for t in LEGACY_FORBIDDEN if text.count(t) > 0}
    if remaining:
        raise SystemExit(
            'Cleanup incomplete; forbidden legacy tokens remain: ' +
            ', '.join(f'{k}={v}' for k, v in sorted(remaining.items())) +
            f'\nBackup left at {bak}'
        )

    WEB.write_text(text, encoding='utf-8')

    print('OK removed legacy Q/A keyboard code v1')
    print('Backup:', bak)
    print('Changes:')
    if changes:
        for c in changes:
            print(' -', c)
    else:
        print(' - no legacy blocks found; file already clean')
    print('Legacy forbidden tokens remaining: 0')


if __name__ == '__main__':
    main()