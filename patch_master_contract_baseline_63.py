#!/usr/bin/env python3
from pathlib import Path
import time

p = Path("PTZ_MASTER_CONTRACT.md")
s = p.read_text(encoding="utf-8", errors="ignore")

bak = p.with_suffix(p.suffix + f".bak_baseline_63_{int(time.time())}")
bak.write_text(s, encoding="utf-8")

replacements = {
    "PASS = 62\nFAIL = 0": "PASS = 63\nFAIL = 0",
    "PASS=62 FAIL=0": "PASS=63 FAIL=0",
    "PASS = 62": "PASS = 63",
}

changed = 0
for old, new in replacements.items():
    count = s.count(old)
    if count:
        s = s.replace(old, new)
        changed += count

# Add a concise note if missing.
note = """
## Latest confirmed baseline

```text
ptz_contract_audit.py
PASS = 63
FAIL = 0
```

Additional audit check included:

```text
Lifecycle persistent custom presets available
```
"""

if "Latest confirmed baseline" not in s:
    s = s.rstrip() + "\n\n" + note.strip() + "\n"
    changed += 1

p.write_text(s, encoding="utf-8")

print("OK updated PTZ_MASTER_CONTRACT.md baseline to PASS=63 FAIL=0")
print("Changed items:", changed)
print("Backup:", bak)