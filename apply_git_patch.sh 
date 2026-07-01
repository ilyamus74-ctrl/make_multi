#!/usr/bin/env bash
set -euo pipefail

PATCH_FILE="${1:-/tmp/new_patch.diff}"

echo "[INFO] Repo: $(pwd)"
echo "[INFO] Patch: $PATCH_FILE"

if [ ! -f "$PATCH_FILE" ]; then
  echo "[ERROR] Patch file not found: $PATCH_FILE"
  echo "Usage: ./apply_git_patch.sh /path/to/patch.diff"
  exit 2
fi

if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  echo "[ERROR] Current directory is not a git repo"
  exit 2
fi

echo "[INFO] Current branch:"
git branch --show-current || true

echo "[INFO] Current git status:"
git status --short

echo "[INFO] Normalizing CRLF if needed..."
TMP_PATCH="$(mktemp /tmp/git_patch_XXXXXX.diff)"
sed 's/\r$//' "$PATCH_FILE" > "$TMP_PATCH"

echo "[INFO] Checking patch..."
if ! git apply --check "$TMP_PATCH"; then
  echo "[ERROR] Patch check failed. Nothing applied."
  echo "[INFO] Try inspect:"
  echo "  git apply --check $TMP_PATCH"
  exit 1
fi

echo "[INFO] Applying patch..."
git apply "$TMP_PATCH"

echo "[INFO] Patch applied successfully."

echo "[INFO] Changed files:"
git status --short

echo "[INFO] PHP syntax check for changed PHP files..."
CHANGED_PHP_FILES="$(git diff --name-only -- '*.php' || true)"

if [ -n "$CHANGED_PHP_FILES" ]; then
  while IFS= read -r file; do
    if [ -f "$file" ]; then
      echo "[PHP-LINT] $file"
      php -l "$file"
    fi
  done <<< "$CHANGED_PHP_FILES"
else
  echo "[INFO] No changed PHP files."
fi

echo "[INFO] Done."

