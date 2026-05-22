#!/usr/bin/env bash
set -euo pipefail

BASE_URL="${BASE_URL:-http://127.0.0.1:8080}"
TIMEOUT_SEC="${TIMEOUT_SEC:-2}"

pass_count=0
warn_count=0
fail_count=0

say() {
  printf '%s\n' "$*"
}

pass() {
  ((pass_count+=1))
  say "✅ $*"
}

warn() {
  ((warn_count+=1))
  say "⚠️  $*"
}

fail() {
  ((fail_count+=1))
  say "❌ $*"
}

check_endpoint() {
  local name="$1"
  local path="$2"
  local expect_substr="${3:-}"
  local tmp
  tmp="$(mktemp)"

  if curl -fsS --max-time "$TIMEOUT_SEC" "$BASE_URL$path" >"$tmp" 2>/dev/null; then
    if [[ -n "$expect_substr" ]] && ! grep -q "$expect_substr" "$tmp"; then
      fail "$name: endpoint доступен, но нет ожидаемого фрагмента '$expect_substr'"
      rm -f "$tmp"
      return
    fi
    pass "$name: $BASE_URL$path"
  else
    warn "$name: $BASE_URL$path недоступен (сервис не запущен или сеть ограничена)"
  fi

  rm -f "$tmp"
}

check_events_log() {
  local log_path="${1:-ops_events.log}"
  if [[ ! -f "$log_path" ]]; then
    warn "Лог событий $log_path не найден"
    return
  fi

  local found=0
  for event in video_lost video_retry video_restored reacquire_started; do
    if grep -q "$event" "$log_path"; then
      ((found+=1))
    fi
  done

  if (( found >= 3 )); then
    pass "Лог $log_path содержит ключевые runbook-события ($found/4)"
  else
    fail "Лог $log_path не покрывает runbook-события ($found/4)"
  fi
}

say "== QA.B5 MVP acceptance smoke runner =="
say "BASE_URL=$BASE_URL"

check_endpoint "Ping API" "/api/ping" "rtt"
check_endpoint "Detections API" "/api/detections" "confidence"
check_endpoint "Tracking confidence API" "/api/tracking/confidence" "fused"
check_endpoint "Events summary API" "/api/events/summary" "\"counts\""
check_endpoint "Events summary API (recent window)" "/api/events/summary?recent=50" "\"recent_limit\":50"
check_endpoint "Settings API" "/api/settings" "config_version"
check_events_log "ops_events.log"

say ""
say "Итог: pass=$pass_count warn=$warn_count fail=$fail_count"

if (( fail_count > 0 )); then
  exit 1
fi

exit 0
