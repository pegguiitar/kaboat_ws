#!/usr/bin/env bash
# batch.sh <도메인 시작> <이름:시나리오:seed> … — 여러 판을 동시에 (판당 CPU 약 1.7코어, 실시간이라 판당 최대 16분).
# 노트북이 절전에 들어가면 시계가 튀어 판이 무효가 된다: gnome-session-inhibit --inhibit idle:suspend bash batch.sh …
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
OUT="${KABOAT_E2E_OUT:-/tmp/kaboat_e2e_runs}"; mkdir -p "$OUT"
D=$1; shift
for spec in "$@"; do
  IFS=: read -r name scen seed <<< "$spec"
  bash "$HERE/run_one.sh" "$name" "$D" "$seed" "$HERE/scen/$scen.json" > "$OUT/$name.out" 2>&1 &
  D=$((D+1)); sleep 2
done
wait
python3 "$HERE/cleanup.py"
