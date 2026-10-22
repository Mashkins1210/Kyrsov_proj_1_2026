#!/usr/bin/env bash
# Shared practice session; unlimited attempts, all history preserved.
set -Eeuo pipefail
export LC_ALL=C.UTF-8 LANG=C.UTF-8 PYTHONUTF8=1
if [[ -t 0 ]]; then stty iutf8; fi
cd /home/robot/code
[[ -x venv/bin/python ]] || { echo "Не найден Python проекта" >&2; exit 1; }
exec 9>/tmp/vlrobot-rehearsal.lock
flock -n 9 || { echo "Другой заезд уже запущен" >&2; exit 1; }
venv/bin/python scripts/rehearsal.py --check-only >/dev/null
mkdir -p data/rehearsals
run_dir=$(mktemp -d data/rehearsals/run-XXXXXXXX)
session_file=data/rehearsal-session.json
while true; do
  read -r -p "Введите промпт: " mission || exit 130
  printf '%s' "$mission" > "$run_dir/mission.txt"
  if venv/bin/python scripts/validate_mission_text.py "$run_dir/mission.txt"; then break; fi
  mv "$run_dir/mission.txt" "$run_dir/invalid-mission-$(date +%s%N).bin"
  echo "Повторите ввод промпта."
done
printf 'ПРОБНЫЙ ЗАЕЗД. Сессия: %s
' "$session_file"
read -r -p "Робот на [0,0], смотрит к [1,0]. Начать? [да]: " confirm
[[ "$confirm" == "да" ]] || { echo "Запуск отменён"; exit 0; }
child=''
stop_child() {
  if [[ -n "$child" ]] && kill -0 "$child" 2>/dev/null; then
    kill -INT "$child" 2>/dev/null || true
    wait "$child" 2>/dev/null || true
  fi
}
trap stop_child EXIT
trap 'stop_child; exit 130' INT
trap 'stop_child; exit 143' TERM
venv/bin/python scripts/rehearsal.py \
  --mission-file "$run_dir/mission.txt" \
  --session "$session_file" \
  --drive --start-confirmed --unlimited-rehearsals \
  --operator-accepts-power-faults --operator-cleared-start-gap &
child=$!
status=0
wait "$child" || status=$?
child=''
printf 'Заезд завершён, код: %s. Материалы: %s
' "$status" "$run_dir"
exit "$status"
