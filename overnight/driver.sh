#!/usr/bin/env bash
# Overnight driver: keeps one headless Claude Code session working until overnight/DONE exists
# or the deadline passes. Supervisor guidance in overnight/INBOX.md is appended to the next
# resume prompt and archived. Run under systemd-inhibit so the machine does not sleep.
set -u
P=/home/alex/SUIND/TreeMapper; O=$P/overnight
ID=$(cat "$O/SESSION_ID")
# End time: RUN_UNTIL (any `date -d` string) or 9 h after launch.
DEADLINE=$(date -d "${RUN_UNTIL:-now + 9 hours}" +%s)
END_HHMM=$(date -d @"$DEADLINE" +%H:%M)
WRAP=$(( DEADLINE - 50*60 ))
COMMON=(--model claude-fable-5-1 --fallback-model claude-opus-5-5 --permission-mode auto
        --add-dir /media/alex/waspsim/cloud_build_bundle --output-format stream-json --verbose)
log() { echo "[driver $(date +%T)] $*" | tee -a "$O/driver.log"; }
cd "$P"
echo $$ > "$O/driver.pid"
log "start pid=$$ session=$ID deadline=$(date -d @$DEADLINE +%T)"

run_turn() {  # $1 = prompt, $2.. = session flags
  local prompt="$1"; shift
  claude -p "$prompt" "$@" "${COMMON[@]}" >> "$O/worker.jsonl" 2>> "$O/worker.err"
}

turn=0
while [ ! -f "$O/DONE" ] && [ "$(date +%s)" -lt "$DEADLINE" ]; do
  turn=$((turn+1))
  if [ ! -f "$O/started" ]; then
    log "turn $turn: kickoff"
    run_turn "$(cat "$O/KICKOFF.md")" --session-id "$ID"; rc=$?
    [ $rc -eq 0 ] && touch "$O/started"
  else
    now=$(date +%s)
    if [ "$now" -ge "$WRAP" ]; then
      base="WRAP-UP WINDOW: it is $(date +%H:%M); the run ends at $END_HHMM. Stop starting new work. Finish and commit the handoff deliverables from brief section 12 (README, results report with scorecards incl. the existing-system baseline, design doc, decision log), make sure the tree builds and tests pass, update PROGRESS.md, then write overnight/DONE."
    else
      base="Continue the overnight build from where you left off (it is $(date +%H:%M); the run ends at $END_HHMM). Re-read PROGRESS.md, then keep executing the plan. Remember the time box in docs/LOCAL_RUN.md."
    fi
    msg=""
    if [ -s "$O/INBOX.md" ]; then
      msg=$(cat "$O/INBOX.md"); mv "$O/INBOX.md" "$O/inbox_archive/$(date +%H%M%S).md"
      base="$base

SUPERVISOR GUIDANCE (from the human's assistant, act on it and acknowledge it in PROGRESS.md):
$msg"
      log "turn $turn: delivering supervisor guidance"
    fi
    log "turn $turn: resume"
    run_turn "$base" --resume "$ID"; rc=$?
  fi
  if [ $rc -ne 0 ]; then
    log "turn $turn exited rc=$rc (likely rate/usage limit or error) - backing off 10 min"; sleep 600
  else
    log "turn $turn ended cleanly"; sleep 15
  fi
done
[ -f "$O/DONE" ] && log "DONE: $(cat "$O/DONE")" || log "deadline reached without DONE"
rm -f "$O/driver.pid"
