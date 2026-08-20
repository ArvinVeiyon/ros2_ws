#!/usr/bin/env bash
# Watch for FC reboots while provoking the hardfault.
#
# An FC reboot shows up on the companion as the uXRCE-DDS client re-establishing
# its session (a fresh create_participant). That is the cheapest reboot detector
# we have - it needs nothing from the FC and survives the FC being gone.
#
# 2026-08-16: nine of these in ~65 min were misread as "DDS session instability"
# for most of a session. They were FC reboots. Do not repeat that mistake - a
# reconnect here means THE FC RESTARTED.
#
#   tools/fc_reboot_watch.sh          # follow live
#   tools/fc_reboot_watch.sh --since '10:00'
set -u
SINCE="${2:-now}"
echo "watching for FC reboots (DDS re-establish). Ctrl-C to stop."
echo "baseline reconnects this boot:"
journalctl -u microxrce-agent -b --no-pager 2>/dev/null \
  | grep 'create_participant' | awk '{print "  " $1, $2, $3}'
echo "--- live ---"
journalctl -u microxrce-agent -f --since "$SINCE" --no-pager 2>/dev/null \
  | grep --line-buffered 'create_participant' \
  | while read -r line; do
      printf '\a>>> %s  FC REBOOTED <<<\n' "$(date '+%H:%M:%S')"
    done
