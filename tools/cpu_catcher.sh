#!/bin/bash
# Catch short-lived CPU spikes that vanish before `ps` can be run by hand.
#
# Three times on 2026-08-09 a process hit ~100% of a core and was gone by the
# time it could be inspected (twice as "npm", once as "pemmican-cli"). This
# samples /proc directly every second and, on catching one, immediately records
# its full command line AND its parent chain - the parent is the interesting
# part, because it says what spawned it.

THRESH=${THRESH:-55}          # percent of one core
HZ=$(getconf CLK_TCK)
# The known-heavy ROS processes; everything else is a candidate, including claude.
IGNORE='rtabmap|component_container|wheel_odometry|MicroXRCEAgent|rc_control_node|depthimage_to_laserscan|pointcloud_to_laserscan|static_transform|cpu_catcher'

declare -A prev
while :; do
    declare -A cur
    for st in /proc/[0-9]*/stat; do
        [ -r "$st" ] || continue
        read -r line < "$st" 2>/dev/null || continue
        pid=${line%% *}
        rest=${line#*) }
        set -- $rest
        cur[$pid]=$(( ${12} + ${13} ))     # utime + stime, after the (comm) field
    done
    sleep 1
    for st in /proc/[0-9]*/stat; do
        [ -r "$st" ] || continue
        read -r line < "$st" 2>/dev/null || continue
        pid=${line%% *}
        rest=${line#*) }
        set -- $rest
        now=$(( ${12} + ${13} ))
        old=${prev[$pid]:-${cur[$pid]:-}}
        [ -z "$old" ] && continue
        pct=$(( 100 * (now - old) / HZ ))
        if [ "$pct" -ge "$THRESH" ]; then
            cmd=$(tr '\0' ' ' < /proc/$pid/cmdline 2>/dev/null | cut -c1-160)
            [ -z "$cmd" ] && continue
            echo "$cmd" | grep -qE "$IGNORE" && continue
            ppid=$(awk '{print $4}' /proc/$pid/stat 2>/dev/null)
            pcmd=$(tr '\0' ' ' < /proc/$ppid/cmdline 2>/dev/null | cut -c1-120)
            gpid=$(awk '{print $4}' /proc/$ppid/stat 2>/dev/null)
            gcmd=$(tr '\0' ' ' < /proc/$gpid/cmdline 2>/dev/null | cut -c1-120)
            echo "CAUGHT $(date +%H:%M:%S)  ${pct}%  pid=$pid"
            echo "   cmd    : $cmd"
            echo "   parent : [$ppid] $pcmd"
            echo "   gparent: [$gpid] $gcmd"
        fi
    done
    for k in "${!cur[@]}"; do prev[$k]=${cur[$k]}; done
done
