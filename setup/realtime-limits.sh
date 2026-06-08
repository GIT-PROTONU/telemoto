#!/usr/bin/env bash
# Let the realtime kernel actually be used for low-jitter control.
#
# WHY: `ulimit -r` is currently 0, so SCHED_FIFO is forbidden — MoveIt Servo's
# servo loop (thread_priority 40, SCHED_FIFO) silently falls back to normal
# scheduling and can be preempted by RViz/move_group, causing jitter. Granting
# rtprio unlocks the RT kernel you already booted (6.x-realtime).
#
# RUN:  sudo bash setup/realtime-limits.sh   # then LOG OUT and back in
set -euo pipefail
[[ $EUID -eq 0 ]] || { echo "run with sudo"; exit 1; }

USER_NAME="${SUDO_USER:-$USER}"
LIMITS=/etc/security/limits.d/99-telamoto-realtime.conf

cat > "$LIMITS" <<'EOF'
# Telamoto realtime: allow SCHED_FIFO + locked memory for low-jitter control.
@realtime   -   rtprio      98
@realtime   -   memlock     unlimited
EOF

getent group realtime >/dev/null || groupadd realtime
usermod -aG realtime "$USER_NAME"

echo "[rt] wrote $LIMITS; added '$USER_NAME' to group 'realtime'."
echo "[rt] LOG OUT and back in, then verify:  ulimit -r   (expect 98)"
cat <<'EOF'
[rt] OPTIONAL (advanced, needs a reboot): dedicate a CPU core to the control
     loop to eliminate preemption entirely —
       1. add  isolcpus=3 nohz_full=3 rcu_nocbs=3  to the kernel cmdline
          (edit /etc/default/grub GRUB_CMDLINE_LINUX, run update-grub, reboot)
       2. pin the controller to it, e.g. wrap the node launch with  taskset -c 3
     Skip unless you still see jitter after the rtprio + direct-link fixes.
EOF
