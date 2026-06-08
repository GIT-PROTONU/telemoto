#!/usr/bin/env bash
# Dedicated, direct, low-jitter link from the PC's onboard NIC to the UR10.
#
# WHY: today the robot (192.168.10.2) is reached through the USB Ethernet
# adapter on the home network, hairpinning via the home router — a USB-polling +
# extra-hop source of latency/jitter that hurts realtime control. The onboard
# PCIe NIC (eno1) cabled straight into the robot is direct and deterministic.
#
# RUN (after plugging eno1 directly into the UR10 controller):
#   sudo IFACE=eno1 bash setup/robot-net.sh
# Leaves the USB adapter (and your internet) completely untouched.
set -euo pipefail

IFACE="${IFACE:-eno1}"               # onboard PCIe NIC for the robot
HOST_IP="${HOST_IP:-192.168.10.1}"   # this PC, on the robot subnet
PROFILE="telamoto-robot"

[[ $EUID -eq 0 ]] || { echo "run with sudo"; exit 1; }
echo "[net] configuring $IFACE -> $HOST_IP/24 (direct robot link)"

# Persistent NetworkManager profile: static IP, never a default route, no DNS,
# IPv6 off — keeps this interface off the routing/DNS path entirely.
if ! nmcli -g NAME connection show | grep -qx "$PROFILE"; then
  nmcli connection add type ethernet ifname "$IFACE" con-name "$PROFILE" \
    ipv4.method manual ipv4.addresses "$HOST_IP/24"
fi
nmcli connection modify "$PROFILE" \
  ipv4.method manual ipv4.addresses "$HOST_IP/24" \
  ipv4.never-default yes ipv4.ignore-auto-dns yes ipv6.method disabled \
  connection.autoconnect yes
nmcli connection up "$PROFILE"

# Low-latency NIC tuning (best-effort — not every NIC exposes every knob).
ethtool -C "$IFACE" rx-usecs 0 tx-usecs 0 adaptive-rx off adaptive-tx off 2>/dev/null \
  || echo "[net] interrupt-coalescing tuning unsupported on $IFACE — skipped"
ethtool -A "$IFACE" rx off tx off 2>/dev/null \
  || echo "[net] pause-frame tuning unsupported on $IFACE — skipped"

echo "[net] link state:"
ethtool "$IFACE" 2>/dev/null | grep -iE "speed|duplex|link detected" || true
cat <<'EOF'
[net] done.
      On the pendant: Setup Robot > Network > Static address
        IP 192.168.10.2  mask 255.255.255.0  gateway (blank)
      Then verify from the PC:  ping -c3 192.168.10.2
EOF
