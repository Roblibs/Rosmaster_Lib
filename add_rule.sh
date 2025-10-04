#!/usr/bin/env bash
# add_rule.sh - Create a persistent udev symlink for the Rosmaster board.
# Usage: sudo ./add_rule.sh myserial
# Optional second argument: vendor:product (default 1a86:7523 for CH340/CH34x)
# Example with custom name and default VID:PID:
#   sudo ./add_rule.sh my_ros_board
# Example with custom VID:PID (hex):
#   sudo ./add_rule.sh my_ros_board 1a86:7523
# After running, replug the device or run:
#   sudo udevadm control --reload-rules && sudo udevadm trigger
# Then verify:
#   ls -l /dev/<name>

set -euo pipefail

if [[ $EUID -ne 0 ]]; then
  echo "[ERROR] Please run as root (use sudo)." >&2
  exit 1
fi

NAME="${1:-}"
if [[ -z "$NAME" ]]; then
  echo "Usage: sudo $0 <symlink_name> [vendor:product]" >&2
  exit 1
fi

VIDPID="${2:-1a86:7523}"
if [[ ! "$VIDPID" =~ ^[0-9a-fA-F]{4}:[0-9a-fA-F]{4}$ ]]; then
  echo "[ERROR] vendor:product must look like 1a86:7523" >&2
  exit 1
fi
VID="${VIDPID%%:*}"
PID="${VIDPID##*:}"

RULE_FILE="/etc/udev/rules.d/99-rosmaster.rules"
RULE_LINE="KERNEL==\"ttyUSB*\", ATTRS{idVendor}\"==\"$VID\", ATTRS{idProduct}\"==\"$PID\", MODE:=\"0777\", SYMLINK+=\"$NAME\""

# Write (replace or append uniquely)
if [[ -f "$RULE_FILE" ]]; then
  # Remove any previous line for this name
  grep -v "SYMLINK+=\\\"$NAME\\\"" "$RULE_FILE" > "${RULE_FILE}.tmp" || true
  mv "${RULE_FILE}.tmp" "$RULE_FILE"
fi

echo "$RULE_LINE" >> "$RULE_FILE"

echo "[INFO] Rule written to $RULE_FILE:" >&2
cat "$RULE_FILE" >&2

echo "[INFO] Reloading udev rules..." >&2
udevadm control --reload-rules
udevadm trigger || true

echo "[INFO] Done. Unplug and replug the board if the symlink isn't present yet." >&2

echo "[INFO] Verify with: ls -l /dev/$NAME" >&2
