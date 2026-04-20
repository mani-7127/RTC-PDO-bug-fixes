#!/bin/bash
# setup_log_tmpfs.sh
# Mounts a RAM-backed tmpfs for jamun logs so log writes never hit the SD card.
# This eliminates the main source of HMI lag and PDO watchdog overruns.
#
# Run once as root, or add to /etc/fstab for persistence across reboots.
#
# Usage:
#   sudo bash setup_log_tmpfs.sh
#
# To make permanent, add to /etc/fstab:
#   tmpfs /mnt/app/jamun/logs tmpfs defaults,size=64m,mode=0777 0 0

LOG_DIR="/mnt/app/jamun/logs"

mkdir -p "$LOG_DIR"

if mountpoint -q "$LOG_DIR"; then
    echo "tmpfs already mounted at $LOG_DIR"
else
    mount -t tmpfs -o size=64m,mode=0777 tmpfs "$LOG_DIR"
    echo "tmpfs mounted at $LOG_DIR (64MB RAM)"
fi

echo "Log writes will now go to RAM instead of SD card."
echo "Logs are lost on reboot — copy important logs before rebooting."
echo ""
echo "To make permanent, add to /etc/fstab:"
echo "  tmpfs $LOG_DIR tmpfs defaults,size=64m,mode=0777 0 0"