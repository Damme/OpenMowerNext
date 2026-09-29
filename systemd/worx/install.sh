#!/bin/bash
# Installs the om2 systemd units on the Pi (run there with sudo from this folder).
#   sudo ./install.sh            install + enable (start at next boot)
#   sudo ./install.sh --now      also start now (stop the hand-started str2str first!)
set -e
cd "$(dirname "$0")"
if [ ! -f /opt/om2/config/ntrip.env ]; then
  echo "missing /opt/om2/config/ntrip.env, create it first:" >&2
  echo "  echo 'NTRIP_URL=ntrip://user:password@host/MOUNT' > /opt/om2/config/ntrip.env; chmod 600 /opt/om2/config/ntrip.env" >&2
  exit 1
fi
install -m 644 om2-gps-serial.service om2-gps-ntrip.service om2-robot.service /etc/systemd/system/
systemctl daemon-reload
systemctl enable om2-gps-serial.service om2-gps-ntrip.service om2-robot.service
if [ "$1" = "--now" ]; then
  if pgrep -x str2str >/dev/null; then
    echo "str2str already running (start_gps.sh?) - stop it first: pkill -x str2str" >&2
    exit 1
  fi
  systemctl start om2-gps-serial.service om2-gps-ntrip.service om2-robot.service
fi
systemctl --no-pager status om2-gps-serial.service om2-gps-ntrip.service om2-robot.service | cat
