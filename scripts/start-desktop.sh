#!/usr/bin/env bash
set -euo pipefail

# Lightweight software-rendered X11 display, viewable from macOS via noVNC.
Xvfb :99 -screen 0 1440x900x24 -ac +extension GLX +render -noreset \
  > /tmp/xvfb.log 2>&1 &
sleep 2
openbox > /tmp/openbox.log 2>&1 &
x11vnc -display :99 -localhost -rfbport 5900 -forever -shared \
  -nopw -noxdamage -quiet > /tmp/x11vnc.log 2>&1 &
exec websockify --web=/usr/share/novnc 0.0.0.0:6080 127.0.0.1:5900
