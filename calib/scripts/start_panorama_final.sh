#!/bin/bash
CONFIG=${1:-/data/local/tmp/kalibr_cam_chain_a4.yaml}
WIDTH=${2:-960}
HEIGHT=${3:-240}
FPS=${4:-15}
BLEND=${5:-0.10}
VFOV=${6:-90}
PORT=${7:-8092}

screen -S pano_eq -X quit >/dev/null 2>&1 || true
screen -S dash -X quit >/dev/null 2>&1 || true
sleep 1
screen -wipe >/dev/null 2>&1

setsid screen -dmS pano_eq bash -c "exec /data/local/tmp/start_pano_q1.sh $CONFIG $WIDTH $HEIGHT 4 /tmp/pano_eq.log $FPS 0.20 $BLEND $VFOV" </dev/null >/dev/null 2>&1
sleep 3
setsid screen -dmS dash bash -c "exec /data/local/tmp/run_preview_dashboard.sh --port $PORT >>/tmp/dash.log 2>&1" </dev/null >/dev/null 2>&1
sleep 2
if ! pgrep -f preview_dashboard.py >/dev/null 2>&1; then
  setsid screen -dmS dash bash -c "exec /data/local/tmp/run_preview_dashboard.sh --port $PORT >>/tmp/dash.log 2>&1" </dev/null >/dev/null 2>&1
fi
if pgrep -f 'roslaunch.*panorama_q1' >/dev/null 2>&1; then echo "pano_launch=running"; else echo "pano_launch=not_running"; fi
if pgrep -f preview_dashboard.py >/dev/null 2>&1; then echo "dashboard=running"; else echo "dashboard=not_running"; fi
exit 0