#!/bin/sh
# One joint closing its loop on a MuJoCo arm (sim/mujoco/arm.xml), in lockstep. Needs `sim` on PATH, sim/build/joint.elf from
# `sh firmware/build.sh`, and a venv with mujoco, numpy and matplotlib: VENV=<dir> (python -m venv $VENV; $VENV/bin/pip install mujoco numpy matplotlib pillow).
# sim embeds the same Python version as the venv's, so PYTHONPATH points at the venv's site-packages.
# Output: sim/out/arm-sim.log (ARM t=... every 10 ms), sim/screenshots/arm-step.png; exit 1 unless SETTLED.
set -e
cd "$(dirname "$0")/.."
: "${VENV:?set VENV to a venv with mujoco installed}"
SP=$(ls -d "$VENV"/lib/python3.*/site-packages)
mkdir -p sim/out
# ENC_OFFSET_RAD: the encoder reads pi at the rest pose, matching ENC_REST_ANGLE in the firmware (default pi)
ENC_OFFSET_RAD=3.14159 PYTHONPATH=$SP "${SIM:-sim}" run --scenario sim/scenario.yaml --usb-cdc-console dut.usb --timeout 14 --ascii \
  --expect "JOINT 1 ready" > sim/out/arm-uart.log 2> sim/out/arm-sim.log
"$VENV/bin/python" sim/plot.py | tee sim/out/arm-result.txt
grep -q '^SETTLED' sim/out/arm-result.txt
