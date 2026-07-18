#!/usr/bin/env bash
# Convenience wrapper. On a machine with `iverilog` or `verilator` already on
# PATH you don't need this -- just `cd tb && make` (Icarus) or
# `cd tb && SIM=verilator python3 runner.py`.
set -e
export SIM="${SIM:-verilator}"
cd "$(dirname "$0")/tb"
python3 runner.py "$@"
