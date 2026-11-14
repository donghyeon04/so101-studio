#!/usr/bin/env bash
# Runs so101_studio in a conda env, from any folder (no conda activate needed).
# Env name: SO101_ENV (default so101). Or give the env folder directly with SO101_ENV_DIR.
set -e
ENV_NAME="${SO101_ENV:-so101}"
if [ -z "${SO101_ENV_DIR:-}" ]; then
  for root in "$HOME/miniforge3" "$HOME/miniconda3" "$HOME/anaconda3" "/opt/conda" "/opt/miniforge3"; do
    if [ -x "$root/envs/$ENV_NAME/bin/python" ]; then SO101_ENV_DIR="$root/envs/$ENV_NAME"; break; fi
  done
fi
if [ -z "${SO101_ENV_DIR:-}" ]; then
  echo "conda env \"$ENV_NAME\" not found. Create it: conda env create -f environment.yml (or set SO101_ENV / SO101_ENV_DIR)" >&2
  exit 1
fi
if [ "$(uname -s)" = "Linux" ] && ! id -nG | tr ' ' '\n' | grep -qx dialout; then
  echo "serial permission: add yourself to the dialout group, then log in again:  sudo usermod -aG dialout \$USER" >&2
fi
cd "$(dirname "${BASH_SOURCE[0]}")"
# Run this folder's code even if the editable install points at another folder (worktrees share one conda env).
export PYTHONPATH="$PWD/src${PYTHONPATH:+:$PYTHONPATH}"
exec "$SO101_ENV_DIR/bin/python" -m so101_studio "$@"
