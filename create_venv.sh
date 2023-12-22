#!/usr/bin/env bash
#
# Creates a virtual environment that handles Python module dependencies for all packages necessary to run DRL local planner
#
SCRIPT_DIR=$(realpath $(dirname $0))

cd $SCRIPT_DIR
virtualenv -p python3 .venv
source .venv/bin/activate

pip3 install -r requirements.txt

# Extracted from the requirements.txt as on some platforms this error may show up:
#   ERROR: Could not find a version that satisfies the requirement pkg-resources==0.0.0 (from versions: none)
#   ERROR: No matching distribution found for pkg-resources==0.0.0
# which breaks the whole installation
pip3 install pkg-resources==0.0.0

# prepare rl_agent sources
cd $SCRIPT_DIR/rl_agent
python setup.py install

deactivate
