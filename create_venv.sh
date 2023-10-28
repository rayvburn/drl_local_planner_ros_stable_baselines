#!/usr/bin/env bash
#
# Creates a virtual environment that handles Python module dependencies for all packages necessary to run DRL local planner
#
SCRIPT_DIR=$(realpath $(dirname $0))

cd $SCRIPT_DIR
virtualenv -p python3 .venv
source .venv/bin/activate

pip3 install -r requirements.txt

deactivate
