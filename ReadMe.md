# Autonomous Tracktor Architecture

![alt text](./doc/Architecture.jpg)

### using virtualenv (recommend)

virtualenv voiceassistant.venv
source voiceassistant.venv/bin/activate



### To install all needed python libraries run:

sudo /usr/bin/python3 -m pip install -r ~/Autonomous-Tractor/requirements.txt


### Usage

Autopilot Mode:
/bin/python3 ~/Autonomous-Tractor/core.py


Manual + FPV Mode:
/bin/python3 ~/Autonomous-Tractor/Mission/server.py
/bin/python3 ~/Autonomous-Tractor/Mission/client.py