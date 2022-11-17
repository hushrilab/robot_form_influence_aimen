#!/bin/bash
set -v
set -e
sudo apt install -y ros-kinetic-moveit-commander 
sudo cp talos.srdf /opt/pal/erbium/share/talos_moveit_config/config/talos.srdf
