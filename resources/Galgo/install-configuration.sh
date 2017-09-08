#!/bin/sh

# This script modifies system configuration to required by Galgo.
# Install it on **robot Galgo's computer** and reboot.
# Requires sudo privileges.

sudo cp ./udevadm-trigger.sh /usr/local/sbin/udevadm-trigger.sh
sudo cp ./98-adafruit-ft232h.rules /etc/udev/rules.d/98-adafruit-ft232h.rules
sudo cp ./rc.local /etc/rc.local
