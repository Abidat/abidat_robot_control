#!/bin/bash

sudo cp -uv 99-uvc.rules  /etc/udev/rules.d/99-uvc.rules
sudo cp -uv servomotor.rules  /etc/udev/rules.d/servomotor.rules

sudo udevadm control --reload-rules 
sudo udevadm trigger

echo 'successfully finished the udev config'
