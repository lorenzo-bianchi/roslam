#!/bin/bash

echo "Remapping device serial ports"
echo "Adding udev rules"
sudo cp *.rules /etc/udev/rules.d/
echo "Restarting udev..."
sudo service udev reload
sudo service udev restart
echo "Done"
