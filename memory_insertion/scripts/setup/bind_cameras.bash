#!/bin/bash
sudo rfcomm release 1
sudo rfcomm release 2
sleep 1
# sudo rfcomm bind 1 08:3A:F2:44:6F:EE
# sudo rfcomm bind 2 08:3A:F2:45:06:E2
sudo rfcomm -i hci0 bind rfcomm1 08:3A:F2:44:6F:EE 1
sudo rfcomm -i hci0 bind rfcomm2 08:3A:F2:45:06:E2 1
