echo 'KERNEL=="ttyACM*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", ATTRS{serial}=="0002", MODE:="0777", GROUP:="dialout", SYMLINK+="stm32_chassis"' > /etc/udev/rules.d/robot.rules
service udev reload
sleep 2
service udev restart