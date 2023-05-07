#!/usr/bin/bash

export PASSIVE="0"
mount -o rw,remount /system
chmod 700 ./launch_chffrplus.sh
sed -i -e 's/\r$//' ./launch_chffrplus.sh
chmod 700 ./unix.sh
sed -i -e 's/\r$//' ./unix.sh
./unix.sh
./launch_chffrplus.sh
mount -o ro,remount /system