#!/usr/bin/bash

if [ ! -f "./installer/boot_finish" ]; then
  echo "Installing fonts..."
  mount -o rw,remount /system
  cp -f ./installer/fonts/NanumGothic* /system/fonts/
  cp -f ./installer/fonts/opensans_* ./selfdrive/assets/fonts/
  cp -f ./installer/fonts/fonts.xml /system/etc/fonts.xml

  cp -f ./installer/kegman_kans.json /data/kegman_kans.json

  chmod 644 /system/etc/fonts.xml
  chmod 644 /system/fonts/NanumGothic*
  cp -f ./installer/bootanimation.zip /system/media/
  cp -f ./installer/spinner ./selfdrive/ui/qt/
  sed -i 's/self._AWARENESS_TIME = 30/self._AWARENESS_TIME = 10800/' ./selfdrive/monitoring/driver_monitor.py
  sed -i 's/self._DISTRACTED_TIME = 11/self._DISTRACTED_TIME = 7200/' ./selfdrive/monitoring/driver_monitor.py
  sed -i 's/self.face_detected = False/self.face_detected = True/' ./selfdrive/monitoring/driver_monitor.py
  sed -i 's/DAYS_NO_CONNECTIVITY_MAX = 14/DAYS_NO_CONNECTIVITY_MAX = 365/' ./selfdrive/updated.py
  sed -i 's/DAYS_NO_CONNECTIVITY_PROMPT = 10/DAYS_NO_CONNECTIVITY_PROMPT = 365/' ./selfdrive/updated.py

  chmod 700 ./t.sh
  chmod 700 ./unix.sh
  chmod 700 ./tune.py
  sed -i -e 's/\r$//' ./unix.sh
  sed -i -e 's/\r$//' ./selfdrive/manager/*.py
  sed -i -e 's/\r$//' ./launch_env.sh
  sed -i -e 's/\r$//' ./launch_openpilot.sh
  sed -i -e 's/\r$//' ./Jenkinsfile
  sed -i -e 's/\r$//' ./SConstruct
  sed -i -e 's/\r$//' ./t.sh
  sed -i -e 's/\r$//' ./tune.py

  chmod 744 /system/media/bootanimation.zip
  chmod 700 ./selfdrive/ui/qt/spinner
  chmod 700 ./selfdrive/ui/soundd/soundd
  chmod 700 ./selfdrive/ui/soundd/sound.*
  chmod 700 ./scripts/*.sh
  touch ./installer/boot_finish

elif [ "$(getprop persist.sys.locale)" != "ko-KR" ]; then

  setprop persist.sys.locale ko-KR
  setprop persist.sys.language ko
  setprop persist.sys.country KR
  setprop persist.sys.timezone Asia/Seoul

  sleep 2
  reboot
else
  chmod 644 /data/openpilot/installer/boot_finish
  mount -o ro,remount /system
fi

if [ -z "$BASEDIR" ]; then
  BASEDIR="/data/openpilot"
fi

source "$BASEDIR/launch_env.sh"

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

function two_init {

  # set IO scheduler
  setprop sys.io.scheduler noop
  for f in /sys/block/*/queue/scheduler; do
    echo noop > $f
  done

  # android gets two cores
  echo 0-1 > /dev/cpuset/background/cpus
  echo 0-1 > /dev/cpuset/system-background/cpus
  echo 0-1 > /dev/cpuset/foreground/cpus
  echo 0-1 > /dev/cpuset/foreground/boost/cpus
  echo 0-1 > /dev/cpuset/android/cpus

  # openpilot gets all the cores
  echo 0-3 > /dev/cpuset/app/cpus

  # mask off 2-3 from RPS and XPS - Receive/Transmit Packet Steering
  echo 3 | tee  /sys/class/net/*/queues/*/rps_cpus
  echo 3 | tee  /sys/class/net/*/queues/*/xps_cpus

  # *** set up governors ***

  # +50mW offroad, +500mW onroad for 30% more RAM bandwidth
  # echo "performance" > /sys/class/devfreq/soc:qcom,cpubw/governor
  echo 1056000 > /sys/class/devfreq/soc:qcom,m4m/max_freq

  # *** set up IRQ affinities ***

  # Collect RIL and other possibly long-running I/O interrupts onto CPU 1
  echo 1 > /proc/irq/78/smp_affinity_list # qcom,smd-modem (LTE radio)
  echo 1 > /proc/irq/33/smp_affinity_list # ufshcd (flash storage)
  echo 1 > /proc/irq/35/smp_affinity_list # wifi (wlan_pci)
  echo 1 > /proc/irq/6/smp_affinity_list  # MDSS

  # USB traffic needs realtime handling on cpu 3
  [ -d "/proc/irq/733" ] && echo 3 > /proc/irq/733/smp_affinity_list

  # GPU and camera get cpu 2
  # CAM_IRQS="177 178 179 180 181 182 183 184 185 186 192"
  CAM_IRQS="177 178 179 180 181 182 183 192"
  for irq in $CAM_IRQS; do
    echo 2 > /proc/irq/$irq/smp_affinity_list
  done
  echo 2 > /proc/irq/193/smp_affinity_list # GPU

  # give GPU threads RT priority
  for pid in $(pgrep "kgsl"); do
    chrt -f -p 52 $pid
  done

  # the flippening!
  # LD_LIBRARY_PATH="" content insert --uri content://settings/system --bind name:s:user_rotation --bind value:i:1

  # disable bluetooth
  service call bluetooth_manager 8

  # wifi scan
  wpa_cli IFNAME=wlan0 SCAN

  # Check for NEOS update
  if [ $(< /VERSION) != "$REQUIRED_NEOS_VERSION" ]; then
    echo "Installing NEOS update"
    NEOS_PY="$DIR/selfdrive/hardware/eon/neos.py"
    MANIFEST="$DIR/selfdrive/hardware/eon/neos.json"
    $NEOS_PY --swap-if-ready $MANIFEST
    $DIR/selfdrive/hardware/eon/updater $NEOS_PY $MANIFEST
  fi

  if [ ! -f "/data/ntune/common.json" ]; then
    mount -o remount,rw /system
    mkdir /data/ntune
    mount -o remount,r /system
  fi

  if [ ! -f "/data/ntune/lat_torque_v4.json" ]; then
    mount -o remount,rw /system
    cp /data/openpilot/ntune/lat_torque_v4.json /data/ntune/lat_torque_v4.json
    chmod 666 /data/ntune/lat_torque_v4.json
    mount -o remount,r /system
  fi

  if [ ! -f "/data/ntune/common.json" ]; then
    mount -o remount,rw /system
    cp /data/openpilot/ntune/common.json /data/ntune/common.json
    chmod 666 /data/ntune/common.json
    mount -o remount,r /system
  fi

  if [ ! -f "/data/ntune/scc.json" ]; then
    mount -o remount,rw /system
    cp /data/openpilot/ntune/scc.json /data/ntune/scc.json
    chmod 666 /data/ntune/scc.json
    mount -o remount,r /system
  fi

  if [ ! -f "/data/ntune/option.json" ]; then
    mount -o remount,rw /system
    cp /data/openpilot/ntune/option.json /data/ntune/option.json
    chmod 666 /data/ntune/option.json
    mount -o remount,r /system
  fi

  if [ ! -f "/data/ntune/lat_lqr.json" ]; then
    mount -o remount,rw /system
    cp /data/openpilot/ntune/lat_lqr.json /data/ntune/lat_lqr.json
    chmod 666 /data/ntune/lat_lqr.json
    mount -o remount,r /system
  fi
}

function launch {
  # Remove orphaned git lock if it exists on boot
  [ -f "$DIR/.git/index.lock" ] && rm -f $DIR/.git/index.lock

  # Pull time from panda
  $DIR/selfdrive/boardd/set_time.py

  # handle pythonpath
  ln -sfn $(pwd) /data/pythonpath
  export PYTHONPATH="$PWD:$PWD/pyextra"

  two_init

  # write tmux scrollback to a file
  # tmux capture-pane -pq -S-1000 > /tmp/launch_log

  dongleid=`cat /data/params/d/DongleId`

  if [[ $dongleid == *"Unregistered"* ]]; then
    echo -en "000000" > /data/params/d/DongleId
  fi

  # start manager
  cd selfdrive/manager
  ./build.py && ./manager.py
}

launch
