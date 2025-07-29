#!/bin/bash
set -e

# Gerekli ROS ortamını kesin olarak hazırla
source /opt/ros/humble/setup.bash

echo "Starting simulation components..."
# Simülasyon bileşenlerini arka planda başlat (&)
gzserver -s libgazebo_ros_init.so -s libgazebo_ros_factory.so &
(cd /root/PX4-Autopilot && export PX4_SIM_MODEL=standard_plane && build/px4_sitl_default/bin/px4 ROMFS/px4fmu_common -s etc/init.d-posix/rcS) &
micrortps_agent -t UDP &

# Her şeyin başlaması için 60 saniye bekle
echo "Waiting 60 seconds for simulation to initialize..."
sleep 60

# Kendi workspace'imizi tanıt ve testi çalıştır
echo "Sourcing workspace and running the test..."
source /root/ros2_ws/install/setup.bash
colcon test --packages-select test_package --ctest-args '-R test_task_1_arm_disarm'
