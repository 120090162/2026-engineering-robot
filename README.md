# 2026-engineering-robot

编译
```bash
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 10
source install/setup.bash
ros2 launch franka_keyboard_control franka_interface.launch.py
sudo chmod 777 /dev/ttyACM0
ros2 run franka_keyboard_control keyboard_servo_node

```

设置串口权限
```bash
sudo chmod 777 ./script/create_udev_rules.sh
./script/create_udev_rules.sh
```
