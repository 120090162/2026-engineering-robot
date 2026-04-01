# 2026-engineering-robot

编译
```bash
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 10
source install/setup.bash # zsh
ros2 launch franka_keyboard_control franka_interface.launch.py
sudo chmod 777 /dev/ttyACM0
ros2 run franka_keyboard_control keyboard_servo_node # rc 控制
# or
ros2 run franka_keyboard_control rm_servo_keyboard_input_test # 键盘控制

```

设置串口权限
```bash
sudo chmod 777 ./script/create_udev_rules.sh
./script/create_udev_rules.sh
```
