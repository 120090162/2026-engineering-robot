# 2026-engineering-robot

编译
```bash
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 10
source install/setup.bash # zsh
ros2 launch franka_keyboard_control franka_interface.launch.py
sudo chmod 777 /dev/ttyACM0
ros2 run franka_keyboard_control keyboard_servo_node # rc 控制 / 键盘控制
# or
ros2 run franka_keyboard_control rm_servo_keyboard_input_test # 键盘控制

```

一键启动
```bash
# note: 如果要用键盘控制，不能使用这个方案
ros2 launch franka_keyboard_control bringup.launch.py
```

设置串口权限
```bash
sudo chmod 777 ./script/create_udev_rules.sh
./script/create_udev_rules.sh
```

设置程序自启动与看门狗逻辑, refer to https://github.com/CSU-FYT-Vision/FYT2024_vision
```bash
touch screen.output
cd src/rm_upstart
sudo chmod +x ./register_service.sh
sudo ./register_service.sh

# 正常时有如下输出
# Creating systemd service file at /etc/systemd/system/rm.service...
# Reloading systemd daemon...
# Enabling service rm.service...
# Starting service rm.service...
# Service rm.service has been registered and started.
```

```bash
systemctl status rm # 查看自启动服务运行情况
systemctl stop rm # 暂时关闭自启动服务
systemctl disable rm # 使自启动服务失效
```