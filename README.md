# 安装ROS
  [Ubuntu22.04.1安装ROS2入门级教程(ros-humble)_ros humble_Python-AI Xenon的博客-CSDN博客](https://blog.csdn.net/yxn4065/article/details/127352587)

# 创建工作空间

  ```Shell
mkdir -p ~/AUTO_AIM
```

# 下载源代码

  在 `AUTO_AIM` 目录下

  ```Shell
git clone 
```


  ```Shell
sudo apt install ros-humble-foxglove-bridge
```


# 编译

  在 `ros_ws` 目录下

  ```Shell
rosdep install --from-paths src --ignore-src -r -y
```


  ```Shell
colcon build --symlink-install
```


# 运行节点

  ```Shell
sudo chmod 777 /dev/ttyACM0
```


  运行每个节点，必须新建终端并输入命令，且运行前需要执行 `source install/setup.bash`

  ```Shell
source install/setup.bash
ros2 launch rm_vision_bringup no_hardware.launch.py
```


  ```Shell
source install/setup.bash
ros2 launch rm_vision_bringup vision_bringup.launch.py
```


  - 单独运行子模块（一般用不上，写在这只为了有时开发要调用 rv 独立模块调试）

    ```Shell
source install/setup.bash
ros2 launch auto_aim_bringup auto_aim.launch.py 
```


    ```Shell
source install/setup.bash
ros2 launch hik_camera hik_camera.launch.py
```


    ```Shell
source install/setup.bash
ros2 launch rm_serial_driver serial_driver.launch.py
```


# 启动可视化

  打开新的终端

  ```Shell
source install/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```


  使用 Chromium 浏览器打开 [https://studio.foxglove.dev/](https://studio.foxglove.dev/) ，并打开 connection

  > 建议下载**[foxglove客户端](https://foxglove.dev/download)**，避免Web端视频闪烁


# ROSbridge的参数

[ROSbridge的参数](https://flowus.cn/86e7e54f-a0fc-467d-909d-95d1509f62f2)

# 【拓展】其他操作

  ## 查看相机帧率

    ```Shell
ros2 topic hz /camera_info
```


  ## 关闭所有节点

    ```Shell
ros2 node killall
```




