# 简介
本项目源自于RV，将识别器，预测器，弹道结算全放在了上位机，在机器人运动控制做好的情况下，一辆车的配适时间压缩到半小时，视觉从0上手跳车只需要半天。

# 调车
### 1.相机标定
```bash
ros2 launch hik_camera_ros2_driver hik_camera_launch.py

ros2 run camera_calibration cameracalibrator --size 7x10 --square 0.015 image:=/camera/image camera:=/camera
```
`size` 为棋盘格交点的数量 `width x height`

`square` 为一个黑色棋子的大小，单位为 m

### 2.PNP检测：PNP结算的距离与实际距离在7m,5m,3m,1.5m时的误差
### 3.urdf：根据实际测量修改
### 4.IMU与相机多传感修正：目标不动，本车动，看position的变化
### 5.打静止装甲板，修正到目标中心
### 6.不同距离下打旋转装甲板


# 使用
### 安装ROS
  [Ubuntu22.04.1安装ROS2入门级教程(ros-humble)_ros humble_Python-AI Xenon的博客-CSDN博客](https://blog.csdn.net/yxn4065/article/details/127352587)

### 创建工作空间

  ```Shell
mkdir -p ~/AUTO_AIM
```

### 下载源代码

  在 `AUTO_AIM` 目录下

  ```Shell
git clone git@github.com:MzKyle/Observation-of-the-whole-vehicle.git
```


  ```Shell
sudo apt install ros-humble-foxglove-bridge
```


### 编译

  在 `ros_ws` 目录下

  ```Shell
rosdep install --from-paths src --ignore-src -r -y
```


  ```Shell
colcon build --symlink-install
```


### 运行节点

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


  - 单独运行子模块

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


