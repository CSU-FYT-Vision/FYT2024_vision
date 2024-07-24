# FYT2024 Vision Project

 中南大学FYT战队24赛季视觉项目主仓库，该项目在原rm_vision项目上扩展了自瞄选板、能量机关识别与预测、哨兵定位、自主导航等功能，为RoboMaster机器人实现了一套通用的算法框架


## 一、项目结构

*表示不在本仓库中直接提供，这部分模块已在[CSU-RM-Sentry](https://github.com/baiyeweiguang/CSU-RM-Sentry)开源

```
.
│
├── rm_bringup (启动及参数文件)
│
├── rm_robot_description (机器人urdf文件，坐标系的定义)
│
├── rm_interfaces (自定义msg、srv)
│
├── rm_hardware_driver
│   ├── livox_ros_driver2 (*Livox激光雷达驱动)
│   │
│   ├── rm_camera_driver (相机驱动)
│   │
│   └── rm_serial_driver (串口驱动)
│
├── rm_auto_aim (自瞄算法)
│
├── rm_rune (打符算法)
│
├── rm_localization (*定位算法)
│
├── rm_perception (*感知算法)
│
├── rm_navigation (*导航算法)
│
├── rm_decision (*自主决策算法)
│
├── rm_utils (工具包) 
│   ├── math (包括PnP解算、弹道补偿等)
│   │
│   └── logger (日志库)
│
└── rm_upstart (自启动配置)
```

## 二、环境

如果你不需要完整功能，可以直接把相关的功能包删除掉

### 1. 基础
- Ubuntu 22.04
- ROS2 Humble
- 大恒相机驱动

### 2. 自瞄 
- fmt库
  ```bash
  sudo apt install libfmt-dev
  ```
- Sophus库 (G2O库依赖)
   ```bash
   git clone https://github.com/strasdat/Sophus
   cd Sophus
   mkdir build && cd build
   cmake ..
   make -j
   sudo make install
   ```
- G2O库 (优化装甲板Yaw角度)
    ```bash
    sudo apt install libeigen3-dev libspdlog-dev libsuitesparse-dev qtdeclarative5-dev qt5-qmake libqglviewer-dev-qt5
    git clone https://github.com/RainerKuemmerle/g2o
    cd g2o
    mkdir build && cd build
    cmake ..
    make -j
    sudo make install
    ```
### 3. 能量机关
- OpenVINO库 (能量机关识别)
  
   参考[OpenVINO官方文档](https://docs.openvino.ai/2022.3/openvino_docs_install_guides_installing_openvino_from_archive_linux.html)，建议同时安装GPU相关依赖

- Ceres库 (能量机关曲线拟合)
    ```bash
    sudo apt install libceres-dev
    ```

### 4. 导航
- Livox SDK2

  参考[Livox官方仓库](https://github.com/Livox-SDK/Livox-SDK2)

- Navigation2 (导航)
    ```bash
    sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
    ```

- PCL库 (导航时对点云的处理)
    ```
    sudo apt install libpcl-dev
    ```

### 5. 其他

本文档中可能有缺漏，如有，可以用`rosdep`安装剩下依赖

```bash
rosdep install --from-paths src --ignore-src -r -y

```

## 三、编译与运行

修改rm_bringup/config/launch_params.yaml，选择需要启动的功能

```bash
# 编译
colcon build --symlink-install --parallel-workers 4 #本仓库包含的功能包过多，建议限制同时编译的线程数
# 运行
source install/setup.bash
ros2 launch rm_bringup bringup.launch.py
```

默认日志和内录视频路径为`~/fyt2024-log/`

> 我们的日志库是用fmt搓的，不使用ros2的日志库


## 四、自启动

- 编译程序后，进入rm_upstart文件夹

```bash
cd rm_upstart
```

- 修改**rm_watch_dog.sh**中的`NAMESPACE`（ros命名空间）、`NODE_NAMES`（需要看门狗监控的节点）和`WORKING_DIR` （代码路径）

- 注册服务
  
```bash
sudo chmod +x ./register_service.sh
sudo ./register_service.sh

# 正常时有如下输出
# Creating systemd service file at /etc/systemd/system/rm.service...
# Reloading systemd daemon...
# Enabling service rm.service...
# Starting service rm.service...
# Service rm.service has been registered and started.
```

- 查看程序状态

```bash
systemctl status rm
```

- 查看终端输出
```
查看screen.output或~/fyt2024-log下的日志
```  

- 关闭程序

```bash
systemctl stop rm
```

- 取消自启动

```bash
systemctl disable rm
```

## 维护者及开源许可证

> 赛季结束开源

Maintainer : FYT Vision Group

```
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
```

## 致谢

感谢这个赛季视觉组的每一个成员的付出，感谢以下开源项目：

- [rm_vision](https://gitlab.com/rm_vision) rv是本项目的基础，提供了一套可参考的，规范、易用、高效的视觉算法框架
- [rmoss](https://github.com/robomaster-oss/rmoss_core) rmoss项目为RoboMaster提供通用基础功能模块包，本项目的串口驱动模块基于rmoss_base进行开发
- [沈阳航空航天大学TUP战队2022赛季步兵视觉开源](https://github.com/tup-robomaster/TUP-InfantryVision-2022) 为本项目的能量机关识别与预测算法提供了参考
- [沈阳航空航天大学YOLOX关键点检测模型](https://github.com/tup-robomaster/TUP-NN-Train-2) 提供了本项目能量机关识别模型训练代码
- [四川大学OpenVINO异步推理代码](https://github.com/Ericsii/rm_vision-OpenVINO) 提供了本项目能量机关识别模型部署的代码
- [上海交通大学自适应扩展卡尔曼滤波](https://github.com/julyfun/rm.cv.fans/tree/main) 使用Ceres自动微分功能，自动计算Jacobian矩阵


## 更新日志

- 参考上交开源，实现了EKF的自动求Jacobian矩阵
- 增加了粒子滤波器，为状态估计提供新的选择
- 修复了打符崩溃的问题（OpenVINO在推理时不能创建新的InferRequest，通过互斥锁解决）
- 将自瞄解算修改为定时器回调，固定解算的频率
- 增加手动补偿器ManualCompensator
- 重写PnP选解逻辑
- 修改了BA优化的代码，抽象出新的类ArmorPoseEstimator
