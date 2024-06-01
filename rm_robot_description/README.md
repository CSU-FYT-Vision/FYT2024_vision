# rm_robot_description
RoboMaster 视觉自瞄系统所需的 URDF

<img src="docs/rm_vision.svg" alt="rm_vision" width="200" height="200">

该项目为 [rm_vision](https://github.com/chenjunnn/rm_vision) 的子模块

## 坐标系定义

单位和方向请参考 https://www.ros.org/reps/rep-0103.html

odom: 以云台中心为原点的惯性系

yaw_joint: 表述云台的 yaw 轴与惯性系的旋转关系

pitch_joint: 表述云台的 pitch 轴与惯性系的旋转关系

camera_joint: 表述相机到惯性系的变换关系

camera_optical_joint: 表述以 z 轴为前方的相机坐标系转换为 x 轴为前方的相机坐标系的旋转关系

## 使用方法

修改 [urdf/rm_gimbal.urdf.xacro](urdf/rm_gimbal.urdf.xacro) 中的 `gimbal_camera_transfrom` 

xyz 与 rpy 对应机器人云台上相机到云台中心的平移与旋转关系，可以由机械图纸测量得到，或在机器人上直接测量
