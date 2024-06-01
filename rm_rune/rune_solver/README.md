# rune_solver

FYT视觉24赛季能量机关解算和预测ROS2包。

## fyt::rune::RuneSolverNode

1. 计算能量机关的三维坐标
2. 对坐标进行卡尔曼滤波消除抖动
3. 拟合角度变化曲线，预测$\Delta t$后的坐标
4. 解算出云台的角度发送给电控

### 发布话题 

*  `cmd_gimbal` (`rm_interfaces/msg/GimbalCmd`) - 最终发送到下位机的角度信息
*  `predict_target` (`geometry_msgs/msg/PointStamped`) - 预测的靶心坐标
*  `observed_angle` (`rm_interfaces/msg/DebugRuneAngle`) - 当前识别到的能量机关角度
*  `predict_angle` (`rm_interfaces/msg/DebugRuneAngle`) - 预测的能量机关角度
*  `markers` (`visualization_markers/msg/MarkerArray`) - 用于Debug的可视化Marker，包括当前识别到的靶心坐标可视化、R标坐标可视化和预测的靶心坐标可视化

### 订阅话题 

* `rune_target` (`rm_interfaces/msg/RuneTarget`) - 识别到的待击打能量机关五个关键点

### 参数 

* `debug` (bool, default: true) - 是否开启debug模式.
* `predict_time` (double, default: "0.0") - 预测时间补偿，最终预测时间为$\Delta t = t(子弹飞行) + t(传输延迟) + predict\_time$.
* `compensator_type` (string, default: resistance) - 弹道补偿模型
* `auto_type_determined` (bool, default: true) - 设为true后会自动判断大符还是小符，设为false则用串口设定的模式判断
* `ekf.q` (double[]) - 卡尔曼滤波的状态转移噪声
* `ekf.r` (double[]) - 卡尔曼滤波的观测噪声
  