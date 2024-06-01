# rm_serial_driver

FYT视觉24赛季串口通信模块

## fyt::SerialDriverNode

串口驱动节点

### 发布话题

*  `serial/receive` (`rm_interfaces/msg/SerialReceiveData`) - 下位机发送到上位机的数据
*  `tf` (`geometry_msgs/msg/TransformStamped`) - 云台的tf变换
  
### Subscribed Topics

* `cmd_gimbal` (`rm_interfaces/msg/GimbalCmd`) - 云台控制信息
* `cmd_chassis` (`rm_interfaces/msg/ChassisCmd`) - 底盘控制信息

### 参数

* `target_frame` (string, default: "odom") - 下位机欧拉角的相对坐标系
* `timestamp_offset` (double, default: 0.0) - tf数据的时间戳补偿
* `port_name` (string, default: "/dev/ttyUART") - 串口设备对应的文件名
* `protocol` (string, default: "infantry") - 协议类型
* `enable_data_print` (bool, default: false) - 是否打印串口读出的原始数据

## fyt::VirtualSerial

仿真串口驱动节点

### 发布话题

*  `serial/receive` (`rm_interfaces/msg/SerialReceiveData`) - 下位机发送到上位机的数据（固定数据）
*  `tf` (`geometry_msgs/msg/TransformStamped`) - 云台的tf变换（固定数据）
  
### 参数

* `pitch` (double, default: 0.0) - 固定的pitch角度 
* `yaw` (double, default: 0.0) - 固定的yaw角度 
* `vision_mode` (int, default: 0) - 视觉模式 
