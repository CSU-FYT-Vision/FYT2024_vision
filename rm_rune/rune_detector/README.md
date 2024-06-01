# rune_detector

FYT视觉24赛季能量机关识别ROS2包。从图像中识别出能量机关的5个关键点。

OpenVINO推理部分参考了[rm_vision-OpenVINO](https://github.com/Ericsii/rm_vision-OpenVINO)

## fyt::rune::RuneDetectorNode

能量机关识别器节点，订阅图像话题，调用OpenVINO进行网络推理，定位出能量机关的五个关键点

### 发布话题 

* `rune_target` (`rm_interfaces/msg/RuneTarget`) - 识别到的待击打能量机关五个关键点
*  `bebug_img` (`sensor_msgs/msg/Image`) - debug图像，绘制出识别到的目标和识别R标的二值化ROI

### 订阅话题 

* `image_raw` (`sensor_msgs/msg/Image`) - 工业相机采集的图像

### 参数 

* `detector.model` (string, default: "yolox_rune.onnx") - 训练好的网络权重文件.
* `detector.device_type` (string, default: CPU) - 推理网络的设备，可选，CPU/GPU/AUTO 之一.
* `debug` (bool, default: true) - 是否开启debug模式.
* `requests_limit` (int, default: 5) - 推理请求队列的最大长度，大于0时意味着异步推理，会消耗更多的处理器资源换取推理速度.
* `detect_r_tag` (bool, default: true) - 是否使用传统方法识别R标，相比网络预测，传统方法识别R标会更稳定.
