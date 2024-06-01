# rm_utils 

## 1. 介绍
该模块提供通用的工具类以及函数，包括：
* 扩展卡尔曼滤波器 ExtendedKalmanFilter
* PnP解算器 PnPSolver
* 弹道补偿器 TrajectoryCompensator
* FYT日志库
* URL路径解析器 URLResolver
* 心跳发布者 HeartBeatPublisher

## 2. 使用方法

### 2.0 扩展卡尔曼滤波

见armor_solver/src/armor_solver_node.cpp

### 2.1 PnP解算

示例：
```cpp
#include "rm_utils/math/pnp_solver.hpp"

// 1. 创建PnP解算器
// 构造函数参数(cv::Mat, cv::Mat, cv::PnPMethod)：相机内参矩阵、畸变系数、解算方法（默认是IPPE）
auto pnp_solver = PnPSolver(camera_matrix, distortion_coefficients);
// 2. 为PnPSolver设置一个物体坐标系
std::vector<cv::Point3f> object_points = {
    cv::Point3f(-1.0f, 1.0f, 0.0f),
    cv::Point3f(1.0f, 1.0f, 0.0f),
    cv::Point3f(1.0f, -1.0f, 0.0f),
    cv::Point3f(-1.0f, -1.0f, 0.0f)
};
pnp_solver.setObjectPoints(object_points, "object_frame");
// 3. PnP解算
cv::Mat rvec, tvec;
pnp_solver.solvePnP(image_points, rvec, tvec, "object_frame");

// 对于打符来说，因为以前的PnP是直接返回一个表示位姿的Eigen::VectorXd，所以这里也提供了一个函数，用于将rvec和tvec转换为VectorXd
// 4. 获取位姿
Eigen::VectorXd pose = pnp_solver.getPose(rvec, tvec);
```

### 2.2 弹道补偿

示例：
```cpp
#include "rm_utils/math/trajectory_compensator.hpp"

auto compensator = CompensatorFactory::createCompensator("ideal");
double pitch = 0;
Eigen::Vector3d p = Eigen::Vector3d(1.0, 0, 0);
double temp_pitch = pitch;
if (trajectory_compensator_->compensate(p, temp_pitch)) {
    pitch = temp_pitch;
}
```

### 2.3 Eigen和cv::Mat的相互转换

示例：
```cpp
#include "rm_utils/math/utils.hpp"

cv::Mat mat = cv::Mat::eye(3, 3, CV_32FC1);
Eigen::Matrix3f eigen_mat = utils::cvToEigen(mat);
cv::Mat mat2 = utils::eigenToCv(eigen_mat);
```

### 2.4 FYT日志库

示例：
```cpp
#include "rm_utils/logger/log.hpp"

// 1. 初始化
// 参数：日志名称、日志文件路径、日志级别
FYT_REGISTER_LOGGER("test_logger", "/tmp/test_logger.log", INFO);

// 2. 使用
FYT_INFO("test_logger", "This is a test log");
int a = 1;
FYT_WARN("test_logger", "a = {}", a);
```

### 2.5 URL Resolver

能够用很方便的方式（类camera_info_url）访问文件和或ros2包的install目录

示例:
```cpp
#include "rm_utils/url_resolver.hpp"

namespace fs = std::filesystem;
fs::path model_path =
  utils::URLResolver::getResolvedPath("package://armor_detector/model/lenet.onnx");

if (fs::exists(model_path)) {
    std::cout<<model_path.string()<<std::endl;
}

fs::path file_path = utils::URLResolver::getResolvedPath("file://home/zcf/123.txt");
```

### 2.6 HearBeatPublisher

定时发布心跳数据

示例：
```cpp
#include "rm_utils/heartbeat.hpp"

class TestNode : public rclcp::Node {
public:
  TestNode(const rclcpp::NodeOptions& options) : Node("test", opitions) {
    heartbeat_ = HeartBeatPublisher::create(this); // 传入Node*
  }
private:
  HeartBeatPublisher::SharedPtr heartbeat_;
};
```