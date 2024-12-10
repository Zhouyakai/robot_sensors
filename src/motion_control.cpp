#include "motion_control_node.hpp" // 包含你的类定义

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotionControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
