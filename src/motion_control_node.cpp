#include "motion_control_node.hpp" // 包含你的类定义

// 定义并初始化静态成员变量
std::mutex MotionControlNode::m_serialMutex;

MotionControlNode::MotionControlNode() : Node("motion_control"), m_br(this) {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

    m_velSub = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", qos, std::bind(&MotionControlNode::vel_callback, this, _1));
    m_odomPublisher = this->create_publisher<nav_msgs::msg::Odometry>("odom", qos);

    if (!initialize()) {
        return;
    }

    m_loopThread = std::thread(std::bind(&MotionControlNode::loop_serial_communication, this));
    RCLCPP_INFO(this->get_logger(), "Please post /cmd_vel topic to control the movement.");
    RCLCPP_INFO(this->get_logger(), "Waiting for the /cmd_vel topic .......");
}

MotionControlNode::~MotionControlNode() {
    shutdown();
}

bool MotionControlNode::initialize() {
    m_setupOk = set_serial();
    if (!m_setupOk) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set up serial communication.");
        return false;
    }
    RCLCPP_INFO(this->get_logger(),
                "\n\n\n*************************\n* Application Restarted *\n*************************");
    return true;
}

void MotionControlNode::shutdown() {
    m_shouldExit.store(true);
    if (m_loopThread.joinable()) {
        m_loopThread.join();
    }
    close_serial_port(); // 当节点关闭时执行清理操作
}

// 打开串口
bool MotionControlNode::open_serial_port(const char* portName, int& portHandle) {
    portHandle = open(portName, O_RDWR);
    if (portHandle == -1) {
        RCLCPP_ERROR(rclcpp::get_logger("motion_control"), "无法打开串口设备 %s", portName);
        return false;
    }
    return true;
}

// 设置串口参数
bool MotionControlNode::set_serial() {
    if (!open_serial_port("/dev/ttyS4", m_ttyS4))
        return false;
    if (!open_serial_port("/dev/ttyS9", m_ttyS9))
        return false;

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(m_ttyS4, &tty) != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("motion_control"), "无法获取串口 m_ttyS4 属性");
        return false;
    }
    if (tcgetattr(m_ttyS9, &tty) != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("motion_control"), "无法获取串口 m_ttyS9 属性");
        return false;
    }

    cfsetospeed(&tty, B115200); // 设置波特率为115200
    cfsetispeed(&tty, B115200);
    tty.c_cflag &= ~CSIZE; // 设置CSIZE为CS8，即8位数据长度
    tty.c_cflag |= CS8;
    tty.c_cflag |= (CLOCAL | CREAD); // 设置CLOCAL和CREAD，使能接收器和本地模式
    tty.c_cflag &= ~PARENB; // 设置PARENB为0，即无校验位
    tty.c_cflag &= ~CSTOPB; // 设置CSTOPB为0，即1位停止位
    tty.c_cflag &= ~CRTSCTS; // 设置CRTSCTS为0，即不使用硬件流控制
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // 设置ICANON为0，即非规范模式，这样read就不会受行缓冲的影响
    tty.c_oflag &= ~OPOST; // 设置OPOST为0，即禁用输出处理
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // 设置ICANON为0，即非规范模式
    tty.c_cc[VMIN] = 0; // 设置VMIN为0，VMAX为0，非阻塞
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(m_ttyS4, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("motion_control"), "无法设置串口 m_ttyS4 属性");
        return false;
    }
    if (tcsetattr(m_ttyS9, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("motion_control"), "无法设置串口 m_ttyS9 属性");
        return false;
    }

    return true;
}

// 发送指令函数
bool MotionControlNode::send_command(int serial_fd, unsigned char *command, size_t command_len)
{
    ssize_t bytes_written = write(serial_fd, command, command_len);
    if (bytes_written != static_cast<ssize_t>(command_len)) {
        RCLCPP_ERROR(rclcpp::get_logger("motion_control"), "发送指令失败");
        return false;
    }
    return true;
}

// 接收反馈函数
bool MotionControlNode::receive_feedback(int serial_fd, unsigned char *feedback, size_t feedback_len)
{
    ssize_t bytes_read = read(serial_fd, feedback, feedback_len);
    if (bytes_read != static_cast<ssize_t>(feedback_len)) {
        RCLCPP_ERROR(rclcpp::get_logger("motion_control"), "接收反馈失败，实际读取了 %zd 字节", bytes_read);
        return false;
    }
    return true;
}

// 发送指令并接收反馈函数
bool MotionControlNode::send_and_receive_feedback(int serial_fd, unsigned char* speed_command, int speed_command_size)
{
    bool ret;
    unsigned char feedback[10];
    m_serialMutex.lock();
    if (send_command(serial_fd, speed_command, speed_command_size) && receive_feedback(serial_fd, feedback, sizeof(feedback))) {
        // 解析反馈数据
        unsigned char mode_value = feedback[1];
        short torque_current = (feedback[2] << 8) | feedback[3];
        short speed = (feedback[4] << 8) | feedback[5];
        short position = (feedback[6] << 8) | feedback[7];
        unsigned char fault_code = feedback[8];

        RCLCPP_INFO(rclcpp::get_logger("motion_control"), "serial_fd%d 模式值：%d", serial_fd, static_cast<int>(mode_value));
        RCLCPP_INFO(rclcpp::get_logger("motion_control"), "serial_fd%d 转矩电流：%d", serial_fd, torque_current);
        RCLCPP_INFO(rclcpp::get_logger("motion_control"), "serial_fd%d 速度：%d", serial_fd, speed);
        RCLCPP_INFO(rclcpp::get_logger("motion_control"), "serial_fd%d 位置：%d", serial_fd, position);
        RCLCPP_INFO(rclcpp::get_logger("motion_control"), "serial_fd%d 故障码：%d", serial_fd, static_cast<int>(fault_code));

        ret = true;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("motion_control"), "serial_fd%d 发送指令或接收反馈失败", serial_fd);
        ret = false;
    }
    m_serialMutex.unlock();
    return ret;
}

// 关闭串口
void MotionControlNode::close_serial_port() {
    unsigned char stop_run[] = {0x01, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50};
    if (m_ttyS4 != -1) {
        send_command(m_ttyS4, stop_run, sizeof(stop_run));
        close(m_ttyS4);
        RCLCPP_INFO(rclcpp::get_logger("motion_control"), "m_ttyS4串口已关闭");
    }
    if (m_ttyS9 != -1) {
        send_command(m_ttyS9, stop_run, sizeof(stop_run));
        close(m_ttyS9);
        RCLCPP_INFO(rclcpp::get_logger("motion_control"), "m_ttyS9串口已关闭");
    }
}

// 超时停止运动
void MotionControlNode::timeout_stop_run()
{
    auto current_time = this->now();
    auto elapsed_time = (current_time - m_lastReceiveTime).seconds();

    if (elapsed_time >= m_timeoutSeconds) {
        unsigned char stop_run[] = {0x01, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50};
        send_command(m_ttyS4, stop_run, sizeof(stop_run));
        send_command(m_ttyS9, stop_run, sizeof(stop_run));
        RCLCPP_WARN(this->get_logger(), "Time out to stop motion!!!");
    }
}

// 限制速度范围的函数
short MotionControlNode::limit_speed(double wheel_speed)
{
    if (wheel_speed < -m_maxWheelSpeed) {
        return -m_maxRpm;
    } else if (wheel_speed > -m_maxWheelSpeed && wheel_speed < m_maxWheelSpeed) {
        return static_cast<short>(std::round(m_reductionRatio * wheel_speed / (2 * M_PI * m_wheelRadius)));
    } else {
        return m_maxRpm;
    }
}

// 运动学逆解运算
void MotionControlNode::kinematic_inverse(double linear_speed, double angular_speed, short &out_left_wheel_rpm, short &out_right_wheel_rpm)
{
    double out_left_wheel_speed, out_right_wheel_speed;
    out_left_wheel_speed = linear_speed - (angular_speed * m_wheelbase) / 2.0;
    out_right_wheel_speed = -(linear_speed + (angular_speed * m_wheelbase) / 2.0); // 右轮转速与左轮相反

    // 限制速度范围
    out_left_wheel_rpm = limit_speed(out_left_wheel_speed);
    out_right_wheel_rpm = limit_speed(out_right_wheel_speed);
}

// 计算CRC-8/MAXIM校验码
unsigned char MotionControlNode::calculate_crc(const unsigned char *data, size_t length) {
    unsigned char crc = 0x00; // Initial value

    for (size_t i = 0; i < length; ++i) {
        crc = crc8Table[crc ^ data[i]];
    }

    return crc;
}

// 解析速度消息，生成运动控制指令
void MotionControlNode::speed_analysis(const geometry_msgs::msg::Twist::SharedPtr msg, unsigned char *left_wheel_cmd, unsigned char *right_wheel_cmd)
{
    // 计算左右轮的转速
    short left_rpm, right_rpm;
    kinematic_inverse(msg->linear.x, msg->angular.z, left_rpm, right_rpm);
    RCLCPP_INFO(rclcpp::get_logger("motion_control"), "The calculated rotational speeds ---  left_rpm: %d  right_rpm: %d", left_rpm, right_rpm);

    // 处理左右轮的转速信息，生成速度控制指令
    for (int side = 0; side < 2; ++side) {
        unsigned char *wheel_cmd_ptr;
        unsigned char *data_ptr;
        unsigned char wheel_id;

        wheel_id = 0x01;
        if (side == 0) { // 左轮
            wheel_cmd_ptr = left_wheel_cmd;
        } else { // 右轮
            wheel_cmd_ptr = right_wheel_cmd;
        }

        // 设置wheel ID
        *wheel_cmd_ptr = wheel_id;

        // 设置数据字节
        data_ptr = wheel_cmd_ptr + 1;
        *data_ptr++ = 0x64; // 固定值

        // 设置高八位和低八位
        *data_ptr++ = (side == 0 ? left_rpm : right_rpm) >> 8;
        *data_ptr++ = (side == 0 ? left_rpm : right_rpm) & 0xFF;

        // 设置剩余的数据
        for (int i = 4; i < 9; ++i) {
            if (i == 6) // 设置加速快慢
                *data_ptr++ = 0x05;
            else
                *data_ptr++ = 0x00;
        }

        // 计算 CRC 校验码
        size_t length = 9;
        unsigned char crc8 = calculate_crc(wheel_cmd_ptr, length);

        // 设置 CRC 校验码
        *data_ptr = crc8;
    }
}

void MotionControlNode::vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    RCLCPP_INFO(rclcpp::get_logger("motion_control"), "Received linear velocity: x = %f, angular velocity: z = %f", msg->linear.x, msg->angular.z);

    unsigned char left_speed_command[10];
    unsigned char right_speed_command[10];
    speed_analysis(msg, left_speed_command, right_speed_command);

    // 更新上次接收到速度指令的时间点为当前时间
    m_lastReceiveTime = this->now();

    // 发送速度指令并接收反馈
    if (!send_and_receive_feedback(m_ttyS4, left_speed_command, sizeof(left_speed_command))) {
        RCLCPP_ERROR(rclcpp::get_logger("motion_control"), "处理 m_ttyS4 失败");
    }

    if (!send_and_receive_feedback(m_ttyS9, right_speed_command, sizeof(right_speed_command))) {
        RCLCPP_ERROR(rclcpp::get_logger("motion_control"), "处理 m_ttyS9 失败");
    }
}

bool MotionControlNode::clear_buffer(int serial_fd)
{
    unsigned char buffer[1024];
    while (true) {
        ssize_t bytes_read = read(serial_fd, buffer, sizeof(buffer));
        if (bytes_read <= 0) {
            break;  // 没有更多数据或错误发生
        }
    }
    return true;
}

bool MotionControlNode::get_wheel_speed(double &left_wheel_velocity, double &right_wheel_velocity)
{
    unsigned char search_speed[10] = {0x01, 0x74, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04}; // 查询当前轮子反馈

    short left_rpm_cur = 0, right_rpm_cur = 0;
    int ttyS[] = {m_ttyS4, m_ttyS9}; // 使用成员变量

    std::lock_guard<std::mutex> lock(m_serialMutex);

    for (int i = 0; i < sizeof(ttyS) / sizeof(ttyS[0]); ++i) {
        unsigned char feedback[10];

        // 清空接收缓冲区
        if (!clear_buffer(ttyS[i])) {
            RCLCPP_ERROR(this->get_logger(), "%s 清空接收缓冲区失败", (i == 0 ? "left_wheel" : "right_wheel"));
            return false;
        }

        // 发送命令
        if (!send_command(ttyS[i], search_speed, sizeof(search_speed))) {
            RCLCPP_ERROR(this->get_logger(), "%s 发送指令失败", (i == 0 ? "left_wheel" : "right_wheel"));
            return false;
        }

        // 延时一段时间
        std::this_thread::sleep_for(std::chrono::milliseconds(5));  // 使用标准库的延时

        // 接收反馈
        if (!receive_feedback(ttyS[i], feedback, sizeof(feedback))) {
            RCLCPP_ERROR(this->get_logger(), "%s 接收反馈失败", (i == 0 ? "left_wheel" : "right_wheel"));
            return false;
        }

        // 解析反馈数据
        unsigned char wheel_id = feedback[0];
        unsigned char mode_value = feedback[1];
        short rpm = (feedback[4] << 8) | feedback[5];
        i == 0 ? left_rpm_cur = rpm : right_rpm_cur = rpm;

        // 检查反馈数据是否正常
        if (wheel_id != 0x01 || mode_value != 0x74 || (i == 0 && std::abs(left_rpm_cur - m_leftRpm) > 100) || (i == 1 && std::abs(right_rpm_cur - m_rightRpm) > 100)) {
            RCLCPP_ERROR(this->get_logger(), "%s 反馈数据异常", (i == 0 ? "left_wheel" : "right_wheel"));
            return false;
        }
    }

    // 当前转速为有效转速，更新数值
    m_leftRpm = left_rpm_cur;
    m_rightRpm = right_rpm_cur;

    // 计算左右轮速度，m/s
    if (std::abs(m_rightRpm + m_leftRpm) <= 3) // 左右轮转速差距<=3，认为两轮速度相等，减小里程计误差
    {
        m_leftRpm = -m_rightRpm;
    }
    left_wheel_velocity = (2 * M_PI * m_wheelRadius * m_leftRpm) / m_reductionRatio;
    right_wheel_velocity = -((2 * M_PI * m_wheelRadius * m_rightRpm) / m_reductionRatio); // 右轮转速与左轮相反

    return true;
}

void MotionControlNode::broadcast_odom_base_link_transform(const nav_msgs::msg::Odometry& odom) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header = odom.header;
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = odom.pose.pose.position.x;
    transform.transform.translation.y = odom.pose.pose.position.y;
    transform.transform.translation.z = odom.pose.pose.position.z;
    transform.transform.rotation = odom.pose.pose.orientation;
    m_br.sendTransform(transform);
}

// 计算里程计信息并发布
nav_msgs::msg::Odometry MotionControlNode::calculate_odometry() {
    // 获取当前时间和计算时间差
    auto current_time = this->now();
    auto dt = (current_time - m_prevTime).seconds();
    m_prevTime = current_time;

    // 初始化速度变量
    double left_wheel_velocity, right_wheel_velocity;
    bool get_new_speed = get_wheel_speed(left_wheel_velocity, right_wheel_velocity);

    if (!get_new_speed) {
        RCLCPP_WARN(this->get_logger(), "------无效-----速度------");
        // 如果没有获得新的速度，则返回默认的里程计消息
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";
        odom_msg.header.stamp = current_time;
        return odom_msg;
    } else {
        RCLCPP_INFO(this->get_logger(), "------有效-----速度------");
    }

    // 计算线速度和角速度
    double v = (left_wheel_velocity + right_wheel_velocity) / 2.0;
    double omega = (right_wheel_velocity - left_wheel_velocity) / m_wheelbase;

    // 积分更新位置和姿态
    m_theta += omega * dt;
    m_theta = std::atan2(sin(m_theta), cos(m_theta));  // 规范化角度到[-π, π]
    m_x += v * cos(m_theta) * dt;
    m_y += v * sin(m_theta) * dt;

    // 填充里程计信息
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    odom_msg.header.stamp = current_time;
    odom_msg.pose.pose.position.x = m_x;
    odom_msg.pose.pose.position.y = m_y;
    tf2::Quaternion q;
    q.setRPY(0, 0, m_theta);
    odom_msg.pose.pose.orientation = tf2::toMsg(q);

    return odom_msg;
}

void MotionControlNode::loop_serial_communication()
{
    while (rclcpp::ok()) {
        // 计算里程计信息
        nav_msgs::msg::Odometry odom_msg = calculate_odometry();
        odom_publisher->publish(odom_msg); // 发布里程计信息
        broadcast_odom_base_link_transform(odom_msg);
        timeout_stop_run();
    }
}
// Implementations of other member functions...
