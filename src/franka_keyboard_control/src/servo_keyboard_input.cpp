#include "franka_keyboard_control/servo_keyboard_input.hpp"

// 键盘键位定义
#define KEYCODE_LEFT 0x44
#define KEYCODE_RIGHT 0x43
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_Q 0x71
#define KEYCODE_W 0x77
#define KEYCODE_E 0x65
#define KEYCODE_R 0x72
#define KEYCODE_F 0x66
#define KEYCODE_Z 0x7A
#define KEYCODE_X 0x78
#define KEYCODE_C 0x63
#define KEYCODE_V 0x76
#define KEYCODE_B 0x62
#define KEYCODE_T 0x74
#define KEYCODE_Y 0x79
#define KEYCODE_SPACE 0x20

namespace ext_serial_driver
{
    KeyboardServo::KeyboardServo(const rclcpp::NodeOptions &options)
        : Node("keyboard_servo_node", options), port_{new Port(2)}, vel_cmd_(0.2)
    {
        // print_instructions();
        RCLCPP_INFO(get_logger(), "Start KeyboardServo!");
        port_->getParams("/dev/ttyACM0", 115200, "none", "none", "1");

        twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, ROS_QUEUE_SIZE);
        action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this,
            "/manipulator_controller/follow_joint_trajectory");

        try
        {
            port_->serial_driver_->init_port(port_->device_name_, *port_->device_config_);
            if (!port_->serial_driver_->port()->is_open())
            {
                port_->serial_driver_->port()->open();
                port_->receive_thread_ = std::thread(&KeyboardServo::receiveData, this);
                // LOG
                RCLCPP_INFO(get_logger(), "serial open OK!");
            }
        }
        catch (const std::exception &ex)
        {
            RCLCPP_ERROR(
                get_logger(), "Error creating serial port: %s - %s", port_->device_name_.c_str(), ex.what());
            throw ex;
        }

        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            JOINT_STATE_TOPIC, ROS_QUEUE_SIZE,
            std::bind(&KeyboardServo::sendData, this, std::placeholders::_1));

        init_b_grade_rotation();
        publish_b_grade_tf();

        // Heartbeat
        heartbeat_ = HeartBeatPublisher::create(this);
    }

    void KeyboardServo::print_instructions()
    {
        puts("\n=== 机械臂键盘控制 ===");
        puts("模式切换:");
        puts("  左方向键: 平移模式");
        puts("  右方向键: 旋转模式");
        puts("平移模式控制 (WASDQE):");
        puts("  W/S - 前后移动");
        puts("  A/D - 左右移动");
        puts("  Q/E - 升降移动");
        puts("旋转模式控制 (WASDQE):");
        puts("  W/S - 俯仰角(Pitch)");
        puts("  A/D - 偏航角(Yaw)");
        puts("  Q/E - 翻滚角(Roll)");
        puts("其他功能:");
        puts("  R - 世界坐标系 (base_link)");
        puts("  F - 图像坐标系 (link3)");
        puts("  Z - 开车位置（并退出A/B级矿模式）");
        puts("  B - 锁死当前关节角度");
        puts("  V - 解锁当前关节角度");
        puts("  T - 进入A级矿模式");
        puts("  Y - 进入B级矿模式");
        puts("  SPACE - 退出程序");
        puts("\nA级矿模式下:");
        puts("  X - 左下方向速度 (与水平面夹角30度)");
        puts("  C - A级矿兑换位置");
        puts("\nB级矿模式下:");
        puts("  X - 位置1");
        puts("  C - 启用 B 级矿坐标系 (先绕基座 Y +50°，再绕基座 X +45°)");
        puts("  按 R 或 F 可禁用 B 级矿坐标系，恢复正常控制");
        puts("\n非矿模式下 X/C 无效");
        puts("\n状态指示:");
        puts("  关节锁死状态: [按B键锁死，按V键解锁]");
        puts("  当前矿模式: 无 / A级 / B级");
        puts("  B级矿坐标系启用状态: 仅 B 级矿模式下可启用, 按C键切换, R/F/切换模式自动禁用");
    }

    void KeyboardServo::receiveData()
    {
        std::vector<uint8_t> header(1); // unsigned int (16)
        std::vector<uint8_t> data;
        data.reserve(sizeof(UpReceivePacket));

        RCLCPP_INFO(get_logger(), "[Receive] receiveData start!");

        while (rclcpp::ok())
        {
            try
            {
                // RCLCPP_INFO(get_logger(), "[Receive] receive_header %u!", header[0]);
                port_->serial_driver_->port()->receive(header);
                if (header[0] == 0xA8)
                {
                    data.resize(sizeof(UpReceivePacket) - 1);
                    port_->serial_driver_->port()->receive(data);
                    data.insert(data.begin(), header[0]);
                    UpReceivePacket packet = fromVector<UpReceivePacket>(data);

                    bool crc_ok =
                        crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
                    if (crc_ok)
                    {
                        // RCLCPP_INFO(get_logger(), "CRC OK!");
                        uint8_t c = packet.key_code;
                        RCLCPP_INFO(get_logger(), "[Receive] keycode %u!", c);
                        process_key(c);
                    }
                    else
                    {
                        RCLCPP_ERROR(get_logger(), "CRC error!");
                    }
                }
                else
                {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "Invalid header: %02X", header[0]);
                }
            }
            catch (const std::exception &ex)
            {
                RCLCPP_ERROR_THROTTLE(
                    get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
                port_->reopenPort();
            }

            // std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    void KeyboardServo::send_home_goal(const std::vector<double> &positions)
    {
        if (!action_client_->wait_for_action_server(std::chrono::seconds(2)))
        {
            RCLCPP_ERROR(get_logger(), "动作服务器不可用");
            return;
        }

        auto goal_msg = FollowJointTrajectory::Goal();
        goal_msg.trajectory.joint_names = {
            "joint0", "joint1", "joint2", "joint3",
            "joint4", "joint5", "joint6"};

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = positions;
        point.time_from_start.sec = 5;
        goal_msg.trajectory.points.push_back(point);

        auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            [this](const GoalHandle::SharedPtr &goal_handle)
        {
            if (!goal_handle)
            {
                RCLCPP_ERROR(get_logger(), "目标被拒绝");
            }
            else
            {
                RCLCPP_INFO(get_logger(), "目标已接受");
            }
        };

        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void KeyboardServo::lockCurrentJoints()
    {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        if (!has_joint_state_)
        {
            RCLCPP_WARN(get_logger(), "尚未收到关节状态，无法锁死");
            return;
        }

        RCLCPP_INFO(get_logger(), "锁死当前关节角度");
        if (!action_client_->wait_for_action_server(std::chrono::seconds(2)))
        {
            RCLCPP_ERROR(get_logger(), "动作服务器不可用");
            return;
        }

        auto goal_msg = FollowJointTrajectory::Goal();
        goal_msg.trajectory.joint_names = {
            "joint0", "joint1", "joint2", "joint3",
            "joint4", "joint5", "joint6"};

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = current_joint_positions_;
        point.time_from_start.sec = 1;
        goal_msg.trajectory.points.push_back(point);

        auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            [this](const GoalHandle::SharedPtr &goal_handle)
        {
            if (!goal_handle)
            {
                RCLCPP_ERROR(get_logger(), "锁死目标被拒绝");
            }
            else
            {
                RCLCPP_INFO(get_logger(), "关节已锁死");
            }
        };

        action_client_->async_send_goal(goal_msg, send_goal_options);
        is_joint_locked_ = true;
    }

    void KeyboardServo::unlockJoints()
    {
        is_joint_locked_ = false;
        RCLCPP_INFO(get_logger(), "关节已解锁");
    }

    void KeyboardServo::init_b_grade_rotation()
    {
        // 1. 绕固定基座 Y 轴旋转 +50°
        tf2::Quaternion q_y;
        q_y.setRPY(0, 50.0 * M_PI / 180.0, 0);

        // 2. 绕固定基座 X 轴（旋转前的 X 轴）旋转 +45°
        tf2::Quaternion q_x;
        q_x.setRPY(45.0 * M_PI / 180.0, 0, 0);

        // 3. 计算合成四元数：外旋左乘
        tf2::Quaternion q_base_to_b = q_x * q_y;
        q_base_to_b.normalize();

        // 存储旋转矩阵（b_grade_frame -> base_link）
        b_grade_rotation_ = q_base_to_b;
    }

    void KeyboardServo::publish_b_grade_tf()
    {
        static tf2_ros::StaticTransformBroadcaster broadcaster(this);
        geometry_msgs::msg::TransformStamped transform;

        transform.header.stamp = now();
        transform.header.frame_id = BASE_FRAME_ID;
        transform.child_frame_id = "b_grade_frame";

        // 重新计算 q_base_to_b 用于 TF 静态广播
        tf2::Quaternion q_y;
        q_y.setRPY(0, 50.0 * M_PI / 180.0, 0);
        tf2::Quaternion q_x;
        q_x.setRPY(45.0 * M_PI / 180.0, 0, 0);
        tf2::Quaternion q_base_to_b = q_x * q_y;
        q_base_to_b.normalize();

        transform.transform.rotation.x = q_base_to_b.x();
        transform.transform.rotation.y = q_base_to_b.y();
        transform.transform.rotation.z = q_base_to_b.z();
        transform.transform.rotation.w = q_base_to_b.w();

        transform.transform.translation.x = 0.0;
        transform.transform.translation.y = 0.0;
        transform.transform.translation.z = 0.0;

        broadcaster.sendTransform(transform);
        RCLCPP_INFO(get_logger(), "已发布 B 级矿坐标系静态变换 (外旋: 先绕基座 Y +50°, 再绕基座 X +45°)");
    }

    // ========== A 级矿模式 ==========
    void KeyboardServo::send_to_position_A_X()
    {
        if (is_joint_locked_)
        {
            RCLCPP_WARN(get_logger(), "关节已锁死，无法移动");
            return;
        }
        RCLCPP_INFO(get_logger(), "A级矿模式 - 左下方向速度 (与水平面夹角30度)");
        auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
        twist_msg->header.stamp = now();
        twist_msg->header.frame_id = BASE_FRAME_ID;
        double speed = vel_cmd_;
        double angle_rad = 30.0 * M_PI / 180.0;
        twist_msg->twist.linear.x = 0.0;
        twist_msg->twist.linear.y = speed * cos(angle_rad);
        twist_msg->twist.linear.z = -speed * sin(angle_rad);
        twist_pub_->publish(std::move(twist_msg));
    }

    void KeyboardServo::send_to_position_A_C()
    {
        if (is_joint_locked_)
        {
            RCLCPP_WARN(get_logger(), "关节已锁死，无法移动");
            return;
        }
        RCLCPP_INFO(get_logger(), "A级矿模式 - C位置");
        send_home_goal({0.01404, 0.00492, -0.10805, 0.41800, 0.00005, -1.26433, 0.64947});
    }

    // ========== B 级矿模式 ==========
    void KeyboardServo::send_to_position_B_X()
    {
        if (is_joint_locked_)
        {
            RCLCPP_WARN(get_logger(), "关节已锁死，无法移动");
            return;
        }
        RCLCPP_INFO(get_logger(), "B级矿模式 - X位置");
        send_home_goal({0.53600, 0.96518, -0.22727, 0.64461, 0.26468, 1.02392, -0.21919}); // 占位
    }

    void KeyboardServo::send_to_position_A_Base()
    {
        if (is_joint_locked_)
        {
            RCLCPP_WARN(get_logger(), "关节已锁死，无法进入A级矿模式");
            return;
        }
        RCLCPP_INFO(get_logger(), "进入A级矿模式，移动到预设A基础位置");
        send_home_goal({0.01453, 0.00125, -0.17905, 0.28518, 0.00125, 0.10357, 1.68631});
    }

    void KeyboardServo::send_to_position_B_Base()
    {
        if (is_joint_locked_)
        {
            RCLCPP_WARN(get_logger(), "关节已锁死，无法进入B级矿模式");
            return;
        }
        RCLCPP_INFO(get_logger(), "进入B级矿模式，移动到预设B基础位置");
        send_home_goal({0.01316, 0.01038, -0.28587, 0.82598, 0.00545, 0.53621, 0.63569});
    }

    void KeyboardServo::process_key(char c)
    {
        auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
        bool publish_twist = false;
        unsigned char key = static_cast<unsigned char>(c);

        switch (key)
        {
        case KEYCODE_LEFT:
            current_mode_ = ControlMode::TRANSLATION;
            RCLCPP_INFO(get_logger(), "切换到平移模式");
            break;
        case KEYCODE_RIGHT:
            current_mode_ = ControlMode::ROTATION;
            RCLCPP_INFO(get_logger(), "切换到旋转模式");
            break;

        // 运动控制
        case KEYCODE_W:
            if (is_joint_locked_)
            {
                RCLCPP_WARN(get_logger(), "关节已锁死，无法移动");
                break;
            }
            if (current_mode_ == ControlMode::TRANSLATION)
                twist_msg->twist.linear.x = 1.0 * vel_cmd_;
            else
                twist_msg->twist.angular.y = 1.0 * vel_cmd_;
            publish_twist = true;
            break;
        case KEYCODE_S:
            if (is_joint_locked_)
            {
                RCLCPP_WARN(get_logger(), "关节已锁死，无法移动");
                break;
            }
            if (current_mode_ == ControlMode::TRANSLATION)
                twist_msg->twist.linear.x = -1.0 * vel_cmd_;
            else
                twist_msg->twist.angular.y = -1.0 * vel_cmd_;
            publish_twist = true;
            break;
        case KEYCODE_A:
            if (is_joint_locked_)
            {
                RCLCPP_WARN(get_logger(), "关节已锁死，无法移动");
                break;
            }
            if (current_mode_ == ControlMode::TRANSLATION)
                twist_msg->twist.linear.y = -1.0 * vel_cmd_;
            else
                twist_msg->twist.angular.z = 1.0 * vel_cmd_;
            publish_twist = true;
            break;
        case KEYCODE_D:
            if (is_joint_locked_)
            {
                RCLCPP_WARN(get_logger(), "关节已锁死，无法移动");
                break;
            }
            if (current_mode_ == ControlMode::TRANSLATION)
                twist_msg->twist.linear.y = 1.0 * vel_cmd_;
            else
                twist_msg->twist.angular.z = -1.0 * vel_cmd_;
            publish_twist = true;
            break;
        case KEYCODE_Q:
            if (is_joint_locked_)
            {
                RCLCPP_WARN(get_logger(), "关节已锁死，无法移动");
                break;
            }
            if (current_mode_ == ControlMode::TRANSLATION)
                twist_msg->twist.linear.z = 1.0 * vel_cmd_;
            else
                twist_msg->twist.angular.x = 1.0 * vel_cmd_;
            publish_twist = true;
            break;
        case KEYCODE_E:
            if (is_joint_locked_)
            {
                RCLCPP_WARN(get_logger(), "关节已锁死，无法移动");
                break;
            }
            if (current_mode_ == ControlMode::TRANSLATION)
                twist_msg->twist.linear.z = -1.0 * vel_cmd_;
            else
                twist_msg->twist.angular.x = -1.0 * vel_cmd_;
            publish_twist = true;
            break;

        // 坐标系切换：R/F 会禁用 B 级矿坐标系
        case KEYCODE_R:
            frame_to_publish_ = BASE_FRAME_ID;
            use_b_grade_frame_ = false;
            RCLCPP_INFO(get_logger(), "切换至世界坐标系 (base_link)，B级矿坐标系已禁用");
            break;
        case KEYCODE_F:
            frame_to_publish_ = IMAGE_FRAME_ID;
            use_b_grade_frame_ = false;
            RCLCPP_INFO(get_logger(), "切换至图像坐标系 (link3)，B级矿坐标系已禁用");
            break;

        // 进入模式
        case KEYCODE_T:
            if (current_special_mode_ != SpecialMode::A_MODE)
            {
                current_special_mode_ = SpecialMode::A_MODE;
                use_b_grade_frame_ = false; // 离开 B 模式，禁用坐标系
                send_to_position_A_Base();
            }
            else
            {
                RCLCPP_INFO(get_logger(), "已处于A级矿模式，重新移动到A基础位置");
                send_to_position_A_Base();
            }
            break;
        case KEYCODE_Y:
            if (current_special_mode_ != SpecialMode::B_MODE)
            {
                current_special_mode_ = SpecialMode::B_MODE;
                send_to_position_B_Base();
            }
            else
            {
                RCLCPP_INFO(get_logger(), "已处于B级矿模式，重新移动到B基础位置");
                send_to_position_B_Base();
            }
            break;

        case KEYCODE_Z:
            if (current_special_mode_ != SpecialMode::NONE)
            {
                current_special_mode_ = SpecialMode::NONE;
                use_b_grade_frame_ = false;
                RCLCPP_INFO(get_logger(), "已退出矿模式，返回普通操作");
            }
            if (is_joint_locked_)
            {
                RCLCPP_WARN(get_logger(), "关节已锁死，无法执行开车位置");
                break;
            }
            RCLCPP_INFO(get_logger(), "开车位置");
            send_home_goal({0.0, 0.0, -0.24033, 0.10848, 0.0, 0.0, 0.64036});
            break;

        case KEYCODE_X:
            if (is_joint_locked_)
            {
                RCLCPP_WARN(get_logger(), "关节已锁死，无法移动");
                break;
            }
            if (current_special_mode_ == SpecialMode::A_MODE)
            {
                send_to_position_A_X();
            }
            else if (current_special_mode_ == SpecialMode::B_MODE)
            {
                send_to_position_B_X();
            }
            else
            {
                RCLCPP_WARN(get_logger(), "非矿模式下 X 键无效");
            }
            break;

        case KEYCODE_C:
            if (is_joint_locked_)
            {
                RCLCPP_WARN(get_logger(), "关节已锁死，无法移动");
                break;
            }
            if (current_special_mode_ == SpecialMode::A_MODE)
            {
                send_to_position_A_C();
            }
            else if (current_special_mode_ == SpecialMode::B_MODE)
            {
                if (!use_b_grade_frame_)
                {
                    use_b_grade_frame_ = true;
                    RCLCPP_INFO(get_logger(), "已启用 B 级矿坐标系 (速度控制将基于此坐标系，参考系自动设为 base_link)");
                }
                else
                {
                    RCLCPP_INFO(get_logger(), "B 级矿坐标系已启用，无需重复启用");
                }
            }
            else
            {
                RCLCPP_WARN(get_logger(), "非矿模式下 C 键无效");
            }
            break;

        case KEYCODE_V:
            if (is_joint_locked_)
            {
                unlockJoints();
            }
            else
            {
                RCLCPP_INFO(get_logger(), "关节当前处于自由状态，无需解锁");
            }
            break;

        case KEYCODE_B:
            if (!is_joint_locked_) {
                lockCurrentJoints();
            } else {
                RCLCPP_INFO(get_logger(), "关节已处于锁死状态，请勿重复操作");
            }
            break;

            // case KEYCODE_SPACE:
            //     RCLCPP_INFO(get_logger(), "退出程序");
            //     rclcpp::shutdown();
            //     break;

        default:
            break;
        }

        if (publish_twist)
        {
            if (use_b_grade_frame_)
            {
                // 将速度矢量从 b_grade_frame 旋转到 base_link
                if (current_mode_ == ControlMode::TRANSLATION)
                {
                    tf2::Vector3 v(twist_msg->twist.linear.x,
                                   twist_msg->twist.linear.y,
                                   twist_msg->twist.linear.z);
                    v = tf2::quatRotate(b_grade_rotation_, v);
                    twist_msg->twist.linear.x = v.x();
                    twist_msg->twist.linear.y = v.y();
                    twist_msg->twist.linear.z = v.z();
                }
                else
                {
                    tf2::Vector3 w(twist_msg->twist.angular.x,
                                   twist_msg->twist.angular.y,
                                   twist_msg->twist.angular.z);
                    w = tf2::quatRotate(b_grade_rotation_, w);
                    twist_msg->twist.angular.x = w.x();
                    twist_msg->twist.angular.y = w.y();
                    twist_msg->twist.angular.z = w.z();
                }
                twist_msg->header.frame_id = BASE_FRAME_ID;
            }
            else
            {
                twist_msg->header.frame_id = frame_to_publish_;
            }
            twist_msg->header.stamp = now();
            twist_pub_->publish(std::move(twist_msg));
        }
    }

    void KeyboardServo::sendData(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        current_joint_positions_ = msg->position;

        std::vector<std::string> target_joints = {
            "joint0", "joint1", "joint2", "joint3",
            "joint4", "joint5", "joint6"};

        std::vector<double> ordered_positions(target_joints.size(), 0.0);

        try
        {
            SendPacket packet;
            for (size_t i = 0; i < msg->name.size(); ++i)
            {
                if (msg->name[i] == "joint0")
                {
                    ordered_positions[0] = msg->position[i];
                    packet.joint0_state = msg->position[i];
                }
                else if (msg->name[i] == "joint1")
                {
                    ordered_positions[1] = msg->position[i];
                    packet.joint1_state = msg->position[i];
                }
                else if (msg->name[i] == "joint2")
                {
                    ordered_positions[2] = msg->position[i];
                    packet.joint2_state = msg->position[i];
                }
                else if (msg->name[i] == "joint3")
                {
                    ordered_positions[3] = msg->position[i];
                    packet.joint3_state = msg->position[i];
                }
                else if (msg->name[i] == "joint4")
                {
                    ordered_positions[4] = msg->position[i];
                    packet.joint4_state = msg->position[i];
                }
                else if (msg->name[i] == "joint5")
                {
                    ordered_positions[5] = msg->position[i];
                    packet.joint5_state = msg->position[i];
                }
                else if (msg->name[i] == "joint6")
                {
                    ordered_positions[6] = msg->position[i];
                    packet.joint6_state = msg->position[i];
                }
            }

            crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
            std::vector<uint8_t> data = toVector(packet);
            port_->serial_driver_->port()->send(data);
        }
        catch (const std::exception &ex)
        {
            RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
            port_->reopenPort();
        }

        current_joint_positions_ = ordered_positions;
        has_joint_state_ = true;
    }
};

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(ext_serial_driver::KeyboardServo)
