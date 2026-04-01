#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <memory>
#include <thread>
#include <mutex>
#include <cmath>

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

// 话题和帧配置
const std::string TWIST_TOPIC = "/servo_demo_node/delta_twist_cmds";
const std::string JOINT_STATE_TOPIC = "/joint_states";
const size_t ROS_QUEUE_SIZE = 10;
const std::string IMAGE_FRAME_ID = "link3";
const std::string BASE_FRAME_ID = "base_link";

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

class KeyboardReader
{
public:
    KeyboardReader() : kfd(0)
    {
        tcgetattr(kfd, &cooked);
        struct termios raw;
        memcpy(&raw, &cooked, sizeof(struct termios));
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VEOL] = 1;
        raw.c_cc[VEOF] = 2;
        tcsetattr(kfd, TCSANOW, &raw);
    }

    void readOne(char *c)
    {
        int rc = read(kfd, c, 1);
        if (rc < 0)
            throw std::runtime_error("read failed");
    }

    void shutdown()
    {
        tcsetattr(kfd, TCSANOW, &cooked);
    }

private:
    int kfd;
    struct termios cooked;
};

class KeyboardServo : public rclcpp::Node
{
public:
    KeyboardServo() : Node("keyboard_servo_node"),
                      current_mode_(ControlMode::TRANSLATION),
                      current_special_mode_(SpecialMode::NONE),
                      is_joint_locked_(false),
                      vel_cmd_(0.2),
                      use_b_grade_frame_(false)
    {
        twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, ROS_QUEUE_SIZE);
        action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this, "/manipulator_controller/follow_joint_trajectory");
        joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            JOINT_STATE_TOPIC, ROS_QUEUE_SIZE,
            std::bind(&KeyboardServo::jointStateCallback, this, std::placeholders::_1));

        init_b_grade_rotation();
        publish_b_grade_tf();
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        current_joint_positions_ = msg->position;

        std::vector<std::string> target_joints = {
            "joint0", "joint1", "joint2", "joint3",
            "joint4", "joint5", "joint6"};

        std::vector<double> ordered_positions(target_joints.size(), 0.0);
        for (size_t i = 0; i < target_joints.size(); ++i)
        {
            for (size_t j = 0; j < msg->name.size(); ++j)
            {
                if (msg->name[j] == target_joints[i])
                {
                    ordered_positions[i] = msg->position[j];
                    break;
                }
            }
        }

        current_joint_positions_ = ordered_positions;
        has_joint_state_ = true;
    }

    void send_home_goal(const std::vector<double> &positions)
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

    void lockCurrentJoints()
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

    void unlockJoints()
    {
        is_joint_locked_ = false;
        RCLCPP_INFO(get_logger(), "关节已解锁");
    }

    void keyLoop()
    {
        char c;
        KeyboardReader input;
        std::thread spin_thread([this]()
                                { rclcpp::spin(shared_from_this()); });
        print_instructions();

        while (rclcpp::ok())
        {
            try
            {
                input.readOne(&c);
            }
            catch (const std::runtime_error &e)
            {
                RCLCPP_ERROR(get_logger(), "读取键盘输入失败: %s", e.what());
                break;
            }
            process_key(c);
        }

        input.shutdown();
        spin_thread.join();
    }

private:
    enum class ControlMode
    {
        TRANSLATION,
        ROTATION
    };
    enum class SpecialMode
    {
        NONE,
        A_MODE,
        B_MODE
    };

    ControlMode current_mode_;
    SpecialMode current_special_mode_;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

    std::mutex joint_state_mutex_;
    std::vector<double> current_joint_positions_;
    bool has_joint_state_ = false;
    bool is_joint_locked_;

    std::string frame_to_publish_ = BASE_FRAME_ID;
    double vel_cmd_;
    bool use_b_grade_frame_;
    tf2::Quaternion b_grade_rotation_; // 从 b_grade_frame 到 base_link 的旋转

    // 修改：旋转顺序为 先绕 Y +30°，再绕旋转后的 X +27°
    void init_b_grade_rotation()
    {
        // 先绕 Y 轴旋转 +30°
        tf2::Quaternion q_y;
        q_y.setRPY(0, 30.0 * M_PI / 180.0, 0);
        tf2::Transform t_y(q_y, tf2::Vector3(0, 0, 0));

        // 新 X 轴方向（在 base_link 中），即旋转后的 X 轴
        tf2::Vector3 new_x = t_y * tf2::Vector3(1, 0, 0);
        new_x.normalize();

        // 绕新 X 轴旋转 +27° 的四元数
        double angle = 27.0 * M_PI / 180.0;
        tf2::Quaternion q_local_axis;
        q_local_axis.setRotation(new_x, angle);
        q_local_axis.normalize();

        // 总旋转四元数 base_link -> b_grade_frame
        tf2::Quaternion q_base_to_b = q_y * q_local_axis; // 先应用 q_y，再应用 q_local_axis
        q_base_to_b.normalize();

        // 存储逆旋转（b_grade_frame -> base_link）
        b_grade_rotation_ = q_base_to_b.inverse();
    }

    // 修改：发布 TF 时使用相同的旋转
    void publish_b_grade_tf()
    {
        static tf2_ros::StaticTransformBroadcaster broadcaster(this);
        geometry_msgs::msg::TransformStamped transform;

        transform.header.stamp = now();
        transform.header.frame_id = BASE_FRAME_ID;
        transform.child_frame_id = "b_grade_frame";

        // 重新计算 q_base_to_b 用于 TF 发布
        tf2::Quaternion q_y;
        q_y.setRPY(0, 30.0 * M_PI / 180.0, 0);
        tf2::Transform t_y(q_y, tf2::Vector3(0, 0, 0));
        tf2::Vector3 new_x = t_y * tf2::Vector3(1, 0, 0);
        new_x.normalize();
        double angle = 27.0 * M_PI / 180.0;
        tf2::Quaternion q_local_axis;
        q_local_axis.setRotation(new_x, angle);
        tf2::Quaternion q_base_to_b = q_y * q_local_axis;
        q_base_to_b.normalize();

        transform.transform.rotation.x = q_base_to_b.x();
        transform.transform.rotation.y = q_base_to_b.y();
        transform.transform.rotation.z = q_base_to_b.z();
        transform.transform.rotation.w = q_base_to_b.w();

        transform.transform.translation.x = 0.0;
        transform.transform.translation.y = 0.0;
        transform.transform.translation.z = 0.0;

        broadcaster.sendTransform(transform);
        RCLCPP_INFO(get_logger(), "已发布 B 级矿坐标系静态变换 (先绕 Y +30°，再绕新 X +27°)");
    }

    // ========== A 级矿模式 ==========
    void send_to_position_A_X()
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

    void send_to_position_A_C()
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
    void send_to_position_B_X()
    {
        if (is_joint_locked_)
        {
            RCLCPP_WARN(get_logger(), "关节已锁死，无法移动");
            return;
        }
        RCLCPP_INFO(get_logger(), "B级矿模式 - X位置");
        send_home_goal({0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0}); // 占位
    }

    void send_to_position_A_Base()
    {
        if (is_joint_locked_)
        {
            RCLCPP_WARN(get_logger(), "关节已锁死，无法进入A级矿模式");
            return;
        }
        RCLCPP_INFO(get_logger(), "进入A级矿模式，移动到预设A基础位置");
        send_home_goal({0.01453, 0.00125, -0.17905, 0.28518, 0.00125, 0.10357, 1.68631});
    }

    void send_to_position_B_Base()
    {
        if (is_joint_locked_)
        {
            RCLCPP_WARN(get_logger(), "关节已锁死，无法进入B级矿模式");
            return;
        }
        RCLCPP_INFO(get_logger(), "进入B级矿模式，移动到预设B基础位置");
        send_home_goal({0.01316, 0.01038, -0.28587, 0.82598, 0.00545, 0.53621, 0.63569});
    }

    void print_instructions()
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
        puts("  B - 锁死/解锁当前关节角度");
        puts("  T - 进入A级矿模式");
        puts("  Y - 进入B级矿模式");
        puts("  SPACE - 退出程序");
        puts("\nA级矿模式下:");
        puts("  X - 左下方向速度 (与水平面夹角30度)");
        puts("  C - A级矿兑换位置");
        puts("\nB级矿模式下:");
        puts("  X - 位置1");
        puts("  C - 启用 B 级矿坐标系 (先绕 Y +30°，再绕新 X +27°)");
        puts("  按 R 或 F 可禁用 B 级矿坐标系，恢复正常控制");
        puts("\n非矿模式下 X/C/V 无效");
        puts("\n状态指示:");
        puts("  关节锁死状态: [按B键切换]");
        puts("  当前矿模式: 无 / A级 / B级");
        puts("  B级矿坐标系启用状态: 仅 B 级矿模式下可启用, 按C键切换, R/F/切换模式自动禁用");
    }

    void process_key(char c)
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
            RCLCPP_WARN(get_logger(), "V 键已无功能");
            break;

        case KEYCODE_B:
            if (is_joint_locked_)
                unlockJoints();
            else
                lockCurrentJoints();
            break;

        case KEYCODE_SPACE:
            RCLCPP_INFO(get_logger(), "退出程序");
            rclcpp::shutdown();
            break;

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
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardServo>();
    node->keyLoop();
    rclcpp::shutdown();
    return 0;
}