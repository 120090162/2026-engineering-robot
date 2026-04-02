#ifndef SERVO__RM_SERIAL_DRIVER_HPP_
#define SERVO__RM_SERIAL_DRIVER_HPP_

#include "franka_keyboard_control/crc.hpp"
#include "franka_keyboard_control/packet.hpp"
#include "franka_keyboard_control/serial_driver.hpp"
// ros2
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "sensor_msgs/msg/joint_state.hpp"
// tf2
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
#include <vector>
#include <chrono>

#include "rm_utils/heartbeat.hpp"

using namespace fyt;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

namespace ext_serial_driver
{
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
        // 话题和帧配置
        std::string TWIST_TOPIC = "/servo_demo_node/delta_twist_cmds";
        std::string JOINT_STATE_TOPIC = "/joint_states";
        size_t ROS_QUEUE_SIZE = 10;
        std::vector<std::string> frame;
        std::string IMAGE_FRAME_ID = "link3";
        std::string BASE_FRAME_ID = "base_link";

        explicit KeyboardServo(const rclcpp::NodeOptions &options);
        ~KeyboardServo() override;

    private:
        enum ControlMode
        {
            TRANSLATION,
            ROTATION
        };
        enum class SpecialMode
        {
            NONE,
            A_MODE,
            B_MODE,
            C_MODE,
        };

        ControlMode current_mode_ = TRANSLATION;
        SpecialMode current_special_mode_ = SpecialMode::NONE;

        // Serial port
        std::unique_ptr<Port> port_;

        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
        rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;

        // 键盘输入线程
        std::string control_type_;
        std::thread keyboard_thread_;

        std::mutex joint_state_mutex_;
        std::vector<double> current_joint_positions_;
        bool has_joint_state_ = false;
        bool is_joint_locked_ = false;

        std::string frame_to_publish_ = BASE_FRAME_ID;
        double vel_cmd_;

        bool use_b_grade_frame_ = false;
        tf2::Quaternion b_grade_rotation_; // 从 b_grade_frame 到 base_link 的旋转

        // Heartbeat
        HeartBeatPublisher::SharedPtr heartbeat_;

        void send_home_goal(const std::vector<double> &positions);
        void lockCurrentJoints();
        void unlockJoints();
        void init_b_grade_rotation(); // 旋转顺序为 先绕 Y +30°，再绕旋转后的 X +27°
        void publish_b_grade_tf();    // 发布 TF 时使用相同的旋转
        // ========== A 级矿模式 ==========
        void send_to_position_A_X();
        void send_to_position_A_C();
        // ========== B 级矿模式 ==========
        void send_to_position_B_X();
        void send_to_position_A_Base();
        void send_to_position_B_Base();
        void print_instructions();
        void process_key(char c);
        void keyboardLoop();
        void receiveData();
        void sendData(const sensor_msgs::msg::JointState::SharedPtr msg);
        void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    };
}

#endif // UP_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
