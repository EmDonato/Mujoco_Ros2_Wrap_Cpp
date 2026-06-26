#pragma once

#include <rclcpp/rclcpp.hpp>


#include "sensor_msgs/msg/joint_state.hpp" 
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include <tf2_ros/transform_broadcaster.h>

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <cstring>
#include <vector>
#include <string>

class MujocoWrap : public rclcpp::Node
{
public:
    MujocoWrap();
    ~MujocoWrap();

    void render_once();
    bool viewer_running() const;
    float get_fps();

private:
    // ================= ROS2 =================
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;

    rclcpp::TimerBase::SharedPtr timer_odom_;
    rclcpp::TimerBase::SharedPtr timer_twist_;
    rclcpp::TimerBase::SharedPtr timer_joints_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    sensor_msgs::msg::JointState msg;
    geometry_msgs::msg::TwistStamped twist;
    nav_msgs::msg::Odometry odom;

    // ================= MuJoCo =================
    mjModel* model_ = nullptr;
    mjData* data_ = nullptr;
    mjData* data_render_  = nullptr;

    bool pause_ = false;
    double dt_ = 0.0;

    // =========== Configurable params ==========
    std::string  file_xml;
    std::string  cntrl_type;

    int sim_time_period_us;
    float fps;

    int odom_time_period_ms;
    int joint_time_period_ms;
    int twist_time_period_ms;
    
    bool show_axes;
    bool show_contact_forces;
    bool show_contact_point;
    bool show_joint_direction;


    // ================= Viewer =================
    int body_id;
    mjvCamera cam_;
    mjvOption opt_;
    mjvScene scn_;
    mjrContext con_;
    GLFWwindow* window_ = nullptr;

    // ================= Threading =================
    std::thread physics_thread_;
    std::mutex mutex_;        //phisics loop
    std::mutex mutex_ctrl_; 
    //std::mutex mutex_odom_;   //publish odom
    //std::mutex mutex_twist_;  //publish twist
    //std::mutex mutex_joints_; //publish joints

    std::atomic<bool> running_{false};
    std::vector<double> aus_input; //ausiliar variable for collect message input

    // ================= Methods =================
    void listener_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void physics_loop();
    void send_twist_();
    void send_odom_();
    void send_joints_();

    // GLFW callbacks
    static void mouse_button_callback(GLFWwindow* window, int button, int act, int mods);
    static void mouse_move_callback(GLFWwindow* window, double xpos, double ypos);
    static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
    static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);

};