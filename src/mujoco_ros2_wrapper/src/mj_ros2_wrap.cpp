#include "mujoco_ros2_wrapper/mj_ros2_wrap.hpp"
#include <chrono>
#include <thread>
#include <cmath>
#include <string>
#include <Eigen/Geometry>


// ================= COSTRUCTORS =================
MujocoWrap::MujocoWrap() : Node("mujoco_wrap_node") {


    // =========================== PARAMS ==============================
    
    this->declare_parameter("file_name", "Robots/unitree_g1/scene.xml");
    this->declare_parameter("cntrl_type", "TRQ");

    this->declare_parameter("timer.sim_time_us", 2000);
    this->declare_parameter("timer.fps", 30.0);

    this->declare_parameter("topic.odom_pub_ms",   100);
    this->declare_parameter("topic.twist_pub_ms",  100);
    this->declare_parameter("topic.joints_pub_ms", 100);

    this->declare_parameter("viewer.show_axes",   false);
    this->declare_parameter("viewer.show_force",  false);
    this->declare_parameter("viewer.show_points", false);
    this->declare_parameter("viewer.show_joint",  false);


    this->get_parameter("file_name",                  file_xml);
    this->get_parameter("cntrl_type",                 cntrl_type);


    this->get_parameter("timer.sim_time_us",          sim_time_period_us);
    this->get_parameter("timer.fps",                  fps);

    this->get_parameter("topic.odom_pub_ms",          odom_time_period_ms);
    this->get_parameter("topic.twist_pub_ms",         twist_time_period_ms);
    this->get_parameter("topic.joints_pub_ms",        joint_time_period_ms);

    this->get_parameter("viewer.show_axes",           show_axes);
    this->get_parameter("viewer.show_force",          show_contact_forces);
    this->get_parameter("viewer.show_points",         show_contact_point);
    this->get_parameter("viewer.show_joint",          show_joint_direction);





    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = callback_group_;
    
    //publisher
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::SensorDataQoS());
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/twist", rclcpp::SensorDataQoS());
    odom_pub_  = this->create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::SensorDataQoS());
    clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());

    //subscriber
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joints_torque_input", 10, std::bind(&MujocoWrap::listener_callback, this, std::placeholders::_1), sub_options);
    
    // MUJOCO INIT
    char error[1000] = "Could not load model";
    std::cout << "MuJoCo version: " << mj_versionString() << std::endl;

    model_ = mj_loadXML(const_cast<char*>(file_xml.c_str()), //TODO PASS FROM CONFIG FILE
           nullptr,
           error,
           sizeof(error));
    if (!model_) throw std::runtime_error(error);

    // Ottimizzazioni solutore // TODDO CONFIG FILE 
    model_->opt.integrator = mjINT_IMPLICITFAST;
    model_->opt.iterations = 50;
    model_->opt.tolerance = 1e-5;

    //body_id = 0;//mj_name2id(model_, mjOBJ_BODY, "base");

    data_ = mj_makeData(model_);
    data_render_ = mj_makeData(model_);
    dt_ = model_->opt.timestep; 

    // --------VIEWER INIT--------------
    if (!glfwInit()) throw std::runtime_error("Could not initialize GLFW");

    window_ = glfwCreateWindow(1200, 900, "MuJoCo Viewer", nullptr, nullptr);//size windows
    if (!window_) { glfwTerminate(); throw std::runtime_error("Failed to create GLFW window"); }

    glfwMakeContextCurrent(window_);
    glfwSwapInterval(0); 

    glfwSetWindowUserPointer(window_, this);
    glfwSetMouseButtonCallback(window_, MujocoWrap::mouse_button_callback);
    glfwSetCursorPosCallback(window_, MujocoWrap::mouse_move_callback);
    glfwSetScrollCallback(window_, MujocoWrap::scroll_callback);
    glfwSetKeyCallback(window_, MujocoWrap::key_callback);

    mjv_defaultCamera(&cam_);
    
    cam_.type = mjCAMERA_TRACKING;
    cam_.trackbodyid = 0;
    

    //cam_.type = mjCAMERA_FREE;
    
    mjv_defaultOption(&opt_);
    mjv_defaultScene(&scn_);
    mjr_defaultContext(&con_);
    mjv_makeScene(model_, &scn_, 2000);
    mjr_makeContext(model_, &con_, mjFONTSCALE_150);

    //view graphics elemnts 
    if(show_joint_direction) opt_.flags[mjVIS_JOINT] = 1; //axes joint
    //if() opt_.flags[mjVIS_ACTUATOR] = 1;     //actuator direction
    if(show_contact_point) opt_.flags[mjVIS_CONTACTPOINT] = 1; //contact point
    if(show_contact_forces) opt_.flags[mjVIS_CONTACTFORCE] = 1; //contact forces
    if(show_axes) opt_.frame = mjFRAME_BODY;            //axes

    //timers
    timer_odom_ = create_wall_timer(std::chrono::milliseconds(odom_time_period_ms), 
                                    std::bind(&MujocoWrap::send_odom_, this));
    timer_twist_ = create_wall_timer(std::chrono::milliseconds(twist_time_period_ms), 
                                    std::bind(&MujocoWrap::send_twist_, this));
    timer_joints_ = create_wall_timer(std::chrono::milliseconds(joint_time_period_ms), 
                                    std::bind(&MujocoWrap::send_joints_, this));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    aus_input.resize(model_->nu, 0.0);
    running_ = true;
    physics_thread_ = std::thread(&MujocoWrap::physics_loop, this);

}

MujocoWrap::~MujocoWrap() {
    running_ = false;
    if (physics_thread_.joinable()) physics_thread_.join();
    mjr_freeContext(&con_);
    mjv_freeScene(&scn_);
    if (window_) { glfwDestroyWindow(window_); glfwTerminate(); }
    if (data_) mj_deleteData(data_);
    if (data_render_) mj_deleteData(data_render_);
    if (model_) mj_deleteModel(model_);
}

// ================= CALLBACKS ROS2 ===============
void MujocoWrap::listener_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_ctrl_);

    for (size_t i = 0; i < msg->name.size() && i < msg->effort.size(); ++i)
    {
        int id = mj_name2id(model_, mjOBJ_ACTUATOR, msg->name[i].c_str());

        if (id >= 0 && id < static_cast<int>(aus_input.size()))
        {
            if (cntrl_type == "TRQ")   aus_input[id] = msg->effort[i];
            else if (cntrl_type == "VEL") aus_input[id] = msg->velocity[i];
            else if (cntrl_type == "POS") aus_input[id] = msg->position[i];
        }
    }
}

void MujocoWrap::send_joints_(){

    // ======================
    //     JOINT STATES
    // ====================== 
    std::lock_guard<std::mutex> lock(mutex_);

    msg.header.stamp.sec  = static_cast<int32_t>(data_->time);
    msg.header.stamp.nanosec  = static_cast<uint32_t>((data_->time - msg.header.stamp.sec) * 1e9);  

    msg.name.clear();
    msg.position.clear();
    msg.velocity.clear();
    msg.effort.clear();

    for (int jid = 0; jid < model_->njnt; jid++)
    {
        if (model_->jnt_type[jid] == mjJNT_FREE)
            continue;

        const char* name = mj_id2name(model_, mjOBJ_JOINT, jid);

        msg.name.push_back(name ? name : "");
        msg.position.push_back(data_->qpos[model_->jnt_qposadr[jid]]);
        msg.velocity.push_back(data_->qvel[model_->jnt_dofadr[jid]]);
        msg.effort.push_back(data_->qfrc_actuator[model_->jnt_dofadr[jid]]);
    }
    joint_pub_->publish(msg);

}

void MujocoWrap::send_odom_(){

        
    // ================================
    // ODOM MESSAGE (CENTER FREE JOINT)
    // ================================

    odom = nav_msgs::msg::Odometry();
    {
        std::lock_guard<std::mutex> lock(mutex_);

        odom.header.stamp.sec  = static_cast<int32_t>(data_->time);
        odom.header.stamp.nanosec  = static_cast<uint32_t>((data_->time - odom.header.stamp.sec) * 1e9);  
        odom.header.frame_id = "world";
        odom.child_frame_id  = "base_link";

        // Position
        odom.pose.pose.position.x = data_->qpos[0];
        odom.pose.pose.position.y = data_->qpos[1];
        odom.pose.pose.position.z = data_->qpos[2];

        odom.pose.pose.orientation.w = data_->qpos[3];
        odom.pose.pose.orientation.x = data_->qpos[4];
        odom.pose.pose.orientation.y = data_->qpos[5];
        odom.pose.pose.orientation.z = data_->qpos[6];

        Eigen::Quaterniond q(
            data_->qpos[3],  // w
            data_->qpos[4],  // x
            data_->qpos[5],  // y
            data_->qpos[6]   // z
        );

        Eigen::Vector3d v_world(data_->qvel[0], data_->qvel[1], data_->qvel[2]);
        Eigen::Vector3d v_body = q.inverse() * v_world;

        odom.twist.twist.linear.x = v_body.x();  
        odom.twist.twist.linear.y = v_body.y();  
        odom.twist.twist.linear.z = v_body.z();  


        odom.twist.twist.angular.x = data_->qvel[3];
        odom.twist.twist.angular.y = data_->qvel[4];
        odom.twist.twist.angular.z = data_->qvel[5];
    }
        
    odom_pub_->publish(odom);  

}

void MujocoWrap::send_twist_(){

    // ================================
    // TWISTED VEL (CENTER FREE JOINT)
    // ================================
    twist = geometry_msgs::msg::TwistStamped();
    {
        std::lock_guard<std::mutex> lock(mutex_);

        twist.header.stamp.sec  = static_cast<int32_t>(data_->time);
        twist.header.stamp.nanosec  = static_cast<uint32_t>((data_->time - twist.header.stamp.sec) * 1e9); 
        twist.header.frame_id = "base_link";  // body frame
        Eigen::Quaterniond q(
            data_->qpos[3],  // w
            data_->qpos[4],  // x
            data_->qpos[5],  // y
            data_->qpos[6]   // z
        );

        Eigen::Vector3d v_world(data_->qvel[0], data_->qvel[1], data_->qvel[2]);
        Eigen::Vector3d v_body = q.inverse() * v_world;

        twist.twist.linear.x  = v_body.x();
        twist.twist.linear.y  = v_body.y();
        twist.twist.linear.z  = v_body.z();

        twist.twist.angular.x = data_->qvel[3];  // wx
        twist.twist.angular.y = data_->qvel[4];  // wy
        twist.twist.angular.z = data_->qvel[5];  // wz
    }
    twist_pub_->publish(twist);
    
}

// ================= PHYSICS LOOP (500 HZ default) =================
void MujocoWrap::physics_loop() {

    //this is the main function for the mujoco simulation, 
    //here you do a step in the simulation and later publish the states in the messages 

    using namespace std::chrono;

    const microseconds interval(sim_time_period_us); //TODO time to be elect in teh config file 
    auto next_tick = high_resolution_clock::now();

    while (rclcpp::ok() && running_) {

        {

            
            // ----------------- MUJOCO -----------------

                // ======================
                // CONTROL INPUT
                // ======================
                std::vector<double> input_copy; //safe copy 
                //I can smell a bullshit here, i feel it
                {
                    std::lock_guard<std::mutex> lock(mutex_ctrl_);
                    input_copy = aus_input;
                }

                {
                    std::lock_guard<std::mutex> lock(mutex_);

                    for (int i = 0; i < model_->nu; i++)
                    {
                        data_->ctrl[i] = input_copy[i];
                    }
                    // ======================
                    // STEP SIMULATION
                    // ======================
                    if (!pause_)
                    {
                        mj_step(model_, data_);
                    }
                }

            // . - . - . - . - . - . - . - . - . - . - . - . - .
        }
            // OUTPUT SAME HZ THAN PHYSICS LOOP
            // ------------------ ROS2 -------------------

        {
            // ================================
            //         CLOCK
            // ================================ 
            rosgraph_msgs::msg::Clock msg;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                msg.clock.sec = static_cast<int32_t>(data_->time);
                msg.clock.nanosec = static_cast<uint32_t>((data_->time - msg.clock.sec) * 1e9);  
            }
            clock_pub_->publish(msg);

            // ================================
            //         TF SEND
            // ==========================         

            std::vector<geometry_msgs::msg::TransformStamped> tfs;
            {
                std::lock_guard<std::mutex> lock(mutex_);

                for (int body_id = 1; body_id < model_->nbody; body_id++)
                {

                    geometry_msgs::msg::TransformStamped tf;

                    tf.header.stamp.sec = static_cast<int32_t>(data_->time);
                    tf.header.stamp.nanosec= static_cast<uint32_t>((data_->time - tf.header.stamp.sec) * 1e9);  

                    tf.header.frame_id = "world";
                    tf.child_frame_id =  mj_id2name(model_, mjOBJ_BODY, body_id);

                    tf.child_frame_id =
                        mj_id2name(model_, mjOBJ_BODY, body_id);
                    tf.transform.translation.x = data_->xpos[3*body_id + 0];
                    tf.transform.translation.y = data_->xpos[3*body_id + 1];
                    tf.transform.translation.z = data_->xpos[3*body_id + 2];

                    tf.transform.rotation.w = data_->xquat[4*body_id + 0];
                    tf.transform.rotation.x = data_->xquat[4*body_id + 1];
                    tf.transform.rotation.y = data_->xquat[4*body_id + 2];
                    tf.transform.rotation.z = data_->xquat[4*body_id + 3];
                    tfs.push_back(tf);
                }
            }
            tf_broadcaster_->sendTransform(tfs);


        }
            // . - . - . - . - . - . - . - . - . - . - . - . - .


            
        next_tick += interval;
        std::this_thread::sleep_until(next_tick);
    }
}
// status mouse file-local
namespace {
    bool button_left = false;
    bool button_middle = false;
    bool button_right = false;
    double lastx = 0.0;
    double lasty = 0.0;
} //            
// ================= GLFW CALLBACKS =================
void MujocoWrap::mouse_button_callback(GLFWwindow* window, int, int, int) {
    button_left   = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)   == GLFW_PRESS;
    button_middle = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
    button_right  = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)  == GLFW_PRESS;
    glfwGetCursorPos(window, &lastx, &lasty);
}

void MujocoWrap::mouse_move_callback(GLFWwindow* window, double xpos, double ypos) {
    auto* node = static_cast<MujocoWrap*>(glfwGetWindowUserPointer(window));
    if (!node || (!button_left && !button_middle && !button_right)) return;

    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos; lasty = ypos;

    int width, height;
    glfwGetWindowSize(window, &width, &height);
    if (height <= 0) return;

    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    mjtMouse action;
    if (button_right)      action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if (button_left)  action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else                   action = mjMOUSE_ZOOM;

    std::lock_guard<std::mutex> lock(node->mutex_);
    mjv_moveCamera(node->model_, action, dx / height, dy / height, &node->scn_, &node->cam_);
}

void MujocoWrap::scroll_callback(GLFWwindow* window, double, double yoffset) {
    auto* node = static_cast<MujocoWrap*>(glfwGetWindowUserPointer(window));
    if (!node) return;
    std::lock_guard<std::mutex> lock(node->mutex_);
    mjv_moveCamera(node->model_, mjMOUSE_ZOOM, 0.0, -0.05 * yoffset, &node->scn_, &node->cam_);
}

void MujocoWrap::key_callback(GLFWwindow* window, int key, int scancode, int action, int mods){
    auto* self = static_cast<MujocoWrap*>(glfwGetWindowUserPointer(window));
    if (!self)
        return;

    if (action != GLFW_PRESS)
        return;

    switch(key)
    {
        case GLFW_KEY_P:

            self->pause_ = !self->pause_;
            break;

        case GLFW_KEY_SPACE:

            self->pause_ = !self->pause_;
            break;

        case GLFW_KEY_R:

            //TODO reset status
            break;

        case GLFW_KEY_ESCAPE:

            glfwSetWindowShouldClose(
                window,
                GLFW_TRUE);

            break;
    }
}

// ==================== MUJOCO RENDER ======================
void MujocoWrap::render_once() {
    if (!window_) return;
    glfwMakeContextCurrent(window_);

    {
        // fast copy for async render
        std::lock_guard<std::mutex> lock(mutex_);
        mj_copyData(data_render_, model_, data_); 
        mjv_updateScene(model_, data_render_, &opt_, nullptr, &cam_, mjCAT_ALL, &scn_); //update render

    }

    mjrRect viewport{0, 0, 0, 0};
    glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);

    mjr_render(viewport, &scn_, &con_);
    mjr_overlay(
        mjFONT_NORMAL,
        mjGRID_TOPLEFT,
        viewport,
        ("Mode:" + cntrl_type + "\n" "Pause: " + std::to_string(pause_) + "\n").c_str(), "",
        &con_);
    glfwSwapBuffers(window_);
    glfwPollEvents();
}

bool MujocoWrap::viewer_running() const {
    
    return window_ && !glfwWindowShouldClose(window_);
}

// ====================   UTILITIES ======================
float MujocoWrap::get_fps() {
    return fps;
}
