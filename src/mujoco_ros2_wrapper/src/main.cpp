#include <rclcpp/rclcpp.hpp>
#include "mujoco_ros2_wrapper/mj_ros2_wrap.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MujocoWrap>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);


    std::thread ros_thread([&executor]() {
        executor.spin();   
    });

    rclcpp::WallRate rate(node->get_fps()); //fps simulation views

    while (rclcpp::ok() && node->viewer_running()) {
        node->render_once();
        rate.sleep();
    }


    executor.cancel();
    rclcpp::shutdown();
    if (ros_thread.joinable())
        ros_thread.join();
    return 0;
}