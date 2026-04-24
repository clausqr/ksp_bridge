#include <ksp_bridge/ksp_bridge.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<KSPBridge>();

    // SIGINT during the connect/find retry loops shuts the rcl context down
    // before the node is fully initialized. Spinning on an already-shutdown
    // context throws while creating the executor's guard condition, so skip
    // spin() and fall through to a clean shutdown.
    if (rclcpp::ok()) {
        rclcpp::spin(node);
    }

    rclcpp::shutdown();

    return 0;
}