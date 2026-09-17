#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

using namespace std::placeholders;
 
class SmartPhone : public rclcpp::Node // MODIFY NAME
{
public:
    SmartPhone() : Node("smart_phone") // MODIFY NAME
    {
        subscriber_ = this->create_subscription<example_interfaces::msg::String>(
            "robot_news", 10,
            std::bind(&SmartPhone::callback_robotnews, this,_1));
        RCLCPP_INFO(this->get_logger(), "smartphone has been started");
    }
 
private:
    void callback_robotnews(const example_interfaces::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "%s", msg->data.c_str());
    }

    rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr subscriber_;
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SmartPhone>(); // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
