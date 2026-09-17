#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

using namespace std::chrono_literals;
 
class RobotNewsStation : public rclcpp::Node // MODIFY NAME
{
public:
    RobotNewsStation() : Node("robotnewsStation"), robot_name("n2d2") // MODIFY NAME
    {
        publisher_ = this->create_publisher<example_interfaces::msg::String>("robot_news", 10);
        timer = this->create_wall_timer(0.5s, std::bind(&RobotNewsStation::publisher_callback, this));
        RCLCPP_INFO(this->get_logger(), "robot news station started");
    }
 
private:
    void publisher_callback()
    {
        auto msg = example_interfaces::msg::String();
        msg.data = std::string("hi this is") + robot_name ;
        publisher_ ->publish(msg);
    }
    std::string robot_name;
    rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer;
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotNewsStation>(); // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
