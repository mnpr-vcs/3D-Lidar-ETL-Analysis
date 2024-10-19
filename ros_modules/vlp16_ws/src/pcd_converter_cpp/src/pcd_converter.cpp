#include "rclcpp/rclcpp.hpp"

class PcdConverter : public rclcpp::Node
{
public:
    PcdConverter(): Node("pcd_convert"), counter_(0)
    {
       RCLCPP_INFO(this->get_logger(), "PCD converter ...");

       timer_ = this->create_wall_timer( std::chrono::seconds(1)
                                       , std::bind(&PcdConverter::timerCallback
                                       , this));
    }
private:
    void timerCallback()
    {
        counter_ ++;
        RCLCPP_INFO(this->get_logger(), "Count %d", counter_);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PcdConverter>();
    // auto node = std::make_shared<rclcpp::Node>("pcd_convert");
    // RCLCPP_INFO(node->get_logger(), "PCD converter ...");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}