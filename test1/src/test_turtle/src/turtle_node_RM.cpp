#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <iostream>
#include "chrono"
#include "turtlesim/msg/pose.hpp"
using namespace std::chrono_literals;
class TurtleControlNode : public rclcpp::Node
{
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
    double target_x_y[8][2] = {{5.54,8.54},{6.54,7.54},{5.54,6.54},{6.54,5.54},
                            {7.04,8.54},{7.54,5.54},{7.84,8.54},{8.54,5.54}
    };
    int current_target_index_=0;
    double k_ = {1.5};
    double max_speed_ = {3.0};
    double max_error_ = {0.02};
public:
    TurtleControlNode(const std::string & node_name): Node(node_name)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose",
            10,
            std::bind(&TurtleControlNode::on_pose_received, this, std::placeholders::_1)
        );
    }

    void compute_target(double x, double y)
    {
        target_x_y[0][0] = x;
        target_x_y[0][1] = y;
    }

    void on_pose_received(const turtlesim::msg::Pose::SharedPtr pose)//参数：共享指针
    {
        auto current_x = pose->x;
        auto current_y = pose->y;
        RCLCPP_INFO(this->get_logger(), "current pose: x=%.2f, y=%.2f", current_x, current_y);

        double target_x_ = target_x_y[current_target_index_][0];
        double target_y_ = target_x_y[current_target_index_][1];
        //当前位置与目标位置差,角度差
        auto distance = std::sqrt(std::pow(target_x_ - current_x, 2) + std::pow(target_y_ - current_y, 2));
        auto angle = std::atan2(target_y_ - current_y, target_x_ - current_x)-pose->theta;
        auto msg = geometry_msgs::msg::Twist();
        if(distance > max_error_)
        {
            if(fabs(angle) > max_error_)
            {
                msg.angular.z =angle*k_ ;
                
            }
            else
            {
                msg.linear.x = k_ * distance;
                
            }
        }
        else 
        {
            current_target_index_++;
            if(current_target_index_>=7)
            {
                current_target_index_=7;
            }
        }
        

        if(msg.linear.x > max_speed_){ msg.linear.x = max_speed_;}
                   
        
        publisher_->publish(msg);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleControlNode>("turtle_ControlNode");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}