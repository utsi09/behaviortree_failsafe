#include <behaviortree_cpp/basic_types.h>           // BT::Blackboard
#include "behaviortree_cpp/bt_factory.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "std_msgs/msg/float64.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <limits>

using namespace BT;

class CheckRiskLevel2 : public BT::ConditionNode
{
    public:
        CheckRiskLevel2(const std::string& name, const BT::NodeConfiguration& config) : BT::ConditionNode(name, config)
        {
            node_ = rclcpp::Node::make_shared("bt_check_risk_level_node"); //ROS2 노드 생성
            //risk_level 토픽 구독
            sub_ = node_-> create_subscription<std_msgs::msg::Float64> (
                "/risk_level", 10,
                [this](std_msgs::msg::Float64::SharedPtr msg) {
                    last_risk_ = msg->data;
                }
            );

        }
    static BT::PortsList providedPorts()
    {
        return {};
    }

    NodeStatus tick() override
    {
        rclcpp::spin_some(node_);
        if (last_risk_ >= 20) {
            std::cout<<"위험도 경고 수준 : level " << last_risk_ <<std::endl;
            return BT::NodeStatus::SUCCESS;
        }
        else {
            std::cout<<"위험도 경고 수준 이탈 "<< std::endl;
            return BT::NodeStatus::FAILURE;
        }    
    }

    private:
        rclcpp::Node::SharedPtr node_; //ROS2노드 생성
        rclcpp::Subscription<std_msgs::msg::Float64> :: SharedPtr sub_; //sub 노드?
        double last_risk_{std::numeric_limits<double>::infinity()};
};