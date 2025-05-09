#include <behaviortree_cpp/action_node.h>      // v4: Sync/Async 기본 헤더
#include "std_msgs/msg/int32.hpp"
#include <rclcpp/rclcpp.hpp>
#include <limits>
//#include <action_node.h>
#include "rclcpp_action/rclcpp_action.hpp"
#include "warning_mode_interfaces/action/warning_mode.hpp"
#include <iostream>

using namespace std;
using namespace std::chrono;
using namespace BT;

using WarningModeAction = warning_mode_interfaces::action::WarningMode;
using GoalHandleWarningMode = rclcpp_action::ClientGoalHandle<WarningModeAction>;

class HandBrakePark : public StatefulActionNode {
public:
    HandBrakePark(const std::string& name, const NodeConfig& cfg)
      : StatefulActionNode(name, cfg)
    {
        /* 블랙보드에서 ROS2 노드 가져오기 */
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        client_ = rclcpp_action::create_client<WarningModeAction>(node_,"warning_mode");
        
        goal_sent_ = false;
        result_received_=false;
        
        // sub_ = node_->create_subscription<std_msgs::msg::Int32>(
        //           "/risk_level", 10,
        //           [this](const std_msgs::msg::Int32::SharedPtr msg)
        //           {
        //               risklevel_ = msg->data;
        //           });
    }

    /* 포트가 없으면 빈 map 리턴 */
    static PortsList providedPorts() { return {}; }

    // StatefulActionNode의 순수 가상 함수 구현
    NodeStatus onStart() override
    {
        if (!client_->wait_for_action_server(2s)) { // 2초간 응답없으면 fail 처리
            RCLCPP_ERROR(node_->get_logger(), "Action server not available!");
            return NodeStatus::FAILURE;
        }
        
        auto goal_msg = WarningModeAction::Goal();
        goal_msg.mode=3;

        auto send_goal_options = rclcpp_action::Client<WarningModeAction>::SendGoalOptions();
        send_goal_options.result_callback=[this](const GoalHandleWarningMode::WrappedResult &result){
            result_code_ = result.code; //임시 콜백 변수에 골 옵션 담은걸 결과 코드 변수에 넣기
            result_received_ = true; // 받았으면 트루
        };
        
        client_ -> async_send_goal(goal_msg, send_goal_options); // 옵션이랑 골 메시지 타입 클라이언트에 넣기
        cout<<"저속 주행 신호 전송 성공"<<endl;
        goal_sent_=true; 
        result_received_=false;

        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override
    {
        /* 콜백 처리 */
        rclcpp::spin_some(node_);

        if(!goal_sent_) return NodeStatus::RUNNING;
        if (!result_received_) return NodeStatus::RUNNING;
        
        switch (result_code_) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                return NodeStatus::SUCCESS;

            case rclcpp_action::ResultCode::ABORTED:
            case rclcpp_action::ResultCode::CANCELED:
            default:
                return NodeStatus::FAILURE;

        }


        // if (!received_)
        //     return NodeStatus::RUNNING;
            
        // return (feedback_ == 2) ? NodeStatus::SUCCESS 
        //                        : NodeStatus::FAILURE;
    }


    void onHalted() override
    {
        goal_sent_ = false;
        result_received_=false;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
    rclcpp_action::Client<WarningModeAction>::SharedPtr client_;


    int risklevel_ {0};
    bool goal_sent_;
    bool result_received_;
    rclcpp_action::ResultCode result_code_;

    //
    int goal_{0};
    int cancel_{0};

    int  feedback_{0};


    bool sent_{false};
    bool received_{false};


};