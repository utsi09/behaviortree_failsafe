#include <behaviortree_cpp/action_node.h>      // v4: Sync/Async 기본 헤더
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64.hpp"
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

class WarningMode : public StatefulActionNode {
public:
    WarningMode(const std::string& name, const NodeConfig& cfg)
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
        if (!sub_) {
            sub_ = node_->create_subscription<std_msgs::msg::Float64>(
                "risk_level", 10,
                [this](const std_msgs::msg::Float64::SharedPtr msg){
                    risklevel_ = msg->data;
                });
        }
        
        auto goal_msg = WarningModeAction::Goal();
        goal_msg.mode=0;

        auto send_goal_options = rclcpp_action::Client<WarningModeAction>::SendGoalOptions();
        send_goal_options.result_callback=[this](const GoalHandleWarningMode::WrappedResult &result){
            result_code_ = result.code; //임시 콜백 변수에 골 옵션 담은걸 결과 코드 변수에 넣기
            result_received_ = true; // 받았으면 트루
        };
        
        // client_ -> async_send_goal(goal_msg, send_goal_options); // 옵션이랑 골 메시지 타입 클라이언트에 넣기

        auto future_goal_handle = client_->async_send_goal(goal_msg, send_goal_options);
        //goal_handle_ = future_goal_handle.get(); // 목표 핸들 저장
        cout<<"저속 주행 신호 전송 성공"<<endl;
        goal_sent_=true; 
        result_received_=false;

        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override
    {

        if (!rclcpp::ok()) {
            std::cout << "[BT 종료 요청 감지] ROS 중단됨" << std::endl;
            onHalted();
            return NodeStatus::FAILURE;
        }

        /* 콜백 처리 */
        rclcpp::spin_some(node_);

        if(!goal_sent_) return NodeStatus::RUNNING;
        if (!result_received_) return NodeStatus::RUNNING;
        
        if (risklevel_ < 20)
        {   
            onHalted();
            return NodeStatus::SUCCESS;
        }
        if (risklevel_ > 100)
        {
            onHalted();
            return NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING; //emp : 석시드에서 러닝 수정
        // switch (result_code_) { // 20 ~ 100이면 계속 "저속" 주행해야 하므로 서버의 결과에 따라 종료하지않음 판단은 여기에서

        //     case rclcpp_action::ResultCode::SUCCEEDED:
        //         return NodeStatus::SUCCESS;
        //     case rclcpp_action::ResultCode::ABORTED:
        //     case rclcpp_action::ResultCode::CANCELED:
        //     default:
        //         return NodeStatus::FAILURE;

        //}       
    }


    void onHalted() override
    {
        goal_sent_ = false;
        result_received_=false;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;
    rclcpp_action::Client<WarningModeAction>::SharedPtr client_;


    int risklevel_ {0};
    bool goal_sent_;
    bool result_received_;
    rclcpp_action::ResultCode result_code_;

    //
    int goal_{0};
    int cancel_{0};

    int feedback_{0};


    bool sent_{false};
    bool received_{false};


};