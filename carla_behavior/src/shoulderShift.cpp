#include <behaviortree_cpp/action_node.h>      // v4: Sync/Async 기본 헤더
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64.hpp"
#include <rclcpp/rclcpp.hpp>
#include <limits>
#include <future>
#include "rclcpp_action/rclcpp_action.hpp"
#include "warning_mode_interfaces/action/warning_mode.hpp"
#include <iostream>

using namespace std;
using namespace std::chrono_literals;
using namespace BT;

using WarningModeAction   = warning_mode_interfaces::action::WarningMode;
using GoalHandleWarningMode = rclcpp_action::ClientGoalHandle<WarningModeAction>;

/**
 * ShoulderShift  :  자율‑주행 차량이 위험도를 모니터링하다  (1) 4‑미만이면 goal 취소,
 *                  (2) 액션 서버 결과(SUCCEEDED / CANCELED) 를 받아 SUCCESS 로 종료,
 *                  (3) ABORTED 는 RUNNING(재시도) 로 유지하는 BT Action Node
 */
class ShoulderShift : public StatefulActionNode
{
public:
    ShoulderShift(const std::string& name, const NodeConfig& cfg)
        : StatefulActionNode(name, cfg)
    {
        node_   = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        client_ = rclcpp_action::create_client<WarningModeAction>(node_, "warning_mode");
    }

    //---------------------------------------------------------------------
    // BT PORTS -----------------------------------------------------------
    //---------------------------------------------------------------------
    static PortsList providedPorts() { return {}; }

    //---------------------------------------------------------------------
    //  BT onStart --------------------------------------------------------
    //---------------------------------------------------------------------
    NodeStatus onStart() override
    {
        // 1) 액션 서버 대기 (2 초 제한) -----------------------------------
        if (!client_->wait_for_action_server(2s)) {
            RCLCPP_ERROR(node_->get_logger(), "[CLIENT] Action server not available");
            return NodeStatus::FAILURE;
        }

        // 2) 위험도 topic 구독 -------------------------------------------
        if (!sub_) {
            sub_ = node_->create_subscription<std_msgs::msg::Float64>(
                "risk_level", 10,
                [this](const std_msgs::msg::Float64::SharedPtr msg) {
                    risklevel_ = msg->data;
                });
        }

        // 3) Goal 송신 ----------------------------------------------------
        WarningModeAction::Goal goal_msg;
        goal_msg.mode = 1;   // 1 = 갓길 주행

        rclcpp_action::Client<WarningModeAction>::SendGoalOptions opt;

        // --- Goal 수락/거절 ---------------------------------------------
        opt.goal_response_callback = [this](GoalHandleWarningMode::SharedPtr gh) {
            if (!gh) {
                RCLCPP_ERROR(node_->get_logger(), "[CLIENT] Goal rejected by server");
                result_code_     = rclcpp_action::ResultCode::UNKNOWN;
                result_received_ = true;            // 즉시 종료처리
                return;
            }
            goal_handle_ = gh;
            RCLCPP_INFO(node_->get_logger(), "[CLIENT] Goal accepted by server");
        };

        // --- 결과 수신 ----------------------------------------------------
        opt.result_callback = [this](const GoalHandleWarningMode::WrappedResult& r) {
            RCLCPP_INFO(node_->get_logger(), "[CLIENT] Result callback code=%d", static_cast<int>(r.code));
            result_code_     = r.code;
            result_received_ = true;
        };

        // --- Feedback (원한다면 사용) ------------------------------------
        opt.feedback_callback = [](auto, auto){};   // 현재 미사용

                // 4) Goal 비동기 전송 -------------------------------------------------
        client_->async_send_goal(goal_msg, opt);

        // 초기화 -----------------------------------------------------------
        goal_handle_.reset();
        result_received_ = false;
        return NodeStatus::RUNNING;
    }

    //---------------------------------------------------------------------
    //  BT onRunning ------------------------------------------------------
    //---------------------------------------------------------------------
    NodeStatus onRunning() override
    {
        rclcpp::spin_some(node_);
        if (!rclcpp::ok()) return NodeStatus::FAILURE;

        //---------------------------------------------------------------
        //  위험도 < 4  & 아직 결과 안왔으면  cancel 요청
        //---------------------------------------------------------------
        if (!result_received_ && risklevel_ < 4.0 && goal_handle_) {
            client_->async_cancel_goal(goal_handle_);
            RCLCPP_INFO(node_->get_logger(), "[CLIENT] 위험도 %.2f < 4 → cancel 요청", risklevel_);
            return NodeStatus::RUNNING;     // cancel 결과 기다림
        }

        //---------------------------------------------------------------
        //  결과 대기 / 처리
        //---------------------------------------------------------------
        if (!result_received_) return NodeStatus::RUNNING;

                switch (result_code_)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
            case rclcpp_action::ResultCode::CANCELED:   // cancel 성공도 BT SUCCESS
                return NodeStatus::SUCCESS;
            case rclcpp_action::ResultCode::ABORTED:
                return NodeStatus::FAILURE;            // 서버가 명시적 실패 → FAILURE
            case rclcpp_action::ResultCode::UNKNOWN:    // EXECUTING 등 미완료 → 계속 대기
                return NodeStatus::RUNNING;
            default:
                return NodeStatus::RUNNING;
        }
    }

    //---------------------------------------------------------------------
    //  BT onHalted -------------------------------------------------------
    //---------------------------------------------------------------------
    void onHalted() override
    {
        goal_handle_.reset();
        result_received_ = false;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;
    rclcpp_action::Client<WarningModeAction>::SharedPtr        client_;

    //-----------------------------------------------------------------
    //  상태 변수
    //-----------------------------------------------------------------
    double                                    risklevel_      = std::numeric_limits<double>::infinity();
    bool                                      result_received_{};
    rclcpp_action::ResultCode                 result_code_{};
    GoalHandleWarningMode::SharedPtr          goal_handle_{}; // ADDED : GoalHandle 저장
};
