#include <behaviortree_cpp/basic_types.h>           // BT::Blackboard
#include "behaviortree_cpp/bt_factory.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "std_msgs/msg/float64.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <csignal>
#include "checkRiskLevel.cpp"
#include "checkRiskLevelWarning.cpp"
#include "checkRiskLevelEmergency.cpp"
#include "warningMode.cpp"
#include "shoulderShift.cpp"
#include "shoulderAlign.cpp"
#include "handBrakePark.cpp"

using namespace std::chrono_literals;
using namespace BT;
volatile sig_atomic_t g_should_exit = 0;
 
 
std::string tree_path =
    ament_index_cpp::get_package_share_directory("carla_behavior") +
    "/behavior_trees/monitor_tree.xml";

void signal_handler(int signum)
{
  std::cout << "\n[Signal] Caught signal " << signum << " → 종료 요청됨" << std::endl;
  g_should_exit = 1;
}

 
int main(int argc, char** argv)
{
    rclcpp::init(argc,argv);
    auto shared_node = std::make_shared<rclcpp::Node>("bt_shared_node");
    // We use the BehaviorTreeFactory to register our custom nodes
    BehaviorTreeFactory factory;
    std::signal(SIGINT, signal_handler);
    // factory.registerBuilder<WarningMode>(
    //     "slow_driving",
    //     [shared_node](const std::string& name, const BT::NodeConfiguration& cfg){
    //         return std::make_unique<WarningMode>(name,cfg,shared_node);
    //     }
    // );

    auto blackboard = BT::Blackboard::create();
    blackboard->set<rclcpp::Node::SharedPtr>("node", shared_node);
    
    factory.registerNodeType<CheckRiskLevel>("CheckRisk");
    factory.registerNodeType<CheckRiskLevel2>("CheckRiskWarning");
    factory.registerNodeType<WarningMode>("slow_driving");
    
    factory.registerNodeType<CheckRiskLevel3>("CheckRiskEmergency");
    factory.registerNodeType<ShoulderShift>("ShoulderShift");
    factory.registerNodeType<ShoulderAlign>("ShoulderAlign");
    factory.registerNodeType<HandBrakePark>("HandBrakePark");

    
    // Trees are created at deployment-time (i.e. at run-time, but only once at the beginning).
    // The currently supported format is XML.
    // IMPORTANT: when the object "tree" goes out of scope, all the TreeNodes are destroyed

    auto tree = factory.createTreeFromFile(tree_path, blackboard);

    BT::Groot2Publisher groot_publisher(tree, 1666);//groot 퍼블리셔

    // To "execute" a Tree you need to "tick" it.
    // The tick is propagated to the children based on the logic of the tree.
    // In this case, the entire sequence is executed, because all the children
    // of the Sequence return SUCCESS.

    
    rclcpp::Rate loop_rate(10);
    while(rclcpp::ok() && !g_should_exit) {
      rclcpp::spin_some(shared_node);
      tree.tickWhileRunning(10ms);
      loop_rate.sleep();

    }
    
  
    rclcpp::shutdown();


    
    return 0;
}