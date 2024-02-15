
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "nav2_behavior_tree/behavior_tree_engine.hpp"
#include "nav2_behavior_tree/bt_action_server.hpp"
#include "nav2_util/node_utils.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/xml_parsing.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

using namespace std::chrono_literals;

std::string load_behavior_tree_string_from_file(std::string bt_xml_filename)
{
    // Read the input BT XML from the specified file into a string
    std::ifstream xml_file(bt_xml_filename);
    if (!xml_file.good()) 
    {
        std::cerr << "Couldn't open input XML file: " << bt_xml_filename << std::endl;
        throw std::runtime_error("Couldn't open input XML file");
    }

    auto xml_string = std::string(
        std::istreambuf_iterator<char>(xml_file),
        std::istreambuf_iterator<char>());

    return xml_string;
}

class BtSimple : public rclcpp::Node
{
public:

    BtSimple()
    : rclcpp::Node("bt_simple", rclcpp::NodeOptions()
        .allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true))
    , initialized_(false)
    , bt_loop_duration_(20ms)
    {
        // ADD DEFAULT PLUGIN LIBS HERE
        const std::vector<std::string> default_plugin_libs = {
            
        };

        nav2_util::declare_parameter_if_not_declared(
            this, "plugin_lib_names", rclcpp::ParameterValue(default_plugin_libs));
        nav2_util::declare_parameter_if_not_declared(
            this, "loop", rclcpp::ParameterValue(false));
        nav2_util::declare_parameter_if_not_declared(
            this, "bt_file", rclcpp::ParameterValue("") );

        timer_ = this->create_wall_timer(
            bt_loop_duration_, std::bind(&BtSimple::tick, this));
    }

    void init()
    {
        // Load params
        loop_ = get_parameter("loop").as_bool();
        plugin_libs_ = get_parameter("plugin_lib_names").as_string_array();
        bt_file_ = get_parameter("bt_file").as_string();

        std::cout << "Loading BT from '" << bt_file_ << "'" << std::endl;   

        // Load behavior tree
        bt_ = std::make_shared<nav2_behavior_tree::BehaviorTreeEngine>(plugin_libs_);
        blackboard_ = BT::Blackboard::create();

        // Put items on the blackboard
        blackboard_->set<rclcpp::Node::SharedPtr>("node", shared_from_this());  // NOLINT
        blackboard_->set<std::chrono::milliseconds>("server_timeout", 10ms);
        blackboard_->set<std::chrono::milliseconds>("bt_loop_duration", bt_loop_duration_);
        
        // TODO: Logger

        auto xml_string = load_behavior_tree_string_from_file(bt_file_);
        
        try {
            if(conn_groot_)
            {
                conn_groot_.reset();
            }
            tree_ = bt_->createTreeFromText(xml_string, blackboard_);

            // Since the behavior tree is only known at execution time of the "run" function
            // groot's "monitor" connection will first be possible from first goal on
            // BT::PublisherZMQ conn_groot(*tree);
            conn_groot_ = std::make_shared<BT::PublisherZMQ>(tree_);

            for (auto & blackboard : tree_.blackboard_stack) {
                blackboard->set<rclcpp::Node::SharedPtr>("node", shared_from_this());
                blackboard->set<std::chrono::milliseconds>("server_timeout", 10ms);
                blackboard->set<std::chrono::milliseconds>("bt_loop_duration", bt_loop_duration_);
            }
        } catch (const std::exception & e) {
            RCLCPP_ERROR(this->get_logger(), "Exception when loading BT: %s", e.what());
            return;
        }
        
        initialized_ = true;
    }

private:

    void tick()
    {
        if(!initialized_)
        {
            std::cout << "Waiting for node to be initialized" << std::endl;
            return;
        }

        std::cout << "TICK!" << std::endl;
        BT::NodeStatus result = tree_.tickRoot();
        if(result == BT::NodeStatus::SUCCESS)
        {
            // TODO: restart if loop is enabled
        }
    }

    bool loop_;
    std::shared_ptr<nav2_behavior_tree::BehaviorTreeEngine> bt_;
    std::string bt_file_;
    std::vector<std::string> plugin_libs_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::milliseconds bt_loop_duration_;

    bool initialized_;

    // BT
    BT::Tree tree_;
    BT::Blackboard::Ptr blackboard_;
    std::shared_ptr<BT::PublisherZMQ> conn_groot_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BtSimple>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}