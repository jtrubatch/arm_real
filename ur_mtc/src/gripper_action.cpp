#include <chrono>
#include <functional>
#include <memory>
#include <thread>

#include "control_msgs/action/gripper_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


class GripperAction : public rclcpp::Node 
{
public:
    using Gripper = control_msgs::action::GripperCommand;
    using ServerGoalHandle = rclcpp_action::ServerGoalHandle<Gripper>;
    using ClientGoalHandle = rclcpp_action::ClientGoalHandle<Gripper>;

    explicit GripperAction(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) 
    : Node("GripperAction", options)
    {
        using namespace std::placeholders;

        this->as_ = rclcpp_action::create_server<control_msgs::action::GripperCommand>(
            this,
            "gripper_controller/intermediate_cmd",
            std::bind(&GripperAction::handle_goal, this, _1, _2),
            std::bind(&GripperAction::handle_cancel, this, _1),
            std::bind(&GripperAction::handle_accepted, this, _1));
        
        this->ac_ = rclcpp_action::create_client<Gripper>(
            this,
            "gripper_controller/gripper_cmd");
    }

    // CLIENT
    void send_goal(const Gripper::Goal goal)
    {
        using namespace std::placeholders;
        
        //auto goal_options = rclcpp_action::Client<Gripper>::SendGoalOptions();
        //goal_options.result_callback = std::bind(&GripperAction::result_callback, this, _1);
    }
private:
    rclcpp_action::Server<Gripper>::SharedPtr as_;
    rclcpp_action::Client<Gripper>::SharedPtr ac_;
    bool goal_achieved = false;
    // Server
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Gripper::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request");
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<ServerGoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<ServerGoalHandle> goal_handle)
    {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&GripperAction::execute, this, _1), goal_handle}.detach();
    }
    void execute(const std::shared_ptr<ServerGoalHandle> server_handle)
    {   
        RCLCPP_INFO(this->get_logger(), "Passing Translated Goal");
        const auto goal = server_handle->get_goal();
        auto feedback = std::make_shared<Gripper::Feedback>();
        auto result = std::make_shared<Gripper::Result>();
        // Goal < 0.0 Close Gripper
        auto goal_msg = Gripper::Goal();
        if(goal->command.position < 0.0){
            goal_msg.command.position = 5.0;
        } else {
            goal_msg.command.position = 60.0;
        }
        
        // CLIENT SEND
        auto goal_handle_future = this->ac_->async_send_goal(goal_msg);
        server_handle->succeed(result);
    }
    
    // CLIENT
}; // END GRIPPERACTION CLASS

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto gripper_action = std::make_shared<GripperAction>();

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(gripper_action);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}