#include "geometry_msgs/msg/pose.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "plansys_interface/action/go_to_point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <memory>

class GoToPointNode : public rclcpp::Node {
  public:
	using GoToPoint = plansys_interface::action::GoToPoint;
	using GoalHandleGoToPoint = rclcpp_action::ServerGoalHandle<GoToPoint>;

	GoToPointNode() : rclcpp::Node("GoToPointNode") {

		auto handle_goal = [this](const rclcpp_action::GoalUUID &uuid,
								  std::shared_ptr<const GoToPoint::Goal> goal) {
			RCLCPP_INFO(this->get_logger(),
						"Received goal request with order %f",
						goal->goal.position.x);
			(void)uuid;
			return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
		};

		auto handle_cancel =
			[this](const std::shared_ptr<GoalHandleGoToPoint> goal_handle) {
				RCLCPP_INFO(this->get_logger(),
							"Received request to cancel goal");
				(void)goal_handle;
				return rclcpp_action::CancelResponse::ACCEPT;
			};

		auto handle_accepted =
			[this](const std::shared_ptr<GoalHandleGoToPoint> goal_handle) {
				// this needs to return quickly to avoid blocking the executor,
				// so we declare a lambda function to be called inside a new
				// thread
				auto execute_in_thread = [this, goal_handle]() {
					return this->execute(goal_handle);
				};
				std::thread{execute_in_thread}.detach();
			};

		this->action_server_ = rclcpp_action::create_server<GoToPoint>(
			this, "go_to_point", handle_goal, handle_cancel, handle_accepted);

		nav2_node_ = rclcpp::Node::make_shared("move_action_nav2_client");

		nav2_client_ = rclcpp_action::create_client<
			nav2_msgs::action::NavigateThroughPoses>(nav2_node_,
													 "navigate_through_poses");
	}

  private:
	void execute(const std::shared_ptr<GoalHandleGoToPoint> goal_handle) {
		RCLCPP_INFO(this->get_logger(), "Executing GoToPoint action");

		auto result = std::make_shared<GoToPoint::Result>();
		result->success = true;

		goal_handle->succeed(result);
	}

	rclcpp_action::Server<GoToPoint>::SharedPtr action_server_;
	rclcpp::Node::SharedPtr nav2_node_;
	rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr
		nav2_client_;
};

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);

	auto node = std::make_shared<GoToPointNode>();

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
				"Ready to reach the point and align.");

	rclcpp::spin(node);
	rclcpp::shutdown();
}