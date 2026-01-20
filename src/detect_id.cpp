#include "cv_bridge/cv_bridge.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "image_transport/image_transport.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "opencv2/highgui.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys_interface/action/go_to_point.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/aruco.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>

rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher;
using namespace std::chrono_literals;

geometry_msgs::msg::Pose make_pose(double x, double y, double qz, double qw) {
	geometry_msgs::msg::Pose p;
	p.position.x = x;
	p.position.y = y;
	p.orientation.z = qz;
	p.orientation.w = qw;
	return p;
}

class DetectIdAction : public plansys2::ActionExecutorClient {
	using GoToPoint = plansys_interface::action::GoToPoint;
	using GoalHandleGoToPoint = rclcpp_action::ClientGoalHandle<GoToPoint>;

  public:
	DetectIdAction()
		: plansys2::ActionExecutorClient("detect_id", 500ms), goal_sent_(false),
		  progress_(0.0) {
		go_to_point_client_ =
			rclcpp_action::create_client<GoToPoint>(this, "go_to_point");
		problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
	}

  private:
	void do_work() override {

		auto args = get_arguments();
		if (args.size() == 0) {
			RCLCPP_ERROR(get_logger(), "Not enough arguments for move action");
			this->goal_sent_ = false;
			finish(false, 0.0, "Insufficient arguments");
			return;
		}

		std::unordered_map<std::string, geometry_msgs::msg::Pose> goals{};

		goals["p1"] = make_pose(-6.0, -4.5, -0.9238, 0.3826);
		goals["p2"] = make_pose(-6.0, 7.5, 0.9238, 0.3826);
		goals["p3"] = make_pose(6.0, -4.5, -0.3826, 0.9238);
		goals["p4"] = make_pose(6.0, 7.5, 0.3826, 0.9238);
		goals["base"] = make_pose(0.0, 0.0, 0.0, 0.0);

		const std::string wp_to_navigate = args[1];

		if (!goal_sent_) {
			if (!go_to_point_client_->wait_for_action_server(1s)) {
				RCLCPP_WARN(get_logger(), "NavigateToPose server not ready");
				return;
			}
			GoToPoint::Goal goal_msg{};

			goal_msg.goal = goals[wp_to_navigate];
			goal_msg.capture_img = false;
			RCLCPP_INFO(get_logger(), "goal created");

			goal_sent_ = true;

			auto send_goal_options =
				rclcpp_action::Client<GoToPoint>::SendGoalOptions();

			send_goal_options.result_callback =
				[this, wp_to_navigate](
					const GoalHandleGoToPoint::WrappedResult &result) {
					if (result.result->success) {
						std::string id_function;
						std::unordered_map<std::string, std::string>
							functions_strs{
								{"p1", "marker_id m1"},
								{"p2", "marker_id m2"},
								{"p3", "marker_id m3"},
								{"p4", "marker_id m4"},

							};

						auto id = this->problem_expert_->getFunction(
							functions_strs[wp_to_navigate]);

						id->value = result.result->detected_id;

						this->problem_expert_->updateFunction(*id);
						// this->problem_expert_->updateFunction(
						// 	plansys2::Function("(= (marker_id m1)" +
						// 					   std::to_string(100) + ")"));
						auto new_id = this->problem_expert_->getFunction(
							functions_strs[wp_to_navigate]);

						std::cout << "new name:" << new_id->name
								  << "new value:" << new_id->value << std::endl;

						std::cout << "Goal reached" << std::endl;
						finish(true, 1.0, "Id detected");
						goal_sent_ = false;

					} else {
						finish(false, 0.0, "Goal failed");
						goal_sent_ = false;
					}
				};
			RCLCPP_INFO(get_logger(), "goal options created");

			go_to_point_client_->async_send_goal(goal_msg, send_goal_options);
			RCLCPP_INFO(get_logger(), "goal sent");
		}
	}
	std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;

	rclcpp_action::Client<plansys_interface::action::GoToPoint>::SharedPtr
		go_to_point_client_;
	bool goal_sent_;
	float progress_;
};

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);

	auto detect_id_node = std::make_shared<DetectIdAction>();

	std::cout << "initialized" << std::endl;

	detect_id_node->set_parameter(
		rclcpp::Parameter("action_name", "detect_id"));

	detect_id_node->trigger_transition(
		lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
	detect_id_node->trigger_transition(
		lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
	std::cout << "transition" << std::endl;

	rclcpp::executors::SingleThreadedExecutor executor;

	executor.add_node(detect_id_node->get_node_base_interface());

	executor.spin();

	rclcpp::shutdown();

	return 0;
}