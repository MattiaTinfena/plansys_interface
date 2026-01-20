#include "cv_bridge/cv_bridge.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "image_transport/image_transport.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "opencv2/highgui.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_pddl_parser/Utils.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
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
#include <map>
#include <memory>
#include <ostream>
#include <string>

using namespace std::chrono_literals;

std::ostream &operator<<(std::ostream &os,
						 const plansys2_msgs::msg::Plan &plan) {
	os << "Plan:\n";
	for (const auto &item : plan.items) {
		os << "  Action: " << item.action << "\n"
		   << "  Duration: " << item.duration << "\n"
		   << "  Time: " << item.time << "\n";
	}
	return os;
}

std::ostream &operator<<(std::ostream &os,
						 const plansys2_msgs::action::ExecutePlan_Result &res) {
	os << "ExecutePlan_Result { ";
	os << "success: " << (res.success ? "true" : "false");
	os << " }";
	return os;
}

class ChangeStateAction : public plansys2::ActionExecutorClient {
  public:
	ChangeStateAction()
		: plansys2::ActionExecutorClient("change_to_acquire_state", 500ms) {
		domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
		planner_client_ = std::make_shared<plansys2::PlannerClient>();
		problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
		executor_client_ = std::make_shared<plansys2::ExecutorClient>();
	}

  private:
	void do_work() override {
		auto args = get_arguments();
		if (args.size() == 0) {
			RCLCPP_ERROR(get_logger(),
						 "Not enough arguments for change state action");
			finish(false, 0.0, "Insufficient arguments");
			return;
		}
		finish(true, 1.0, "State changed");

		rclcpp::sleep_for(1s);

		this->problem_expert_->removePredicate(
			plansys2::Predicate("(is_first m3)"));
		this->problem_expert_->removePredicate(
			plansys2::Predicate("(is_next m4 m3)"));
		this->problem_expert_->removePredicate(
			plansys2::Predicate("(is_next m1 m4)"));
		this->problem_expert_->removePredicate(
			plansys2::Predicate("(is_next m2 m1)"));

		std::map<int, int> marker_ids;

		for (int i = 0; i < 4; ++i) {
			int marker_name = i + 1;
			auto id_function = this->problem_expert_->getFunction(
				"marker_id m" + std::to_string(marker_name));
			marker_ids[id_function->value] = marker_name;
		}

		int prev_marker = -1;
		for (const auto &[id, marker_num] : marker_ids) {
			std::cout << "id: " << id << " marker_num: " << marker_num
					  << std::endl;
			if (prev_marker == -1) {
				this->problem_expert_->addPredicate(plansys2::Predicate(
					"(is_first m" + std::to_string(marker_num) + ")"));
				prev_marker = marker_num;

				continue;
			}
			this->problem_expert_->addPredicate(
				plansys2::Predicate("(is_next m" + std::to_string(marker_num) +
									" m" + std::to_string(prev_marker) + ")"));
			prev_marker = marker_num;
		}

		rclcpp::sleep_for(1s);

		executor_client_->cancel_plan_execution();
		auto domain = domain_expert_->getDomain();
		auto problem = problem_expert_->getProblem();

		this->problem_expert_->setGoal(
			plansys2::Goal("(and(photo_taken m1)(photo_taken m2)(photo_taken "
						   "m3)(photo_taken m4))"));

		problem = problem_expert_->getProblem();

		std::cout << "Problem:" << problem << std::endl;

		auto plan = planner_client_->getPlan(domain, problem);

		if (!plan.has_value()) {
			std::cout << "Could not find plan to reach goal "
					  << parser::pddl::toString(problem_expert_->getGoal())
					  << std::endl;
		}

		else {
			executor_client_->start_plan_execution(plan.value());
		}
	}

	std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
	std::shared_ptr<plansys2::PlannerClient> planner_client_;
	std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
	std::shared_ptr<plansys2::ExecutorClient> executor_client_;
	rclcpp::Subscription<plansys2_msgs::msg::ActionExecutionInfo>::SharedPtr
		action_feedback_sub_;
};

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);

	auto change_state_node = std::make_shared<ChangeStateAction>();
	change_state_node->set_parameter(
		rclcpp::Parameter("action_name", "change_to_acquire_state"));
	change_state_node->trigger_transition(
		lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
	change_state_node->trigger_transition(
		lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

	rclcpp::executors::SingleThreadedExecutor executor;

	executor.add_node(change_state_node->get_node_base_interface());

	executor.spin();

	rclcpp::shutdown();

	return 0;
}