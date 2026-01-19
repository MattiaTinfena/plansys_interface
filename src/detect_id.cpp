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
bool align, finished;

void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
	if (!align) {
		return;
	}

	try {
		cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
		cv::imshow("Received Image", img);
		cv::waitKey(1);
		std::vector<int> markerIds;
		std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
		cv::Ptr<cv::aruco::DetectorParameters> parameters =
			cv::aruco::DetectorParameters::create();
		cv::Ptr<cv::aruco::Dictionary> dictionary =
			cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
		cv::aruco::detectMarkers(img, dictionary, markerCorners, markerIds,
								 parameters, rejectedCandidates);

		geometry_msgs::msg::Point message;

		int x_center = 0, y_center = 0;

		if (markerIds.size() == 1) {
			message.z = markerIds[0];

			for (int i = 0; i < markerCorners[0].size(); i++) {
				x_center += markerCorners[0][i].x;
				y_center += markerCorners[0][i].y;
			}
			x_center /= 4;
			y_center /= 4;
		}

		const int x_error = 640 / 2 - x_center;

		std::cout << "x_error: " << x_error << std::endl;

		geometry_msgs::msg::Twist twist{};

		if (std::abs(x_error) < 20) {
			finished = true;
			align = false;
			std::cout << "CORRECTLY ALIGNED" << std::endl;
			velocity_publisher->publish(twist);
			return;
		}

		if (x_error < 0) {
			twist.angular.z = -0.2;
		} else {
			twist.angular.z = 0.2;
		}
		velocity_publisher->publish(twist);
		std::cout << "SETTING VELOCITY ANGULAR Z: " << twist.angular.z;

		// for(int i = 0; i < markerIds.size(); i++){
		//   std::cout << "Marker: " << markerIds[i] << ", Corners: ";
		//   for(int j = 0; j < markerCorners.size(); j++){
		//     std::cout << markerCorners[i][j] << ",";
		//   }
		//   std::cout << std::endl;
		// }
	} catch (cv_bridge::Exception &e) {
		RCLCPP_ERROR(rclcpp::get_logger("subscriber"),
					 "cv_bridge exception: %s", e.what());
		return;
	}
}

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
		// std::cout << "constructor" << std::endl;
		go_to_point_client_ =
			rclcpp_action::create_client<GoToPoint>(this, "go_to_point");
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

		// auto id_1 = this->problem_expert_->getFunction("marker_id m1");

		// std::cout << "name:" << id_1->name << "value:" << id_1->value
		// 		  << std::endl;

		// id_1->value += 1;

		// this->problem_expert_->updateFunction(*id_1);
		// this->problem_expert_->updateFunction(plansys2::Function(
		// 	"(= (marker_id m1)" + std::to_string(100) + ")"));

		std::unordered_map<std::string, geometry_msgs::msg::Pose> goals{};

		goals["p1"] = make_pose(-6.0, -4.5, -0.9238, 0.3826);
		goals["p2"] = make_pose(-6.0, 7.5, 0.9238, 0.3826);
		goals["p3"] = make_pose(6.0, -4.5, -0.3826, 0.9238);
		goals["p4"] = make_pose(6.0, 7.5, 0.3826, 0.9238);
		goals["base"] = make_pose(0.0, 0.0, 0.0, 0.0);

		std::string wp_to_navigate = args[1];
		// std::string wp_starting_point = args[2];

		if (!goal_sent_) {
			if (!go_to_point_client_->wait_for_action_server(1s)) {
				RCLCPP_WARN(get_logger(), "NavigateToPose server not ready");
				return;
			}
			GoToPoint::Goal goal_msg{};

			goal_msg.goal = goals[wp_to_navigate];
			RCLCPP_INFO(get_logger(), "goal created");

			goal_sent_ = true;

			auto send_goal_options =
				rclcpp_action::Client<GoToPoint>::SendGoalOptions();

			send_goal_options.result_callback =
				[this](const GoalHandleGoToPoint::WrappedResult &result) {
					if (result.result->success) {
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

		// rclcpp::spin_some(nav2_node_);
	}
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

	// rclcpp::NodeOptions options;
	// auto image_listener_node =
	// 	rclcpp::Node::make_shared("image_listener", options);
	// image_listener_node->declare_parameter<std::string>("image_transport",
	// 													"compressed");
	// // cv::namedWindow("view");
	// // cv::startWindowThread();
	// image_transport::ImageTransport it(image_listener_node);
	// image_transport::TransportHints hints(image_listener_node.get());

	// image_transport::Subscriber sub =
	// 	it.subscribe("camera/image", 1, image_callback, &hints);
	// velocity_publisher =
	// 	image_listener_node->create_publisher<geometry_msgs::msg::Twist>(
	// 		"/cmd_vel", 10);

	rclcpp::executors::SingleThreadedExecutor executor;

	executor.add_node(detect_id_node->get_node_base_interface());
	// executor.add_node(image_listener_node->get_node_base_interface());

	executor.spin();

	rclcpp::shutdown();

	return 0;
}