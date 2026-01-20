#include "cv_bridge/cv_bridge.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "image_transport/image_transport.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "plansys_interface/action/go_to_point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <opencv2/aruco.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cmath>

using GoToPoint = plansys_interface::action::GoToPoint;
using GoalHandleGoToPoint = rclcpp_action::ServerGoalHandle<GoToPoint>;

using namespace std::chrono_literals;
bool align;

rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher;

class GoToPointNode : public rclcpp::Node {
  public:
	GoToPointNode() : rclcpp::Node("GoToPointNode") {

		nav2_client_ =
			rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
				this, "navigate_to_pose");

		if (!nav2_client_->wait_for_action_server(15s)) {
			RCLCPP_WARN(get_logger(), "NavigateToPose server not ready");
			return;
		}
		auto handle_goal = [this](const rclcpp_action::GoalUUID &uuid,
								  std::shared_ptr<const GoToPoint::Goal> goal) {
			RCLCPP_INFO(this->get_logger(),
						"Received goal request: x = %.2f, y = %.2f, orz = "
						"%.2f, orw = %.2f",
						goal->goal.position.x, goal->goal.position.y,
						goal->goal.orientation.z, goal->goal.orientation.w);
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

		auto nav2_result_callback =
			[this](const rclcpp_action::ClientGoalHandle<
				   nav2_msgs::action::NavigateToPose>::WrappedResult &result) {
				auto result_msg = std::make_shared<GoToPoint::Result>();

				if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
					RCLCPP_ERROR(get_logger(), "Navigation failed:");
					result_msg->success = false;
					result_msg->detected_id = -1;
					this->goal_handle->succeed(result_msg);
				} else {
					RCLCPP_INFO(get_logger(), "Aligning");
					align = true;
				}
			};
		rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::
			SendGoalOptions send_goal_options;
		send_goal_options.result_callback = nav2_result_callback;

		auto handle_accepted =
			[this, send_goal_options](
				const std::shared_ptr<GoalHandleGoToPoint> goal_handle) {
				this->goal_handle = goal_handle;
				auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
				goal_msg.pose.header.frame_id = "map";
				goal_msg.pose.pose = goal_handle->get_goal()->goal;
				nav2_client_->async_send_goal(goal_msg, send_goal_options);
			};

		this->action_server_ = rclcpp_action::create_server<GoToPoint>(
			this, "go_to_point", handle_goal, handle_cancel, handle_accepted);
	}
	std::shared_ptr<GoalHandleGoToPoint> goal_handle;

  private:
	rclcpp_action::Server<GoToPoint>::SharedPtr action_server_;
	rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr
		nav2_client_;
};

std::shared_ptr<GoToPointNode> go_to_point_node;

void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
	if (!align) {
		return;
	}

	try {
		cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
		std::vector<int> markerIds;
		std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
		cv::Ptr<cv::aruco::DetectorParameters> parameters =
			cv::aruco::DetectorParameters::create();
		cv::Ptr<cv::aruco::Dictionary> dictionary =
			cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
		cv::aruco::detectMarkers(img, dictionary, markerCorners, markerIds,
								 parameters, rejectedCandidates);

		int x_center = 0, y_center = 0;

		if (markerIds.size() == 1) {
			for (int i = 0; i < markerCorners[0].size(); i++) {
				x_center += markerCorners[0][i].x;
				y_center += markerCorners[0][i].y;
			}
			x_center /= 4;
			y_center /= 4;

			const int x_error = 640 / 2 - x_center;

			geometry_msgs::msg::Twist twist{};

			if (std::abs(x_error) < 20) {
				align = false;
				std::cout << "CORRECTLY ALIGNED" << std::endl;
				velocity_publisher->publish(twist);
				auto result_msg = std::make_shared<GoToPoint::Result>();
				result_msg->success = true;
				result_msg->detected_id = markerIds[0];

				if (go_to_point_node->goal_handle->get_goal()->capture_img) {
					double radius = 0;
					for (int i = 0; i < 4; i++) {
						double act_dist =
							std::hypot(x_center - markerCorners[0][i].x,
									   y_center - markerCorners[0][i].y);
						std::cout << "act_dist: " << act_dist << std::endl;

						if (radius < act_dist) {
							radius = act_dist;
						}
					}
					std::cout << "Radius: " << radius << std::endl;
					std::cout << "Center : " << x_center << ", " << y_center
							  << std::endl;

					cv::Point center((int)x_center, (int)y_center);

					cv::circle(img, center, (int)radius, cv::Scalar(0, 0, 255),
							   1);
					cv::imshow("Image with circle", img);
					cv::waitKey(1);
				}

				go_to_point_node->goal_handle->succeed(result_msg);
				return;
			}

			if (x_error < 0) {
				twist.angular.z = -0.2;
			} else {
				twist.angular.z = 0.2;
			}
			velocity_publisher->publish(twist);
		}

	} catch (cv_bridge::Exception &e) {
		RCLCPP_ERROR(rclcpp::get_logger("subscriber"),
					 "cv_bridge exception: %s", e.what());
		return;
	}
}

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);

	go_to_point_node = std::make_shared<GoToPointNode>();

	rclcpp::NodeOptions options;
	auto image_listener_node =
		rclcpp::Node::make_shared("image_listener", options);
	image_listener_node->declare_parameter<std::string>("image_transport",
														"compressed");
	cv::namedWindow("Image with circle");
	cv::startWindowThread();
	image_transport::ImageTransport it(image_listener_node);
	image_transport::TransportHints hints(image_listener_node.get());

	image_transport::Subscriber sub =
		it.subscribe("camera/image", 1, image_callback, &hints);
	velocity_publisher =
		image_listener_node->create_publisher<geometry_msgs::msg::Twist>(
			"/cmd_vel", 10);

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
				"Ready to reach the point and align.");

	rclcpp::executors::SingleThreadedExecutor executor;

	executor.add_node(go_to_point_node->get_node_base_interface());
	executor.add_node(image_listener_node->get_node_base_interface());

	executor.spin();

	rclcpp::shutdown();
}