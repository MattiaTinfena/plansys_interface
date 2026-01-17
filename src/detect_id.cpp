#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "image_transport/image_transport.hpp"
#include "opencv2/highgui.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <memory>
#include <chrono>
#include <string>
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <array>
#include <iostream>

struct Point {
  double x;
  double y;
  double z;
};

#define INT_POINTS 0

using namespace std::chrono_literals;

void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg){
  try {
    cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::imshow("Received Image", img);
    cv::waitKey(1);
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
    cv::aruco::detectMarkers(img, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
    
    geometry_msgs::msg::Point message;

    if(markerIds.size() == 1 ){
        message.z = markerIds[0];
        double x_center = 0, y_center = 0;

        for(int i = 0; i < markerCorners[0].size(); i++){
            x_center += markerCorners[0][i].x;
            y_center += markerCorners[0][i].y;
        }
        message.x = x_center/4.0;
        message.y = y_center/4.0;
        // publisher->publish(message);
    }

    for(int i = 0; i < markerIds.size(); i++){
      std::cout << "Marker: " << markerIds[i] << ", Corners: ";
      for(int j = 0; j < markerCorners.size(); j++){
        std::cout << markerCorners[i][j] << ",";
      }
      std::cout << std::endl;
    }
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("subscriber"), "cv_bridge exception: %s", e.what());
    return;
  }
}

class DetectIdAction : public plansys2::ActionExecutorClient
{
public:
  DetectIdAction()
  : plansys2::ActionExecutorClient("detect_id", 500ms), goal_sent_(false), progress_(0.0)
  {
    odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&DetectIdAction::odom_callback, this, std::placeholders::_1)
    );

    nav2_node_ = rclcpp::Node::make_shared("move_action_nav2_client");

    nav2_client_  = rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(
            nav2_node_, "navigate_through_poses");
  }

private:
  void do_work() override
  {
    auto args = get_arguments();
    if (args.size() == 0) {
      RCLCPP_ERROR(get_logger(), "Not enough arguments for move action");
      finish(false, 0.0, "Insufficient arguments");
      return;
    }

    std::unordered_map<std::string, Point> goals=
    {
        {"p1", {-6.0, -4.5, -2.357}},
        {"p2", {-6.0, 7.5, 2.357}},
        {"p3", {6.0, -4.5, -0.785}},
        {"p4", {6.0, 7.5, 0.785}},
        {"base", {0.0, 0.0, 0.0}}
    };

    std::string wp_to_navigate = args[1];
    std::string wp_starting_point = args[2];

    if (!goal_sent_) {
      if (!nav2_client_->wait_for_action_server(1s)) {
        RCLCPP_WARN(get_logger(), "NavigateToPose server not ready");
        return;
      }

      const Point starting = goals[wp_starting_point];
      const Point ending = goals[wp_to_navigate];
      std::cout << "Starting point " << "x: " << starting.x << "y: " << starting.y << std::endl;

      auto goal_msg = nav2_msgs::action::NavigateThroughPoses::Goal();

      for (int i = 0; i < INT_POINTS + 1; ++i) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position.x = ((INT_POINTS - i) * starting.x + (i + 1) * ending.x) / (INT_POINTS + 1.0);
        pose.pose.position.y = ((INT_POINTS - i) * starting.y + (i + 1) * ending.y) / (INT_POINTS + 1.0);
        pose.pose.orientation.z = ending.z;
        std::cout << "Intermediate point " << i << " x: " << pose.pose.position.x << " y: " << pose.pose.position.y << " orientation:" << pose.pose.orientation.z << std::endl;
        
        goal_msg.poses.push_back(pose);
      }

      auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions();

      auto feedback_callback = [this]
        (const std::shared_ptr<const nav2_msgs::action::NavigateThroughPoses::Feedback> feedback)
        {
          std::cout << "Number of poses remainings: " << feedback->number_of_poses_remaining << std::endl;
          if (feedback->number_of_poses_remaining <= 0) {

            finish(true, 1.0, "move completed");
          }
        };

      auto result_callback = [this]
        (const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::WrappedResult & result)
        {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
              this->goal_sent_= false;
              progress_ = 1.0;
              // rclcpp_info(get_logger(), "reached waypoint:");
              finish(true, 1.0, "move completed");

            } else {
              // rclcpp_error(get_logger(), "navigation failed");
              finish(true, 1.0, "move failed");
            }
        };
          
  
      // send_goal_options.feedback_callback = feedback_callback;
      send_goal_options.result_callback = result_callback;
        
      nav2_client_->async_send_goal(goal_msg, send_goal_options);
      goal_sent_ = true;

      start_x_ = current_x_;
      start_y_ = current_y_;
    }

    rclcpp::spin_some(nav2_node_);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
  }
  
  bool goal_sent_;
  float progress_;
  double start_x_ = 0.0, start_y_ = 0.0;
  double current_x_ = 0.0, current_y_ = 0.0;

  rclcpp::Node::SharedPtr nav2_node_;
  rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr nav2_client_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto detect_id_node = std::make_shared<DetectIdAction>();
  detect_id_node->set_parameter(rclcpp::Parameter("action_name", "detect_id"));
  detect_id_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  detect_id_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  rclcpp::NodeOptions options;
  auto image_listener_node = rclcpp::Node::make_shared("image_listener", options);
  image_listener_node->declare_parameter<std::string>("image_transport", "compressed");
  // cv::namedWindow("view");
  // cv::startWindowThread();
  image_transport::ImageTransport it(image_listener_node);
  image_transport::TransportHints hints(image_listener_node.get());

  image_transport::Subscriber sub = it.subscribe("camera/image", 1, image_callback, &hints);
  
  rclcpp::executors::SingleThreadedExecutor executor;

  executor.add_node(detect_id_node->get_node_base_interface());
  executor.add_node(image_listener_node->get_node_base_interface());

  executor.spin();

  rclcpp::shutdown();

  return 0;
}