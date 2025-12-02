

#include "plansys2_pddl_parser/Utils.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"


#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <ostream>

std::ostream& operator<<(std::ostream& os, const plansys2_msgs::msg::Plan & plan)
{
    os << "Plan:\n";
    for (const auto & item : plan.items) {
        os << "  Action: " << item.action << "\n"
           << "  Duration: " << item.duration << "\n"
           << "  Time: " << item.time << "\n";
    }
    return os;
}

std::ostream& operator<<(std::ostream& os, const plansys2_msgs::action::ExecutePlan_Result & res)
{
    os << "ExecutePlan_Result { ";
    os << "success: " << (res.success ? "true" : "false");
    os << " }";
    return os;
}



class Controller : public rclcpp::Node
{
public:
  Controller(): rclcpp::Node("controller")
  {}

  void init()
  {
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();

    action_feedback_sub_ = this->create_subscription<plansys2_msgs::msg::ActionExecutionInfo>(
        "/action_execution_info", 10,
        std::bind(&Controller::action_feedback_callback, this, std::placeholders::_1));
  }

  void plan()
  {

          auto domain = domain_expert_->getDomain();
          auto problem = problem_expert_->getProblem();
          auto plan = planner_client_->getPlan(domain, problem);

          if (!plan.has_value()) {
            std::cout << "Could not find plan to reach goal " <<
              parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
          }

          else{
          std::cout << plan.value() << std::endl;
          executor_client_->start_plan_execution(plan.value());
        }
      

  }



private:

void action_feedback_callback(const plansys2_msgs::msg::ActionExecutionInfo::SharedPtr msg)
{
    if(msg->action_full_name !=":0"){
    action_completion_map_[msg->action_full_name] = msg->completion;
    std::cout << "Action: " << msg->action_full_name
              << " | Completion: " << (msg->completion * 100.0) << "%"
              << " | Status: ";
    switch(msg->status) {
        case plansys2_msgs::msg::ActionExecutionInfo::NOT_EXECUTED:
            std::cout << "NOT_EXECUTED"; break;
        case plansys2_msgs::msg::ActionExecutionInfo::EXECUTING:
            std::cout << "EXECUTING"; break;
        case plansys2_msgs::msg::ActionExecutionInfo::SUCCEEDED:
            std::cout << "SUCCEEDED"; break;
        case plansys2_msgs::msg::ActionExecutionInfo::FAILED:
            std::cout << "FAILED"; break;
        case plansys2_msgs::msg::ActionExecutionInfo::CANCELLED:
            std::cout << "CANCELLED"; break;
        default:
            std::cout << "UNKNOWN"; break;
    }
    std::cout << std::endl;
  }

    bool all_done = true;
    for (auto & a : action_completion_map_) {
        if (a.second < 1.0) {
            all_done = false;
            break;
        }
    }

    if (all_done) {
        std::cout << "Everything done!!" << std::endl;
        rclcpp::shutdown();
    }
}
  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;
  rclcpp::Subscription<plansys2_msgs::msg::ActionExecutionInfo>::SharedPtr action_feedback_sub_;
  std::map<std::string, float> action_completion_map_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Controller>();
  node->init();
  node->plan();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
