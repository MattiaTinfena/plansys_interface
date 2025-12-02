

#include "plansys2_pddl_parser/Utils.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"
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
        }
      

  }

private:

  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Controller>();

  node->init();
  node->plan();
  rclcpp::shutdown();

  return 0;
}
