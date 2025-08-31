#include <stdio.h>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "std_msgs/msg/string.hpp"
#include "kdl/tree.hpp"
#include "kdl/chain.hpp"
#include "kdl/frames.hpp"
#include "kdl/jntarray.hpp"
#include "kdl/chainiksolverpos_lma.hpp"
#include "kdl_parser/kdl_parser.hpp"

#include "cpp_pkg/srv/compute_leg_ik.hpp"

class LegIK : public rclcpp::Node
{
public:
  LegIK() : Node("leg_inverse_kinematics_example")
  {

    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "robot_description",
      rclcpp::QoS(rclcpp::KeepLast(1))
        .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL),
      std::bind(&LegIK::robotDescriptionCallback, this, std::placeholders::_1));

    service_ = this->create_service<cpp_pkg::srv::ComputeLegIK>(
      "/compute_leg_ik",
      std::bind(&LegIK::computeLegIKCallback, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Leg IK Service Server is ready.");
  }

private:
  
  void computeLegIKCallback(
    const std::shared_ptr<cpp_pkg::srv::ComputeLegIK::Request> request,
    std::shared_ptr<cpp_pkg::srv::ComputeLegIK::Response> response)
  {

    tree_.getChain("base_link", request->leg_tip, chain_);
    std::cout << "Chain number of joints:  " << chain_.getNrOfJoints() << std::endl;

    solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(chain_);

    double knee_angle = request->curr_knee_joint;
    double ankle_angle = request->curr_ankle_joint;
    getJointAngles(request->target_foot_pos.x, request->target_foot_pos.y, request->target_foot_pos.z, knee_angle, ankle_angle);
    response->tar_knee_joint = knee_angle;
    response->tar_ankle_joint = ankle_angle;
    response->success_flag = true;
    // RCLCPP_INFO(this->get_logger(), "Incoming request: x=%.2f, y=%.2f, z=%.2f",
    //             request->target_foot_pos.x, request->target_foot_pos.y, request->target_foot_pos.z);
    // RCLCPP_INFO(this->get_logger(), "Sending back response: knee_angle=%.2f, ankle_angle=%.2f",
    //             response->tar_knee_joint, response->tar_ankle_joint);
  }



  void robotDescriptionCallback(const std_msgs::msg::String& msg)
  {

    const std::string urdf = msg.data;
    kdl_parser::treeFromString(urdf, tree_);

    std::cout << "Number of joints: " << tree_.getNrOfJoints() << std::endl;
    std::cout << "Number of segments: " << tree_.getNrOfSegments() << std::endl;
    std::cout << "Root segment: " << tree_.getRootSegment()->first
              << std::endl;
  }


  void getJointAngles(
    const double x, const double y, const double z, double & knee_angle, double & ankle_angle)
  {

    KDL::JntArray q_init(chain_.getNrOfJoints());
    q_init(0) = knee_angle;
    q_init(1) = ankle_angle;
    const KDL::Frame p_in(KDL::Vector(x, y, z));
    KDL::JntArray q_out(chain_.getNrOfJoints());

    solver_->CartToJnt(q_init, p_in, q_out);

    knee_angle = q_out(0);
    ankle_angle = q_out(1);
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Service<cpp_pkg::srv::ComputeLegIK>::SharedPtr service_;
  KDL::Tree tree_;
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainIkSolverPos_LMA> solver_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LegIK>());
  rclcpp::shutdown();
  return 0;
}