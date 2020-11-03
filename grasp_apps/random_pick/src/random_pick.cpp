#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/place_location.hpp>
#include <moveit_msgs/srv/grasp_planning.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_interface/control_ur.hpp>
#include <tf2_ros/static_transform_broadcaster.h>


#define robot_enable

using GraspPlanning = moveit_msgs::srv::GraspPlanning;
/* pick position in [x, y, z, R, P, Y]*/
static std::vector<double> pick_ = {0.0, -0.54, 0.145, 3.14, 0.0, 1.956};
/* place position in [x, y, z, R, P, Y]*/
static std::vector<double> place_ = {0.557, 0.213, 0.194, 3.14, 0.0, 1.607};//20 degree remains pi/9=0.349(add to Yaw origin:1.956)
static int i = 0;
/* 0.0,(between 3.14 and 0.194)

/* pre-pick position in joint values*/
static std::vector<double> joint_values_pick = {1.65, -1.553, -1.477, -1.577, 1.556, 0};
/* place position in joint values*/
static std::vector<double> joint_values_place = {0.385, -1.470, -1.477, -1.577, 1.556, 0};
static double vel_ = 0.5, acc_ = 0.4, vscale_ = 0.5, appr_ = 0.1;
// Debug lower vel acc
// static double vel_ = 0.3, acc_ = 0.2, vscale_ = 0.5, appr_ = 0.1;
static std::shared_ptr<URControl> robot_ = nullptr;
static rclcpp::Node::SharedPtr node_ = nullptr;
static std::shared_ptr<GraspPlanning::Response> result_ = nullptr;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // init robot control
  robot_ = std::make_shared<URControl>("robot_control",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  robot_->parseArgs();
  robot_->startLoop();
  rclcpp::sleep_for(2s);

#ifdef robot_enable
  // reset joint
  robot_->moveToJointValues(joint_values_place, vel_, acc_);
#endif

  // init random pick node
  node_ = rclcpp::Node::make_shared("random_pick");
  tf2_ros::StaticTransformBroadcaster tfb(node_);
  // create client for grasp planning
  auto client = node_->create_client<GraspPlanning>("plan_grasps");
  // wait for service
  while (!client->wait_for_service(5s)) {
    RCLCPP_INFO(node_->get_logger(), "Wait for service");
  }
  RCLCPP_INFO(node_->get_logger(), "Got service");

  while(rclcpp::ok())
  {
      // request grasp poses
      auto request = std::make_shared<GraspPlanning::Request>();
      auto result_future = client->async_send_request(request);
      RCLCPP_INFO(node_->get_logger(), "Request sent");
      // wait for response
      if (rclcpp::spin_until_future_complete(node_, result_future) !=
        rclcpp::executor::FutureReturnCode::SUCCESS)
      {
        RCLCPP_INFO(node_->get_logger(), "Wait for response Failed");
        continue;
      }
      // get response
      if (moveit_msgs::msg::MoveItErrorCodes::SUCCESS == result_future.get()->error_code.val) {
        result_ = result_future.get();
	      RCLCPP_INFO(node_->get_logger(), "Response received %d", result_->error_code.val);
      } else{
        RCLCPP_INFO(node_->get_logger(), "Response no receive");
        continue;
      };

      geometry_msgs::msg::PoseStamped p = result_->grasps[0].grasp_pose;
      // publish grasp pose
      tf2::Quaternion q(p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w);
      double roll, pitch, yaw;
      tf2::Matrix3x3 r;
      r.setRotation(q);
      r.getRPY(roll, pitch, yaw);
      RCLCPP_INFO(node_->get_logger(), "**********pick pose [position %f %f %f, quat %f %f %f %f, RPY %f %f %f]",
        p.pose.position.x, p.pose.position.y, p.pose.position.z,
        p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w,
        roll, pitch, yaw);
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header = p.header;
      tf_msg.child_frame_id = "grasp_pose";
      tf_msg.transform.translation.x = p.pose.position.x;
      tf_msg.transform.translation.y = p.pose.position.y;
      tf_msg.transform.translation.z = p.pose.position.z;
      tf_msg.transform.rotation = p.pose.orientation;
      tfb.sendTransform(tf_msg);

#ifdef robot_enable
      // pick
      robot_->moveToJointValues(joint_values_pick, vel_, acc_);
      robot_->pick(p, vel_, acc_, vscale_, appr_);
      // place
      robot_->moveToJointValues(joint_values_pick, vel_, acc_);
      place_[0] = 0.2, place_[1] = 0.3+(i%3)*0.1, place_[2] = 0.255+(i/3)*0.025;
      i++;
           
      robot_->place(place_[0], place_[1], place_[2], place_[3], place_[4], place_[5], vel_, acc_, vscale_, appr_);
      
#endif
  }

  rclcpp::shutdown();
  return 0;

}
