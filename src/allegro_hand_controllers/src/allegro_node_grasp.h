#ifndef PROJECT_ALLEGRO_NODE_GRASP_HPP
#define PROJECT_ALLEGRO_NODE_GRASP_HPP


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "allegro_node.h"
// Forward class declaration.
class BHand;

// Grasping controller that uses the BHand library for commanding various
// pre-defined grasp (e.g., three-finger ping, envelop, etc...).
//
// This node is most useful when run with the keyboard node (the keyboard node
// sends the correct String to this node). A map from String command -> Grasp
// type is defined in the implementation (cpp) file.
//
// This node can also save & hold a position, but in contrast to the PD node
// you do not have any control over the controller gains.
//
// Author: Felix Duvallet
//
class AllegroNodeGrasp : public AllegroNode {
 public:
    AllegroNodeGrasp(const std::string node_name);

    ~AllegroNodeGrasp();

    void initController(const std::string &whichHand, const std::string &whichType);

    void computeDesiredTorque();

    void libCmdCallback(const std_msgs::msg::String::SharedPtr msg);

    void setJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    void envelopTorqueCallback(const std_msgs::msg::Float32::SharedPtr msg);

    void doIt(bool polling);

 protected:
    // Handles external joint command (sensor_msgs/JointState).
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_sub;

    // Handles defined grasp commands (std_msgs/String).
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr lib_cmd_sub;

    // Handles envelop torque commands (std_msgs/Float32).
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr envelop_torque_sub;

    // Initialize BHand
    BHand *pBHand = nullptr;

    double desired_position[DOF_JOINTS] = {0.0};
};

#endif //PROJECT_ALLEGRO_NODE_GRASP_HPP
