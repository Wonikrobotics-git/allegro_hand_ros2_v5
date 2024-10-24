#include "allegro_node_grasp.h"
#include "candrv/candrv.h"
#include "bhand/BHand.h"
#include "allegro_hand_driver/AllegroHandDrv.h"
#include <iostream>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

int flag = 0;

// The only topic specific to the 'grasp' controller is the envelop torque.
const std::string ENVELOP_TORQUE_TOPIC = "allegroHand/envelop_torque";
std::string pkg1_path;
std::string data_path;

// Define a map from string (received message) to eMotionType (Bhand controller grasp).
std::map<std::string, eMotionType> bhand_grasps = {
        {"home",     eMotionType_HOME},   // home position  
        {"grasp_3",  eMotionType_GRASP_3},  // grasp with 3 fingers
        {"grasp_4",  eMotionType_GRASP_4},  // grasp with 4 fingers
        {"pinch_it", eMotionType_PINCH_IT},  // pinch, index & thumb
        {"pinch_mt", eMotionType_PINCH_MT},  // pinch, middle & thumb
        {"envelop",  eMotionType_ENVELOP},  // envelop grasp (power-y)
        {"off",      eMotionType_NONE},  // turn joints off
        {"gravcomp", eMotionType_GRAVITY_COMP},  // gravity compensation
};


AllegroNodeGrasp::AllegroNodeGrasp(const std::string nodeName)
        : AllegroNode(nodeName),pBHand(nullptr) {
  initController(whichHand,whichType);

  joint_cmd_sub = this->create_subscription<sensor_msgs::msg::JointState>(
          DESIRED_STATE_TOPIC, 3, std::bind(&AllegroNodeGrasp::setJointCallback, this, std::placeholders::_1));
  lib_cmd_sub = this->create_subscription<std_msgs::msg::String>(
          LIB_CMD_TOPIC, 1, std::bind(&AllegroNodeGrasp::libCmdCallback, this, std::placeholders::_1));
  envelop_torque_sub = this->create_subscription<std_msgs::msg::Float32>(
          ENVELOP_TORQUE_TOPIC, 1, std::bind(&AllegroNodeGrasp::envelopTorqueCallback, this, std::placeholders::_1));
}

AllegroNodeGrasp::~AllegroNodeGrasp() {
  delete pBHand;
}

void AllegroNodeGrasp::libCmdCallback(const std_msgs::msg::String::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "CTRL: Heard: [%s]", msg->data.c_str());
  const std::string lib_cmd = msg->data.c_str();

  // Main behavior: apply the grasp directly from the map. Secondary behaviors can still be handled
  // normally (case-by-case basis), note these should *not* be in the map.
 auto itr = bhand_grasps.find(msg->data);
  if (itr != bhand_grasps.end()) {
    pBHand->SetMotionType(itr->second);
    RCLCPP_INFO(this->get_logger(), "motion type = %d", itr->second);

    if (itr->second == 0 || itr->second == 1) {
      command_place(_can_handle);
    } else {
      command_pick(_can_handle);
    }

 } else if (lib_cmd.find("pdControl") == 0) {

  // Main behavior: apply the grasp directly from the map. Secondary behaviors can still be handled
  // normally (case-by-case basis), note these should *not* be in the map.

    RCLCPP_INFO(this->get_logger(), "CTRL: Heard: [pdControl]");
    std::string num_str = lib_cmd.substr(9);

    int pose_num = std::stoi(num_str);
    RCLCPP_INFO(this->get_logger(), "PDControl Mode with pose number %d", pose_num);

    std::string pkg_path = ament_index_cpp::get_package_share_directory("allegro_hand_controllers");
    std::string file_path = pkg_path + "/pose/pose" + std::to_string(pose_num) + ".yaml";

    std::ifstream infile(file_path);
    if (!infile.good()) {
      RCLCPP_WARN(this->get_logger(), "Pose file does not exist. Please select a different command.");
      return;
    }

    YAML::Node node = YAML::LoadFile(file_path);
    std::vector<double> positions = node["position"].as<std::vector<double>>();

    for (int i = 0; i < DOF_JOINTS; i++) {
      desired_position[i] = positions[i];
    }

    pBHand->SetJointDesiredPosition(desired_position);
    pBHand->SetMotionType(eMotionType_JOINT_PD);

  } else if(lib_cmd.compare("sensor") == 0) {
    for (int i = 0; i < DOF_JOINTS; i++)
      desired_position[i] = current_position[i];

    command_pick(_can_handle);

    pBHand->SetJointDesiredPosition(desired_position);
    pBHand->SetMotionType(eMotionType_SAVE);

  }
  else {
    RCLCPP_WARN(this->get_logger(), "Unknown commanded grasp: %s.", lib_cmd.c_str());
  }
}

// Called when a desired joint position message is received
void AllegroNodeGrasp::setJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  
  mutex->lock();

  for (int i = 0; i < DOF_JOINTS; i++)
    desired_position[i] = msg->position[i];
  mutex->unlock();

  pBHand->SetJointDesiredPosition(desired_position);
  pBHand->SetMotionType(eMotionType_JOINT_PD);
  
}

// The grasp controller can set the desired envelop grasp torque by listening to
// Float32 messages on ENVELOP_TORQUE_TOPIC ("allegroHand/envelop_torque").
void AllegroNodeGrasp::envelopTorqueCallback(const std_msgs::msg::Float32::SharedPtr msg) {
  
  const double torque = msg->data;
  RCLCPP_INFO(this->get_logger(), "Setting envelop torque to %.3f.", torque);
  pBHand->SetEnvelopTorqueScalar(torque);
  
}

void AllegroNodeGrasp::computeDesiredTorque() {
  // compute control torque using Bhand library
  
  pBHand->SetJointPosition(current_position_filtered);

  // BHand lib control updated with time stamp
  pBHand->UpdateControl(static_cast<double>(frame) * ALLEGRO_CONTROL_TIME_INTERVAL);

  // Set grasping force and get FK result (if needed)
  pBHand->SetGraspingForce(f);
  pBHand->GetFKResult(x, y, z);

  // Necessary torque obtained from Bhand lib
  pBHand->GetJointTorque(desired_torque);
  
}

void AllegroNodeGrasp::initController(const std::string &whichHand, const std::string &whichType) {
  // Initialize BHand controller
  if (whichHand == "left") {
    pBHand = new BHand(eHandType_Left);
    RCLCPP_WARN(this->get_logger(), "CTRL: Left Allegro Hand controller initialized.");
  } else {
    pBHand = new BHand(eHandType_Right);
    RCLCPP_WARN(this->get_logger(), "CTRL: Right Allegro Hand controller initialized.");
  }
  
  if(whichType == "A"){
    pBHand->SetMotionType(eHardwareType_A);
    RCLCPP_WARN(this->get_logger(), "CTRL: A-Type Allegro Hand controller initialized.");
  } 
  else{
    pBHand->SetMotionType(eHardwareType_B);
    RCLCPP_WARN(this->get_logger(), "CTRL: B-Type Allegro Hand controller initialized.");
  } 


  pBHand->SetTimeInterval(ALLEGRO_CONTROL_TIME_INTERVAL);
  pBHand->SetMotionType(eMotionType_NONE);

  // sets initial desired pos at start pos for PD control
  for (int i = 0; i < DOF_JOINTS; i++)
    desired_position[i] = current_position[i];

  RCLCPP_INFO(this->get_logger(), "*************************************");
  RCLCPP_INFO(this->get_logger(), "         Grasp (BHand) Method        ");
  RCLCPP_INFO(this->get_logger(), "-------------------------------------");
  RCLCPP_INFO(this->get_logger(), "         Every command works.        ");
  RCLCPP_INFO(this->get_logger(), "*************************************");
}

void AllegroNodeGrasp::doIt(bool polling) {
  auto this_node = std::shared_ptr<AllegroNodeGrasp>(this);
  int control_cycle = (int)(1/ALLEGRO_CONTROL_TIME_INTERVAL);
  rclcpp::Rate rate(control_cycle);
  if (polling) {
    RCLCPP_INFO(this->get_logger(), " Polling = true.");
    while (rclcpp::ok()) {
      updateController();
      rclcpp::spin_some(this_node);
      rate.sleep();
    }
  } else {

    RCLCPP_INFO(this->get_logger(), "Polling = false.");
    // Timer callback
    rclcpp::TimerBase::SharedPtr timer = startTimerCallback();
    rclcpp::spin(this_node);
  }
}

int main(int argc, char **argv) {
  auto clean_argv = rclcpp::init_and_remove_ros_arguments(argc, argv); 

  bool polling = false;
  if (clean_argv.size() > 1 && clean_argv[1] == std::string("true")) {
    polling = true;
  }
  //printf("Start controller with polling = %d\n", polling);
  //bool is_sim = std::find(clean_argv.begin(), clean_argv.end(), "--sim") != clean_argv.end();
  AllegroNodeGrasp allegroNode("allegro_node_grasp");
  allegroNode.doIt(polling);
}
