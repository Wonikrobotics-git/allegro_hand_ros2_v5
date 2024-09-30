#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"

#include <iostream>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include "virtualkey_codes.h"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <vector>
#include <ament_index_cpp/get_package_prefix.hpp>
#include "virtualkey_codes.h"

using namespace std;

#define DOF_JOINTS 16


class AHKeyboard : public rclcpp::Node
{
public:
  AHKeyboard();
  void keyLoop();
  void printUsage();

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cmd_pub_;

  void savePose(const std::string& pose_file);
  std::vector<double> readFinalJointStates();
};

AHKeyboard::AHKeyboard() : Node("allegro_hand_keyboard")
{
  cmd_pub_ = this->create_publisher<std_msgs::msg::String>("allegroHand/lib_cmd", 10);
}


int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  rclcpp::shutdown();
  exit(0);
}

std::vector<double> AHKeyboard::readFinalJointStates()
{
  // 입력 받은 숫자에 해당하는 pose 파일 경로 생성
  std::string pkg_path = ament_index_cpp::get_package_prefix("allegro_hand_controllers");
  std::string file_path = pkg_path + "/share/allegro_hand_controllers/pose/final_rviz_joints.yaml";

  // pose 파일 읽기
  YAML::Node node = YAML::LoadFile(file_path);
  std::vector<double> positions = node["position"].as<std::vector<double>>();
  return positions;
}

void AHKeyboard::savePose(const std::string& pose_file)
{
  std::vector<double> positions = readFinalJointStates();

  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "position" << YAML::Value << positions;
  out << YAML::EndMap;

  std::string pkg_path = ament_index_cpp::get_package_prefix("allegro_hand_controllers");
  std::string file_path = pkg_path + "/share/allegro_hand_controllers/pose/" + pose_file;

  std::ofstream fout(file_path);
  fout << out.c_str();
  fout.close();
  RCLCPP_INFO(this->get_logger(), "Pose saved to %s", pose_file.c_str());
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AHKeyboard>();

  signal(SIGINT, quit);

  node->keyLoop();
  rclcpp::shutdown();

  return 0;
}

void AHKeyboard::printUsage() {
  
  std::cout << std::endl;
  std::cout << " -----------------------------------------------------------------------------" << std::endl;
  std::cout << "  Use the keyboard to send Allegro Hand grasp & motion commands" << std::endl;
  std::cout << " -----------------------------------------------------------------------------" << std::endl;

  std::cout << "\tHome Pose:\t\t\t'H'" << std::endl;
  std::cout << "\tPinch (index+thumb):\t\t'P'" << std::endl;
  std::cout << "\tPinch (middle+thumb):\t\t'M'" << std::endl;
  std::cout << "\tGrasp (3 fingers):\t\t'G'" << std::endl;
  std::cout << "\tGrasp (4 fingers):\t\t'F'" << std::endl;
  std::cout << "\tGrasp (envelop):\t\t'E'" << std::endl;
  std::cout << "\tGravity compensation:\t\t'Z'" << std::endl;
  std::cout << "\tFingertip sensor reset:\t\t'R'" << std::endl;
  std::cout << "\tMotors Off (free motion):\t'O'" << std::endl;

  //std::cout << " -----------------------------------------------------------------------------" << std::endl;
  //std::cout << "  MOVE IT\t(Need to install moveit package)" << std::endl;
  //std::cout << " -----------------------------------------------------------------------------" << std::endl;
  //std::cout << "\tPD Control (Custom Pose) :\t'0 ~ 9'" << std::endl;
  //std::cout << "\tSave Latest Moveit Pose:\t'Space + 0 ~ 9'" << std::endl;

  //std::cout << " -----------------------------------------------------------------------------" << std::endl;
  //std::cout << "  OCTAVE\t(Need to install Octave)" << std::endl;
  //std::cout << " -----------------------------------------------------------------------------" << std::endl;
  //std::cout << "\tSave Data & Plot:\t\t'S'" << std::endl;
  //std::cout << " -----------------------------------------------------------------------------" << std::endl;
  //std::cout << "  Note: Unless elsewhere implemented, these keyboard commands only work with " << std::endl;
  //std::cout << "  the 'allegro_hand_core_grasp' and 'allegro_hand_core_grasp_slp' packages." << std::endl;
  std::cout << "  Subscriber code for reading these messages is included in '~core_template'." << std::endl;
  std::cout << " -----------------------------------------------------------------------------\n" << std::endl;

}

#define HANDLE_KEYCODE(keycode, pose_num) \
  case keycode: \
    if (!space_pressed) { \
      ss << "pdControl" << pose_num; \
      dirty = true; \
    } else { \
      RCLCPP_DEBUG(this->get_logger(), #keycode "_key: Save Pose " #pose_num); \
      savePose("pose" #pose_num ".yaml"); \
    } \
    break;


void AHKeyboard::keyLoop()
{
  char c;
  bool dirty=false;
  bool space_pressed = false;

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  sleep(2);
  printUsage();

  for(;;)
  {
    std_msgs::msg::String msg;
    std::stringstream ss;

    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    RCLCPP_DEBUG(this->get_logger(), "value: 0x%02X", c);
    switch(c)
    {

      HANDLE_KEYCODE(KEYCODE_0, 0)
      HANDLE_KEYCODE(KEYCODE_1, 1)
      HANDLE_KEYCODE(KEYCODE_2, 2)
      HANDLE_KEYCODE(KEYCODE_3, 3)
      HANDLE_KEYCODE(KEYCODE_4, 4)
      HANDLE_KEYCODE(KEYCODE_5, 5)
      HANDLE_KEYCODE(KEYCODE_6, 6)
      HANDLE_KEYCODE(KEYCODE_7, 7)
      HANDLE_KEYCODE(KEYCODE_8, 8)
      HANDLE_KEYCODE(KEYCODE_9, 9)

      case VK_SPACE:
        space_pressed = true;
        break;
      case KEYCODE_h:
        RCLCPP_DEBUG(this->get_logger(), "h_key: Home");
        ss << "home3";
        dirty = true;
        break;
      case KEYCODE_R:
        RCLCPP_DEBUG(this->get_logger(), "h_key: finger tip reset");
        ss << "sensor";
        dirty = true;
        break;
        
      case KEYCODE_H:
        RCLCPP_DEBUG(this->get_logger(), "h_key: Home2");
        ss << "home2";
        dirty = true;
        break;  

      case KEYCODE_s:
        RCLCPP_DEBUG(this->get_logger(), "s_key: save");
        ss << "save";
        dirty = true;
      break;

      case KEYCODE_S:
        RCLCPP_DEBUG(this->get_logger(), "h_key: sphere2");
        ss << "sphere2";
        dirty = true;
      break;

      case KEYCODE_t:
        RCLCPP_DEBUG(this->get_logger(), "t_key: home3");
        ss << "home3";
        dirty = true;
      break;

      case KEYCODE_r:
        RCLCPP_DEBUG(this->get_logger(), "r_key: fingertip reset");
        ss << "sensor";
        dirty = true;
        break;
      case KEYCODE_g:
        RCLCPP_DEBUG(this->get_logger(), "h_key: Home");
        ss << "grasp_3";
        dirty = true;
        break;
      case KEYCODE_f:
        RCLCPP_DEBUG(this->get_logger(), "h_key: Home");
        ss << "grasp_4";
        dirty = true;
        break;
      case KEYCODE_p:
        RCLCPP_DEBUG(this->get_logger(), "h_key: Home");
        ss << "pinch_it";
        dirty = true;
        break;
      case KEYCODE_m:
        RCLCPP_DEBUG(this->get_logger(), "h_key: Home");
        ss << "pinch_mt";
        dirty = true;
        break;
      case KEYCODE_e:
        RCLCPP_DEBUG(this->get_logger(), "h_key: Home");
        ss << "envelop";
        dirty = true;
        break;
      case KEYCODE_z:
        RCLCPP_DEBUG(this->get_logger(), "h_key: Home");
        ss << "gravcomp";
        dirty = true;
        break;
      case KEYCODE_o:
        RCLCPP_DEBUG(this->get_logger(), "h_key: Home");
        ss << "off";
        dirty = true;
        break;

      case KEYCODE_slash:
      case KEYCORD_question:
        printUsage();
        break;
    }

    if(c >= KEYCODE_0 && c <= KEYCODE_9) {
      space_pressed = false;
    } else if (c == VK_SPACE) {
      space_pressed = true;
    }

    if(dirty ==true)
    {
      msg.data = ss.str();
      cmd_pub_->publish(msg);
      rclcpp::spin_some(this->get_node_base_interface());
      dirty = false;
    }
  }

  return;
}