// Common allegro node code used by any node. Each node that implements an
// AllegroNode must define the computeDesiredTorque() method.
//
// Author: Hibo (sh-yang@wonik.com)

#include "allegro_node.h"
#include "allegro_hand_driver/AllegroHandDrv.h"


std::string jointNames[DOF_JOINTS] =
        {
                "joint_0_0", "joint_1_0", "joint_2_0", "joint_3_0",
                "joint_4_0", "joint_5_0", "joint_6_0", "joint_7_0",
                "joint_8_0", "joint_9_0", "joint_10_0", "joint_11_0",
                "joint_12_0", "joint_13_0", "joint_14_0", "joint_15_0"
        };


AllegroNode::AllegroNode(const std::string nodeName, bool sim /* = false */)
  : Node(nodeName)
{
  mutex = new std::mutex();
  
  // Create arrays 16 long for each of the four joint state components
  current_joint_state.position.resize(DOF_JOINTS);
  current_joint_state.velocity.resize(DOF_JOINTS);
  current_joint_state.effort.resize(DOF_JOINTS);
  current_joint_state.name.resize(DOF_JOINTS);

  // Initialize values: joint names should match URDF, desired torque and
  // velocity are both zero.
  for (int i = 0; i < DOF_JOINTS; i++) {
    current_joint_state.name[i] = jointNames[i];
    desired_torque[i] = 0.0;
    current_velocity[i] = 0.0;
    current_position_filtered[i] = 0.0;
    current_velocity_filtered[i] = 0.0;
  }

  declare_parameter("hand_info/which_hand", "Right");
  whichHand = get_parameter("hand_info/which_hand").as_string();

  declare_parameter("hand_info/which_type", "A");
  whichType = get_parameter("hand_info/which_type").as_string();

  // Initialize CAN device
  canDevice = 0;
  if(!sim) {
    canDevice = new allegro::AllegroHandDrv();
    declare_parameter("comm/CAN_CH", "can0");
    auto can_ch = this->get_parameter("comm/CAN_CH").as_string();
    if (canDevice->init(can_ch)) {
        //rclcpp::usleep(3000);
    }
    else {
        delete canDevice;
        canDevice = 0;
    }
  }

  // Start ROS time
  tstart = get_clock()->now();
  
  // Advertise current joint state publisher and subscribe to desired joint
  // states.
  joint_state_pub = this->create_publisher<sensor_msgs::msg::JointState>(JOINT_STATE_TOPIC, 3);
  joint_cmd_sub = this->create_subscription<sensor_msgs::msg::JointState>(DESIRED_STATE_TOPIC, 1, // queue size
                                 std::bind(&AllegroNode::desiredStateCallback, this, std::placeholders::_1));

}

AllegroNode::~AllegroNode() {
  if (canDevice) delete canDevice;
  delete mutex;
  rclcpp::shutdown();
}

void AllegroNode::desiredStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  mutex->lock();
  desired_joint_state = *msg;
  mutex->unlock();
}

void AllegroNode::publishData() {
  // current position, velocity and effort (torque) published
  current_joint_state.header.stamp = tnow;
  for (int i = 0; i < DOF_JOINTS; i++) {
    current_joint_state.position[i] = current_position_filtered[i];
    current_joint_state.velocity[i] = current_velocity_filtered[i];
    current_joint_state.effort[i] = desired_torque[i];
  }
  joint_state_pub->publish(current_joint_state);
}

void AllegroNode::updateController() {

  // Calculate loop time;
  tnow = get_clock()->now();
  dt = ALLEGRO_CONTROL_TIME_INTERVAL;//1e-9 * (tnow - tstart).nanoseconds();

  // When running gazebo, sometimes the loop gets called *too* often and dt will
  // be zero. Ensure nothing bad (like divide-by-zero) happens because of this.
  if(dt <= 0) {
    RCLCPP_DEBUG_STREAM_THROTTLE(rclcpp::get_logger("allegro_node"), *get_clock(), 1000, "AllegroNode::updateController dt is zero.");
    return;
  }

  tstart = tnow;


  if (canDevice)
  {
    // try to update joint positions through CAN comm:
    lEmergencyStop = canDevice->readCANFrames();

    // check if all positions are updated:
    if (lEmergencyStop == 0 && canDevice->isJointInfoReady())
    {
      // back-up previous joint positions:
      for (int i = 0; i < DOF_JOINTS; i++) {
        previous_position[i] = current_position[i];
        previous_position_filtered[i] = current_position_filtered[i];
        previous_velocity[i] = current_velocity[i];
      }

      // update joint positions:
      canDevice->getJointInfo(current_position);

      //printf("current_position[12] = %lf, current_position[13] = %lf\n", current_position[12],current_position[13]);
      // low-pass filtering:
      for (int i = 0; i < DOF_JOINTS; i++) {
        current_position_filtered[i] = current_position[i];
        current_velocity[i] =
                (current_position[i] - previous_position[i]) / dt;
        current_velocity_filtered[i] =  current_velocity[i];;
      }

      OperatingMode = 0;

      if (OperatingMode == 0) {
        if ((fingertip_sensor[0] + fingertip_sensor[1] + fingertip_sensor[3]) > 200)
          f[0] = f[1] = f[2] = 2.0f;
        else
          f[0] = f[1] = f[2] = 1.0f;
      }


      // calculate control torque:
      computeDesiredTorque();

      // set & write torque to each joint:
      canDevice->setTorque(desired_torque);
      lEmergencyStop = canDevice->writeJointTorque();

      // reset joint position update flag:
      canDevice->resetJointInfoReady();

      // publish joint positions to ROS topic:
      publishData();

      frame++;
    }

    if(frame == 1)
    {

      if(whichHand.compare("left") == 0)
      {
        if(canDevice->RIGHT_HAND){
        RCLCPP_ERROR(this->get_logger(),"WRONG HANDEDNESS DETECTED!");
        canDevice = 0;
        return;
        }
      }
      else
      {
        if(!canDevice->RIGHT_HAND){
        RCLCPP_ERROR(this->get_logger(),"WRONG HANDEDNESS DETECTED!");
        canDevice = 0;
        return;
        }
      }

      if(whichType.compare("A") == 0)
      {
        if(!canDevice->HAND_TYPE_A){
        RCLCPP_ERROR(this->get_logger(),"WRONG TYPE DETECTED!");
        canDevice = 0;
        return;
        }
      }
      else
      {
        if(canDevice->HAND_TYPE_A){
        RCLCPP_ERROR(this->get_logger(),"WRONG TYPE DETECTED!");
        canDevice = 0;
        return;
        }
      }
      
    }

  }

  if (lEmergencyStop < 0) {
    // Stop program when Allegro Hand is switched off
    RCLCPP_ERROR(rclcpp::get_logger("allegro_node"),"Allegro Hand Node is Shutting Down! (Emergency Stop)");
    rclcpp::shutdown();
  }
}

void AllegroNode::timerCallback() {
  updateController();
}

using namespace std::chrono_literals; 

rclcpp::TimerBase::SharedPtr AllegroNode::startTimerCallback() {
  auto timer = this->create_wall_timer(1ms, std::bind(&AllegroNode::timerCallback, this));
  return timer;
}
