#ifndef OP3_AR_WHOLE_BODY_CTRL_MODULE_H_
#define OP3_AR_WHOLE_BODY_CTRL_MODULE_H_


#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <boost/thread.hpp>

#include <fstream>
//#include <eigen_conversions/eigen_msg.h>
//#include <eigen3/Eigen/Eigen>

#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include "robotis_controller_msgs/StatusMsg.h"

#include "robotis_math/robotis_math.h"
#include "robotis_framework_common/motion_module.h"
#include "op3_kinematics_dynamics/op3_kinematics_dynamics.h"
#include "tcp_ip.h"

const double DEG2RAD = M_PI / 180.0;

namespace adol
{

typedef struct
{
  double x, y, z;
} Position3D;

typedef struct
{
  double x, y, z, roll, pitch, yaw;
} Pose3D;

class OP3ARWholeBodyCtrlModule : public robotis_framework::MotionModule, public robotis_framework::Singleton<OP3ARWholeBodyCtrlModule>
{
public:
  OP3ARWholeBodyCtrlModule();
  virtual ~OP3ARWholeBodyCtrlModule();

  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);
  void stop();
  bool isRunning();
  void onModuleEnable();
  void onModuleDisable();

  void queueThread();

  void setBodyHeight(double body_height);
  void setArmHeadJointAngles();

  void GetDataFromUnity();

  void JointAngleCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
  void ControlOnOffCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
  void OperatorNameCallback(const std_msgs::String::ConstPtr& msg);
  void TeleopStatus();

  float map(float x, float in_min, float in_max, float out_min, float out_max);
  float constrain(float x, float out_min, float out_max);
  float ConvertJointAngle(float datas, int convert_mod, float out_min, float out_max);
  float MappingJointUnity2OP3(float joint_angle, int joint_num);

  float unity_joint_angles[9];

  boost::thread queue_thread_;
  boost::mutex angle_change_mutex_;

  const bool DEBUG;
  int control_cycle_msec_;

  double control_cycle_sec_;
  double r_arm_angle_rad_[3];
  double l_arm_angle_rad_[3];
  double head_pan_angle_rad_;
  double head_tilt_angle_rad_;
  double r_leg_angle_rad_[6];
  double l_leg_angle_rad_[6];
  double body_height_m_;
  double body_height_min;
  double body_height_max;

  double total_mass_kg_;
  double com_state_[2];
  double curr_com_x_pos_m_, prev_com_pos_x_;

  bool module_activate;
  bool teleop_on;
  bool is_file_open_;
  int teleop_status;
  int com_dummy;
  std::string person_name;
  int experiment_test_num;

  robotis_framework::DynamixelState* dyn_state_[21];
  robotis_op::OP3KinematicsDynamics* op3_kd_;
  TCPClient *client_;

  std::ofstream file;
};

}

#endif