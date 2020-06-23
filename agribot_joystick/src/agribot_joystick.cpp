#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <string>
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/MultiArrayLayout.h"

using namespace std;

class JoyTeleop {
 public:
  JoyTeleop();

 private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
  void updateParameters();
  void timerCallback(const ros::TimerEvent& e);
  void publishZeroMessage();
  void set_ServoPoses(float theta);

  double lin_scale, ang_scale, Tur_lin_scale, Tur_ang_scale, srv_scale;
  int deadmanButton, lin_vel, ang_vel, srv_pos, turboButton, stopButton,
      servoButton;
  bool canMove;
  ros::Subscriber joySub;
  ros::Publisher twistPub;
  std_msgs::Int16MultiArray SP_msg;
  ros::Publisher ServoPose;
  ros::NodeHandle nh;
  ros::Timer timeout;
  // float Srv_Pose = 0.0;
};

JoyTeleop::JoyTeleop() {
  joySub = nh.subscribe("/joy", 10, &JoyTeleop::joyCallback, this);
  twistPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  ServoPose = nh.advertise<std_msgs::Int16MultiArray>("ServoPose", 100);

  updateParameters();
}

void JoyTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
  // process and publish
  geometry_msgs::Twist twistMsg;

  // check deadman switch
  bool switchActive = msg->buttons[deadmanButton];
  bool enable_turbo_button = msg->buttons[turboButton];
  bool stopActive = msg->buttons[stopButton];
  bool activeSrv = msg->buttons[servoButton];

  if (!stopActive) {
    if (switchActive) {
      twistMsg.linear.x = lin_scale * msg->axes[lin_vel];
      twistMsg.angular.z = ang_scale * msg->axes[ang_vel] * M_PI / 2;
      canMove = true;
      twistPub.publish(twistMsg);
    } else if (enable_turbo_button) {
      twistMsg.linear.x = Tur_lin_scale * msg->axes[lin_vel];
      twistMsg.angular.z = Tur_ang_scale * msg->axes[ang_vel] * M_PI / 2;
      canMove = true;
      twistPub.publish(twistMsg);
    } else if (activeSrv) {
      twistMsg.linear.y = 1;
      twistMsg.linear.x = lin_scale * msg->axes[lin_vel];
      twistMsg.angular.z = ang_scale * msg->axes[ang_vel] * M_PI / 2;
      canMove = true;
      twistPub.publish(twistMsg);
    } else if (canMove == true) {
      for (int i = 0; i < 5; ++i) {
        publishZeroMessage();
      }
      canMove = false;
    }
  } else {
    set_ServoPoses(900);  // stop
    for (int i = 0; i < 5; ++i) {
      publishZeroMessage();
    }
    canMove = false;
  }
  // canMove = false;
}

void JoyTeleop::set_ServoPoses(float theta) {
  SP_msg.data.clear();
  SP_msg.data.push_back((int16_t)theta);
  SP_msg.data.push_back((int16_t)theta);
  ServoPose.publish(SP_msg);
}

void JoyTeleop::updateParameters() {
  nh.param("/agribot_joystick/Tur_lin_scale", Tur_lin_scale, 2.0);
  nh.param("/agribot_joystick/Tur_ang_scale", Tur_ang_scale, 1.0);

  nh.param("/agribot_joystick/lin_scale", lin_scale, 0.2);
  nh.param("/agribot_joystick/ang_scale", ang_scale, 0.5);
  nh.param("/agribot_joystick/srv_scale", srv_scale, 90.0);

  nh.param("/agribot_joystick/axis_angular", ang_vel, 2);
  nh.param("/agribot_joystick/axis_linear", lin_vel, 1);
  nh.param("/agribot_joystick/axis_servo", srv_pos, 0);

  nh.param("/agribot_joystick/axis_turbo", turboButton, 4);
  nh.param("/agribot_joystick/axis_deadman", deadmanButton, 5);
  nh.param("/agribot_joystick/axis_servo", servoButton, 6);
  nh.param("/agribot_joystick/axis_stop", stopButton, 7);

  // ROS_INFO("%f, %f, %f, %f, %f, %d, %d, %d, %d, %d, %d, %d",
  // Tur_lin_scale,Tur_ang_scale,lin_scale,ang_scale,srv_scale,ang_vel,lin_vel,srv_pos,turboButton,deadmanButton,servoButton,stopButton);
}

void JoyTeleop::timerCallback(const ros::TimerEvent& e) {
  publishZeroMessage();
}

void JoyTeleop::publishZeroMessage() {
  geometry_msgs::Twist msg;
  msg.linear.x = 0;
  msg.angular.z = 0;
  twistPub.publish(msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "agribot_joystick");
  JoyTeleop AgriJoy_node;

  ros::spin();

  return 0;
}
