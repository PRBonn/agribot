/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ROS-Embedded Controller - Servo Joint in Agribot Platfrom     %
% by: Alireza Ahmadi                                            %
% University of Bonn- MSc Robotics & Geodetic Engineering       %
% Alireza.Ahmadi@uni-bonn.de                                    %
% https://github.com/PRBonn                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Empty.h>

bool ROS_en  = true;

ros::NodeHandle  nh;

float ranges[2];

const int led_pin = 13;
// These constants won't change. They're used to give names to the pins used:
const int analogIn_right = A1;  // Analog input pin that the potentiometer is attached to
const int analogIn_left = A0;  // Analog input pin that the potentiometer is attached to

const int PulOut_left = 5; 
const int DirOut_left = 10; 

const int PulOut_right = 6; 
const int DirOut_right = 11; 

const int ledPin =  LED_BUILTIN;// the number of the LED pin

const int power_en = 12;

int Coef = 110;

int sensorValue_left = 0;        // value read from the pot
int sensorValue_right = 0;        // value read from the pot

int outputValue_left = 0;        // value output to the PWM (analog out)
int outputValue_right = 0;        // value output to the PWM (analog out)

int center_left = 0;        // global center variable of left caster
int center_right = 0;       // global center variable of right caster 

int center_f_left = 512;        //512
int center_f_right = 495 ;       //495

int center_b_left = 510-102;        //495-102
int center_b_right = 495-102;       //495-102

int telorance = 1;
int i=0;

int PWM_value = 400;

int old_sensorValue_left = 0;
int old_sensorValue_right = 0;

bool b_right = false;
bool b_left = false;

float D2S = 51/90;

int goal_r = 0.0;
int goal_l = 0.0;

int goal_r_old = 0.0;
int goal_l_old = 0.0;

bool Servo_stop = true;

int max_step = 91;

bool set_Zero = false;

int old_ModeFlag = 0;

void messageCb( const std_msgs::Int16MultiArray& ServoPose){
  Toggle_led(-1,70);

  if(ServoPose.data[2] != old_ModeFlag){
    set_Zero = true;
    if(ServoPose.data[2] == 0){
      center_left = center_f_left;
      center_right = center_f_right;
    }else{
      center_left = center_b_left;
      center_right = center_b_right;
    }
    old_ModeFlag = ServoPose.data[2];
  }
  
  if(ServoPose.data[0] != 900){
    if(abs(ServoPose.data[0]) > max_step + telorance) ServoPose.data[0] = max_step*(ServoPose.data[0]/ServoPose.data[0]);
    goal_r = ((51*ServoPose.data[0])/90) + center_right;
    if(ServoPose.data[1] == 0)set_Zero = true;
    Servo_stop = false;
  }else{
    Servo_stop = true;
  }
  if(ServoPose.data[1] != 900){
    if(abs(ServoPose.data[1]) > max_step + telorance) ServoPose.data[1] = max_step*(ServoPose.data[1]/ServoPose.data[1]);
    goal_l = ((51*ServoPose.data[1])/90) + center_left;
    if(ServoPose.data[1] == 0)set_Zero = true;
    Servo_stop = false;
  }else{
    Servo_stop = true;
  }


//  if(ServoPose.data[0] != 900 && ServoPose.data[1] != 900){
//    if(ServoPose.data[0] == 0 || ServoPose.data[1] == 0 || ServoPose.data[2] != old_ModeFlag){
//      set_Zero = true;
//      if(ServoPose.data[2] == 0){
//        center_left = center_f_left;
//        center_right = center_f_right;
//      }else{
//        center_left = center_b_left;
//        center_right = center_b_right;
//      }
//      old_ModeFlag = ServoPose.data[2];
//    }
//  
//    if(abs(ServoPose.data[0]) > max_step + telorance) ServoPose.data[0] = max_step*(ServoPose.data[0]/ServoPose.data[0]);
//    goal_r = D2S * ServoPose.data[0] + center_right;
//
//    if(abs(ServoPose.data[1]) > max_step + telorance) ServoPose.data[1] = max_step*(ServoPose.data[1]/ServoPose.data[1]);
//    goal_l = D2S * ServoPose.data[1] + center_left;
//    
//    Servo_stop = false;
//  }else{
//    Servo_stop = true;
//  }
}

ros::Subscriber<std_msgs::Int16MultiArray> sub("ServoPose", messageCb );

void setup() {
  delay(100);
  if(!ROS_en){
    Toggle_led(2,150);
    // initialize serial communications at 9600 bps:
    Serial.begin(9600);
  }else{
    Toggle_led(3,150);
    nh.initNode();
    nh.subscribe(sub);
  }
  
  setPwmFrequency(5, (int)(62500/2));
  setPwmFrequency(6, (int)(62500/2));
  pinMode(DirOut_right, OUTPUT);
  pinMode(DirOut_left, OUTPUT);

  pinMode(led_pin,OUTPUT);
  
  pinMode(power_en, OUTPUT);
  digitalWrite(power_en, HIGH);
  
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  digitalWrite(A2, HIGH);
  digitalWrite(A3, HIGH);

  delay(100);

// set Initial potision to 0 or 180
  center_left = center_f_left;
  center_right = center_f_right;
//  center_left = center_b_left;
//  center_right = center_b_right;

  int tmp_r = (51*0)/90;
  int tmp_l = (51*0)/90;
  while(!intial_position_both(center_right+tmp_r,center_left+tmp_l)){
    if(!ROS_en){
      print_val();
    }
  }
  for(int cnt=0; cnt<20; cnt++){
      set_position_both(center_right+tmp_r-3,center_left+tmp_r-3);
      delay(20);
      set_position_both(center_right+tmp_r+1,center_left+tmp_r+1);
      delay(20);
      set_position_both(center_right+tmp_r,center_left+tmp_r);
  }
  
  goal_r = center_right;
  goal_r_old = goal_r;

  goal_l = center_left;
  goal_l_old = goal_l;

}
void loop() {
  
  if(goal_r_old != goal_r || goal_l_old != goal_l){
    goal_r_old = goal_r;
    goal_l_old = goal_l;
    set_position_both(goal_r,goal_l);
    if(goal_r == center_right && goal_l == center_left){
      for(int cnt=0; cnt<30; cnt++){
        set_position_both(center_right,center_left);
        delay(20);
      }
    }
  }

  if(set_Zero){
    while(!intial_position_both(center_right,center_left)){
      delay(1);
    }
    set_Zero = false;
  }
  
  Stop_both(goal_r,goal_l);
  if(!ROS_en){
    print_val();
  }else{
    nh.spinOnce();
  }
  delay(5);
}
