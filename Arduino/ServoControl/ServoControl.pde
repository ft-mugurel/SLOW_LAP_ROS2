#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>

int RPWM = 3;
int LPWM = 6;

int L_EN = 8;
int R_EN = 7;

ros::NodeHandle  nh;

Servo servo;

void servo_cb( const std_msgs::UInt16& cmd_msg){
  servo.write(cmd_msg.data); 
}

void motor_cb( const std_msgs::UInt16& cmd_msg){
  analogWrite(RPWM, cmd_msg.data);
}

ros::Subscriber<std_msgs::UInt16> sub("servo", servo_cb);
ros::Subscriber<std_msgs::UInt16> sub2("motor", motor_cb);

void setup(){
  pinMode(5, OUTPUT);
  pinMode(7, OUTPUT);
  digitalWrite(5, LOW);
  digitalWrite(7, LOW);
  delay(1000);
  digitalWrite(R_EN, HIGH);  
  digitalWrite(L_EN, HIGH);
  nh.initNode();
  nh.subscribe(sub);

  nh.subscribe(sub2);
  servo.attach(5);
}

void loop(){
  nh.spinOnce();
  delay(1);
}
