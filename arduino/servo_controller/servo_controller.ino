#include <ros.h>
#include <std_msgs/UInt16.h>
#include <Servo.h>

#define SERVO_PIN 9

ros::NodeHandle  nh;

Servo servo;

void servo_callback( const std_msgs::UInt16& cmd_msg){
  servo.write(cmd_msg.data);  // set servo angle
  digitalWrite(13, HIGH-digitalRead(13));  // toggle the led
}

ros::Subscriber<std_msgs::UInt16> sub("servo", servo_callback );

void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);

  servo.attach(SERVO_PIN);
}

void loop()
{
  nh.spinOnce();
}
