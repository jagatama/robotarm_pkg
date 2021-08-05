#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include<armrobot_py/AngleArray.h>
#include<armrobot_py/KeysArray.h>

ros::NodeHandle  nh;

armrobot_py::AngleArray arduino_msg;
//ros::Publisher pub_angle("arduino_angle", &arduino_msg);

Servo base;
Servo shoulder;
Servo elbor;
Servo wrist1;
Servo wrist2;
Servo arm;

void servo_cb( const armrobot_py::AngleArray& angle_msg){
  arduino_msg = angle_msg;
  base.write(int(arduino_msg.th[0]));
  delay(5);
  shoulder.write(int(arduino_msg.th[1])+5);
  delay(5);
  elbor.write(int(arduino_msg.th[2])+5);
  delay(5);
  wrist1.write(int(arduino_msg.th[3])+5);
  delay(5);
  wrist2.write(int(arduino_msg.th[4]));
  delay(5);
  arm.write(int(arduino_msg.th[5]));
  delay(5);
//  pub_angle.publish(&arduino_msg);
}

ros::Subscriber<armrobot_py::AngleArray> sub("angle", servo_cb);

void setup(){
  arduino_msg.th_length = 6;
  nh.getHardware() -> setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
//  nh.advertise(pub_angle);
  init_arm();
}

void loop(){
  nh.spinOnce();
  delay(1);
}

void init_arm(){
  base.attach(3);
  base.write(90);
  delay(50);
  shoulder.attach(5);
  shoulder.write(150);
  delay(50);
  elbor.attach(6);
  elbor.write(50);
  delay(50);
  wrist1.attach(9);
  wrist1.write(50);
  delay(50);
  wrist2.attach(10);
  wrist2.write(180);
  delay(50);
  arm.attach(11);
  arm.write(100);
  delay(50);
}
