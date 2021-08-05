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
ros::Publisher pub_angle("arduino_angle", &arduino_msg);

Servo base;
Servo shoulder;
Servo elbor;
Servo wrist1;
Servo wrist2;
Servo arm;

int th[6] = {90, 150, 50, 50, 180, 100};

void servo_cb( const armrobot_py::AngleArray& angle_msg){
  for (int i = 0; i < 6; i++){
    th[i] += angle_msg.th[i];
    if (th[i] > 180){
      th[i] = 180;
    }else if (th[i] < 0){
      th[i] = 0;
    }else if (th[5] > 100){
      th[5] = 100;
    }else if (th[5] < 50){
      th[5] = 50;
    }
  }
//  arduino_msg.th = th;
  base.write(th[0]);
  shoulder.write(th[1]);
  elbor.write(th[2]);
  wrist1.write(th[3]);
  wrist2.write(th[4]);
  arm.write(th[5]);
//  pub_angle.publish(&arduino_msg);
}

ros::Subscriber<armrobot_py::AngleArray> sub("angle",  servo_cb);

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
  base.write(th[0]);
  delay(30);
  shoulder.attach(5);
  shoulder.write(th[1]);
  delay(30);
  elbor.attach(6);
  elbor.write(th[2]);
  delay(30);
  wrist1.attach(9);
  wrist1.write(th[3]);
  delay(30);
  wrist2.attach(10);
  wrist2.write(th[4]);
  delay(30);
  arm.attach(11);
  arm.write(th[5]);
  delay(30);
}
