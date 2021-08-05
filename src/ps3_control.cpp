#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <armrobot_py/AngleArray.h>
#include <string>

class PS3Controller{
    private:
            ros::NodeHandle nh;
            ros::Publisher pub;
            ros::Subscriber sub;
            armrobot_py::AngleArray msg;
            std::vector<short> th = {0, 0, 0, 0, 0, 0};
    public:
            PS3Controller();
            void publication(void);
            void controller_cb(const sensor_msgs::Joy& msg);
};

PS3Controller::PS3Controller(){
    pub = nh.advertise<armrobot_py::AngleArray>("/angle", 30);
    sub = nh.subscribe("/joy", 10, &PS3Controller::controller_cb, this);
}

void PS3Controller::publication(void){
    msg.th.resize(6);
    msg.th[0] = th[0];
    msg.th[1] = th[1];
    msg.th[2] = th[3];
    msg.th[3] = th[4];
    msg.th[4] = th[2];
    msg.th[5] = th[5];
    pub.publish(msg);
}

void PS3Controller::controller_cb(const sensor_msgs::Joy& msg){
    for (int i = 0; i < 6; i++){
        float val = msg.axes[i]*10;
        if (val > 7){
            th[i] = 3;
        }
        else if (val <= 7 && val >= -7) th[i] = 0;
        else{
            th[i] = -3;
        }
    }
    if (msg.buttons[4] == 1) th[2] = -3;
    else if (msg.buttons[5] == 1) th[2] = 3;
    else if (msg.buttons[1] == 1) th[5] = 3;
    else if (msg.buttons[0] == 1) th[5] = -3;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ps3_controller");
  
  PS3Controller pc;

  ros::Rate rate(20);
  while (ros::ok())
  {
      pc.publication();
      ros::spinOnce();
      rate.sleep();
  }
  
  return 0;
}
