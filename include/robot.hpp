#include <geometry_msgs/Twist.h>
#include <vector>
#include "ros/ros.h"
#include "traffic_sign_recognition/sign.h"

class robot {
 private:
    int count = 0; //Broj pronađenih znakova

 public:
    bool flag;  
    float type;  // Vrsta znaka
    double area;  // Veličina znaka

    void signCallback(traffic_sign_recognition::sign msg);

    void command(geometry_msgs::Twist &velocity,
        ros::Publisher &pub, ros::Rate &loop_rate);
};
