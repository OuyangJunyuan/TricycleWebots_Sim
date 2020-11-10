#include "tricycle_class.h"
#include <unistd.h>
int main(int argc, char **argv) {
    cout<<"loard"<<endl;
    ros::init(argc, argv, "tricycle_control_node");
    TricycleWebotsCtrl robot("/cmd_vel","/move_base_simple/goal");
    while (robot.SimOnce()!= -1) {
        ros::spinOnce();
    };
    return 0;
}
