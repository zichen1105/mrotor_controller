#include "mrotor_controller/mrotor_controller.hpp"

int main(int argc, char** argv){
    ros::init(argc, argv, "mrotor_controller");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");   

    dynamic_reconfigure::Server<mrotor_controller::MrotorControllerConfig> srv;
    dynamic_reconfigure::Server<mrotor_controller::MrotorControllerConfig>::CallbackType f;

    mrotorCtrl* mrotorController = new mrotorCtrl(nh, nh_private); 
    ROS_INFO_STREAM("[Ctrl] Initialized as mrotor controller");
    f = boost::bind(&mrotorCtrl::dynamicReconfigureCb, mrotorController, _1, _2);
    srv.setCallback(f);

    ros::spin();
    return 0;
}