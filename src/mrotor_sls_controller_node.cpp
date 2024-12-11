#include "mrotor_sls_controller/mrotor_sls_controller.hpp"

int main(int argc, char** argv){
    ros::init(argc, argv, "mrotor_sls_controller");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");   
    bool sls_mode_enabled = false;
    nh_private.param<bool>("sls_mode_enabled", sls_mode_enabled, false);
    dynamic_reconfigure::Server<mrotor_sls_controller::MrotorSlsControllerConfig> srv;
    dynamic_reconfigure::Server<mrotor_sls_controller::MrotorSlsControllerConfig>::CallbackType f;

    if(!sls_mode_enabled) {
        mrotorCtrl* mrotorController = new mrotorCtrl(nh, nh_private); 
        ROS_INFO_STREAM("[Ctrl] Initialized as mrotor controller");
        f = boost::bind(&mrotorCtrl::dynamicReconfigureCb, mrotorController, _1, _2);
        srv.setCallback(f);
    }

    else {
        mrotorSlsCtrl* mrotorSlsController = new mrotorSlsCtrl(nh, nh_private);
        ROS_INFO_STREAM("[Ctrl] Initialized as SLS controller");
        f = boost::bind(&mrotorSlsCtrl::dynamicReconfigureCb, mrotorSlsController, _1, _2);
        srv.setCallback(f);
    }

    ros::spin();
    return 0;
}