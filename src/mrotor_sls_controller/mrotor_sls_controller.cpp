#include "mrotor_sls_controller/mrotor_sls_controller.hpp"
#include "mrotor_sls_controller/nonlinear_attitude_control.h"
#include "mrotor_sls_controller/StabController.h"
#include "mrotor_sls_controller/TracController.h"
#include "mrotor_sls_controller/QSFGeometricController.h"

mrotorCtrl::mrotorCtrl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private): nh_(nh), nh_private_(nh_private) {
    /* Retrieve vehicle total numbers & id */
    nh_private_.param<int>("mav_num", mav_num_, 1);
    nh_private_.param<int>("mav_id", mav_id_, 1);
    
    /* Subscribers */
    mav_state_sub_ = nh_.subscribe<mavros_msgs::State> ("mavros/state", 10, &mrotorCtrl::mavstateCb, this, ros::TransportHints().tcpNoDelay()); 
    gazebo_link_state_sub_ = nh_.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 1000, &mrotorCtrl::gazeboLinkStateCb, this, ros::TransportHints().tcpNoDelay());
    switch(mav_id_) {
        case 1:
            vicon_sub_ = nh_.subscribe<geometry_msgs::TransformStamped> ("/vicon/px4vision_1/px4vision_1", 1000, &mrotorCtrl::viconCb, this, ros::TransportHints().tcpNoDelay()); 
            break;
        case 2:
            vicon_sub_ = nh_.subscribe<geometry_msgs::TransformStamped> ("/vicon/px4vision_2/px4vision_2", 1000, &mrotorCtrl::viconCb, this, ros::TransportHints().tcpNoDelay()); 
            break;
        default: 
            break;
    }

    /* Publishers */
    target_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped> ("mavros/setpoint_position/local", 10);
    target_attitude_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget> ("mavros/setpoint_raw/attitude", 10);
    target_attitude_debug_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget> ("mrotor_controller/setpoint_raw/attitude", 10);
    system_status_pub_ = nh_.advertise<mavros_msgs::CompanionProcessStatus>("mavros/companion_process/status", 1);


    /* Service Clients */
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    /* Timer */

    statusloop_timer_ = nh_.createTimer(ros::Duration(1), &mrotorCtrl::statusloopCb, this);

    /* Offboard Rate */
    ros::Rate rate(20.0);

    /* Variables */
    double attctrl_tau;
    
    /* Retrieve Parameters */
    // booleans
    nh_private_.param<bool>("ctrl_enabled", ctrl_enabled_, false);
    nh_private_.param<bool>("rate_ctrl_enabled", rate_ctrl_enabled_, true);
    nh_private_.param<bool>("sim_enabled", sim_enabled_, true);
    nh_private_.param<bool>("finite_diff_enabled", finite_diff_enabled_, true);
    // drone physical constants
    nh_private_.param<double>("mav_mass", mav_mass_, 1.56);    
    nh_private_.param<double>("max_acc", max_fb_acc_, 9.0);
    // drone Yaw
    nh_private_.param<double>("yaw_heading", mavYaw_, 0.0);
    // attitude controller
    nh_private_.param<double>("attctrl_tau", attctrl_tau, 0.3);
    // throttle normalization
    nh_private_.param<double>("max_thrust_force", max_thrust_force_, 31.894746920044025);
    nh_private_.param<double>("normalized_thrust_constant", norm_thrust_const_, 0.05);
    nh_private_.param<double>("normalized_thrust_offset",norm_thrust_offset_, 0.0); // -0.0335
    // Controller Gains
    nh_private_.param<double>("Kp_x", Kpos_x_, 8.0);
    nh_private_.param<double>("Kp_y", Kpos_y_, 8.0);
    nh_private_.param<double>("Kp_z", Kpos_z_, 10.0);
    nh_private_.param<double>("Kv_x", Kvel_x_, 1.5);
    nh_private_.param<double>("Kv_y", Kvel_y_, 1.5);
    nh_private_.param<double>("Kv_z", Kvel_z_, 3.3);    
    nh_private_.param<double>("Ka_x", Kpos_x_, 0.0);
    nh_private_.param<double>("Ka_y", Kpos_y_, 0.0);
    nh_private_.param<double>("Ka_z", Kpos_z_, 0.0);
    nh_private_.param<double>("Kj_x", Kvel_x_, 0.0);
    nh_private_.param<double>("Kj_y", Kvel_y_, 0.0);
    nh_private_.param<double>("Kj_z", Kvel_z_, 0.0); 
    // Reference
    nh_private_.param<double>("c_x", c_x_, 0.0);
    nh_private_.param<double>("c_y", c_y_, 0.0);
    nh_private_.param<double>("c_z", c_z_, 0.0);    
    nh_private_.param<double>("r_x", r_x_, 0.0);
    nh_private_.param<double>("r_y", r_y_, 0.0);    
    nh_private_.param<double>("r_z", r_z_, 0.0);  
    nh_private_.param<double>("fr_x", fr_x_, 0.0);
    nh_private_.param<double>("fr_y", fr_y_, 0.0);    
    nh_private_.param<double>("fr_z", fr_z_, 0.0);      
    nh_private_.param<double>("ph_x", ph_x_, 0.0);
    nh_private_.param<double>("ph_y", ph_y_, 0.0);    
    nh_private_.param<double>("ph_z", ph_z_, 0.0);     
    // Initial Positions
    nh_private_.param<double>("pos_x_0", pos_x_0_, 0.0);
    nh_private_.param<double>("pos_y_0", pos_y_0_, 0.0);
    nh_private_.param<double>("pos_z_0", pos_z_0_, 1.0);     
    // tolerance
    nh_private_.param<double>("tracking_exit_min_error", tracking_exit_min_error_, 0.5); 

    /* Send some set-points before starting*/
    // for(int i = 100; ros::ok() && i > 0; --i){
    //     pubTargetPose(0, 0, 1);
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    /* Initialize Vectors */
    mavPos_ << 0.0, 0.0, 0.0;
    mavVel_ << 0.0, 0.0, 0.0;
    Kpos_ << -Kpos_x_, -Kpos_y_, -Kpos_z_;
    Kvel_ << -Kvel_x_, -Kvel_y_, -Kvel_z_;   
    targetPos_ << pos_x_0_, pos_y_0_, pos_z_0_;  // Initial Position
    targetVel_ << 0.0, 0.0, 0.0;
    targetJerk_ = Eigen::Vector3d::Zero(); // Not used so just set it to zero

    controller_ = std::make_shared<NonlinearAttitudeControl>(attctrl_tau);
    // ROS_INFO_STREAM(controller_);

    /* Initialization Successful Message */
    ROS_INFO_STREAM("[mrotorCtrl] Initialization Complete");
    init_complete_ = true;
    gazebo_last_called_ = ros::Time::now();
    vicon_last_called_ = ros::Time::now();
}


mrotorCtrl::~mrotorCtrl(){
    // Destructor 
}


void mrotorCtrl::mavstateCb(const mavros_msgs::State::ConstPtr& msg){
    mav_state_ = *msg;
}


void mrotorCtrl::gazeboLinkStateCb(const gazebo_msgs::LinkStates::ConstPtr& msg){
    // ROS_INFO_STREAM("mav gazebo");
    
    /* Match links on the first call*/
    if(!gazebo_link_name_matched_ && init_complete_){
        ROS_INFO("[gazeboLinkStateCb] Matching Gazebo Links");
        int n_name = sizeof(gazebo_link_name_)/sizeof(*gazebo_link_name_); 
        ROS_INFO_STREAM("[gazeboLinkStateCb] n_name=" << n_name);
        int n_link = mav_num_*8+2; 
        ROS_INFO_STREAM("[gazeboLinkStateCb] n_link=" << n_link);
        int temp_index[n_name];
        for(int i=0; i<n_link; i++){
            for(int j=0; j<n_name; j++){
                if(msg->name[i] == gazebo_link_name_[j]){
                    temp_index[j] = i;
                    
                };
            }
        }
        drone_link_index_ = temp_index[mav_id_-1];
        gazebo_link_name_matched_ = true; ROS_INFO_STREAM("drone_link_index_=" << drone_link_index_);
        ROS_INFO("[gazeboLinkStateCb] Matching Complete");
    }

    if(gazebo_link_name_matched_) {
        /* Read Gazebo Link States*/
        if(!finite_diff_enabled_) {
            mavPos_ = toEigen(msg -> pose[drone_link_index_].position);
            mavVel_ = toEigen(msg -> twist[drone_link_index_].linear);
        }

        else {
            diff_t_ = ros::Time::now().toSec() - gazebo_last_called_.toSec(); 
            gazebo_last_called_ = ros::Time::now();
            mavPos_ = toEigen(msg -> pose[drone_link_index_].position);
            if(diff_t_ > 0) {
                mavVel_ = (mavPos_ - mavPos_prev_) / diff_t_;
            }
            mavPos_prev_ = mavPos_;
        }

        mavAtt_(0) = msg -> pose[drone_link_index_].orientation.w;
        mavAtt_(1) = msg -> pose[drone_link_index_].orientation.x;
        mavAtt_(2) = msg -> pose[drone_link_index_].orientation.y;
        mavAtt_(3) = msg -> pose[drone_link_index_].orientation.z;    
        // mavRate_ = toEigen(msg -> twist[drone_link_index_].angular);
        
        /* Publish Control Commands*/
        exeControl();
    }
}

void mrotorCtrl::viconCb(const geometry_msgs::TransformStamped::ConstPtr& msg) {
    // ROS_INFO_STREAM("Vicon Mrotor Cb");
    diff_t_ = ros::Time::now().toSec() - vicon_last_called_.toSec(); 
    vicon_last_called_ = ros::Time::now();
    mavPos_ = toEigen(msg -> transform.translation);
    if(diff_t_ > 0) {
        mavVel_ = (mavPos_ - mavPos_prev_) / diff_t_;
    }
    mavPos_prev_ = mavPos_;
    mavAtt_(0) = msg -> transform.rotation.w;
    mavAtt_(1) = msg -> transform.rotation.x;
    mavAtt_(2) = msg -> transform.rotation.y;
    mavAtt_(3) = msg -> transform.rotation.z;    
    
    /* Publish Control Commands*/
    exeControl();
}

void mrotorCtrl::dynamicReconfigureCb(mrotor_sls_controller::MrotorSlsControllerConfig &config, uint32_t level) {
    /* Switches */
    if(ctrl_enabled_ != config.ctrl_enabled) {
        ctrl_enabled_ = config.ctrl_enabled;
        ROS_INFO("Reconfigure request : ctrl_enabled = %s ", ctrl_enabled_ ? "true" : "false");
    } 

    else if(rate_ctrl_enabled_ != config.rate_ctrl_enabled) {
        rate_ctrl_enabled_ = config.rate_ctrl_enabled;
        ROS_INFO("Reconfigure request : rate_ctrl_enabled = %s ", rate_ctrl_enabled_ ? "true" : "false");
    } 

    else if(traj_tracking_enabled_ != config.traj_tracking_enabled) {
        traj_tracking_enabled_ = config.traj_tracking_enabled;
        ROS_INFO("Reconfigure request : traj_tracking_enabled = %s ", traj_tracking_enabled_ ? "true" : "false");
    } 

    else if(finite_diff_enabled_ != config.finite_diff_enabled) {
        finite_diff_enabled_ = config.finite_diff_enabled;
        ROS_INFO("Reconfigure request : finite_diff_enabled = %s ", finite_diff_enabled_ ? "true" : "false");
    } 

    /* Max Acceleration*/
    else if (max_fb_acc_ != config.max_acc) {
        max_fb_acc_ = config.max_acc;
        ROS_INFO("Reconfigure request : max_acc = %.2f ", config.max_acc);
    }

    // // Causes large delay!
    // /* Attitude Control */
    // else if (attctrl_tau_ != config.attctrl_tau) {
    //     attctrl_tau_ = config.attctrl_tau;
    //     ROS_INFO("Reconfigure request : attctrl_tau = %.2f ", config.attctrl_tau);
    // } 

    /* Thrust */
    else if(norm_thrust_const_ != config.normalized_thrust_constant) {
        norm_thrust_const_ = config.normalized_thrust_constant;
        ROS_INFO("Reconfigure request : normalized_thrust_constant = %.2f ", norm_thrust_const_);
    }
    else if(norm_thrust_offset_ != config.normalized_thrust_offset) {
        norm_thrust_offset_ = config.normalized_thrust_offset;
        ROS_INFO("Reconfigure request : normalized_thrust_offset = %.2f ", norm_thrust_offset_);
    }

    /* Gains */
    // Position
    else if(Kpos_x_ != config.Kp_x) {
        Kpos_x_ = config.Kp_x;
        ROS_INFO("Reconfigure request : Kp_x = %.2f ", Kpos_x_);
    }
    else if(Kpos_y_ != config.Kp_y) {
        Kpos_y_ = config.Kp_y;
        ROS_INFO("Reconfigure request : Kp_y = %.2f ", Kpos_y_);
    }
    else if(Kpos_z_ != config.Kp_z) {
        Kpos_z_ = config.Kp_z;
        ROS_INFO("Reconfigure request : Kp_z = %.2f ", Kpos_z_);
    }
    // Velocity
    else if(Kvel_x_ != config.Kv_x) {
        Kvel_x_ = config.Kv_x;
        ROS_INFO("Reconfigure request : Kv_x = %.2f ", Kvel_x_);
    }
    else if(Kvel_y_ != config.Kv_y) {
        Kvel_y_ = config.Kv_y;
        ROS_INFO("Reconfigure request : Kv_y = %.2f ", Kvel_y_);
    }
    else if(Kvel_z_ != config.Kv_z) {
        Kvel_z_ = config.Kv_z;
        ROS_INFO("Reconfigure request : Kv_z = %.2f ", Kvel_z_);
    }

    /* References */
    // center
    else if(c_x_ != config.c_x) {
        c_x_ = config.c_x;
        ROS_INFO("Reconfigure request : c_x = %.2f ", c_x_);
    }
    else if(c_x_ != config.c_y) {
        c_y_ = config.c_y;
        ROS_INFO("Reconfigure request : c_y = %.2f ", c_y_);
    }
    else if(c_z_ != config.c_z) {
        c_z_ = config.c_z;
        ROS_INFO("Reconfigure request : c_z = %.2f ", c_z_);
    }
    // radium
    else if(r_x_ != config.r_x) {
        r_x_ = config.r_x;
        ROS_INFO("Reconfigure request : r_x = %.2f ", r_x_);
    }
    else if(r_y_ != config.r_y) {
        r_y_ = config.r_y;
        ROS_INFO("Reconfigure request : r_y = %.2f ", r_y_);
    }
    else if(r_z_ != config.r_z) {
        r_z_ = config.r_z;
        ROS_INFO("Reconfigure request : r_z = %.2f ", r_z_);
    }
    // frequency
    else if(fr_x_ != config.fr_x) {
        fr_x_ = config.fr_x;
        ROS_INFO("Reconfigure request : fr_x = %.2f ", fr_x_);
    }
    else if(fr_y_ != config.fr_y) {
        fr_y_ = config.fr_y;
        ROS_INFO("Reconfigure request : fr_y = %.2f ", fr_y_);
    }
    else if(fr_z_ != config.fr_z) {
        fr_z_ = config.fr_z;
        ROS_INFO("Reconfigure request : fr_z = %.2f ", fr_z_);
    }
    // phase shift
    else if(ph_x_ != config.ph_x) {
        ph_x_ = config.ph_x;
        ROS_INFO("Reconfigure request : ph_x = %.2f ", ph_x_);
    }
    else if(ph_y_ != config.ph_y) {
        ph_y_ = config.ph_y;
        ROS_INFO("Reconfigure request : ph_y = %.2f ", ph_y_);
    }
    else if(ph_z_ != config.ph_z) {
        ph_z_ = config.ph_z;
        ROS_INFO("Reconfigure request : ph_z = %.2f ", ph_z_);
    }

    Kpos_ << -Kpos_x_, -Kpos_y_, -Kpos_z_;
    Kvel_ << -Kvel_x_, -Kvel_y_, -Kvel_z_;  
}

void mrotorCtrl::statusloopCb(const ros::TimerEvent &event) {
    // ROS_INFO_STREAM(diff_t_);
    // printf("diff_t = %.4f", diff_t_);
    if (sim_enabled_) {
        // Enable OFFBoard mode and arm automatically
        // This will only run if the vehicle is simulated
        mavros_msgs::SetMode offb_set_mode;
        arm_cmd_.request.value = true;
        offb_set_mode.request.custom_mode = "OFFBOARD";

        if (mav_state_.mode != "OFFBOARD" && (ros::Time::now() - last_request_ > ros::Duration(5.0))) {
            if (set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request_ = ros::Time::now();
        } 

        else {
            if (!mav_state_.armed && (ros::Time::now() - last_request_ > ros::Duration(5.0))) {
                if (arming_client_.call(arm_cmd_) && arm_cmd_.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request_ = ros::Time::now();
            }
        }
    }
    pubSystemStatus();
}


void mrotorCtrl::pubSystemStatus() {
    mavros_msgs::CompanionProcessStatus msg;

    msg.header.stamp = ros::Time::now();
    msg.component = 196;  // MAV_COMPONENT_ID_AVOIDANCE
    msg.state = (int)companion_state_;

    system_status_pub_.publish(msg);
}


void mrotorCtrl::pubTargetPose(double x, double y, double z) { // Target Pose in ENU frame
    target_pose_.pose.position.x = x;
    target_pose_.pose.position.y = y;
    target_pose_.pose.position.z = z;
    target_pose_pub_.publish(target_pose_);
}


Eigen::Vector4d mrotorCtrl::acc2quaternion(const Eigen::Vector3d &vector_acc, const double &yaw) {
    Eigen::Vector4d quat;
    Eigen::Vector3d zb_des, yb_des, xb_des, proj_xb_des;
    Eigen::Matrix3d rotmat;

    proj_xb_des << std::cos(yaw), std::sin(yaw), 0.0;

    zb_des = vector_acc / vector_acc.norm();
    yb_des = zb_des.cross(proj_xb_des) / (zb_des.cross(proj_xb_des)).norm();
    xb_des = yb_des.cross(zb_des) / (yb_des.cross(zb_des)).norm();

    rotmat << xb_des(0), yb_des(0), zb_des(0), xb_des(1), yb_des(1), zb_des(1), xb_des(2), yb_des(2), zb_des(2);
    quat = rot2Quaternion(rotmat);
    return quat;
}


Eigen::Vector3d mrotorCtrl::applyIOSFBLCtrl(const Eigen::Vector3d &target_pos, const Eigen::Vector3d &target_vel) {
    const Eigen::Vector3d pos_error = mavPos_ - target_pos;
    const Eigen::Vector3d vel_error = mavVel_ - target_vel; 

    Eigen::Vector3d a_fb = Kpos_.asDiagonal() * pos_error + Kvel_.asDiagonal() * vel_error;

    // Clip acceleration
    if (a_fb.norm() > max_fb_acc_)
    a_fb = (max_fb_acc_ / a_fb.norm()) * a_fb;

    // Acceleration reference, not used here
    const Eigen::Vector3d a_ref = Eigen::Vector3d::Zero();

    // Rotor drag compensation, not used here
    const Eigen::Vector3d a_rd = Eigen::Vector3d::Zero();

    // Calculate desired acceleration
    const Eigen::Vector3d a_des = a_fb + a_ref - a_rd - gravity_;

    return a_des;
}


void mrotorCtrl::computeBodyRateCmd(Eigen::Vector4d &bodyrate_cmd, const Eigen::Vector3d &a_des) {
    // Reference attitude
    q_des = acc2quaternion(a_des, mavYaw_);
    controller_ -> Update(mavAtt_, q_des, a_des, targetJerk_);  // Calculate BodyRate
    bodyrate_cmd.head(3) = controller_->getDesiredRate();
    double thrust_command = controller_->getDesiredThrust().z();
    bodyrate_cmd(3) = std::max(0.0, std::min(1.0, norm_thrust_const_ * thrust_command + norm_thrust_offset_));  
    //[bug]// controller_->getDesiredThrust()(3); // Calculate thrust 
}


void mrotorCtrl::pubRateCommands(const Eigen::Vector4d &cmd, const Eigen::Vector4d &target_attitude) {
    mavros_msgs::AttitudeTarget msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.body_rate.x = cmd(0);
    msg.body_rate.y = cmd(1);
    msg.body_rate.z = cmd(2);
    if(rate_ctrl_enabled_){
        msg.type_mask = 128;  // Ignore orientation messages
    }
    else {
        msg.type_mask = 1|2|4;
    }
    msg.orientation.w = target_attitude(0);
    msg.orientation.x = target_attitude(1);
    msg.orientation.y = target_attitude(2);
    msg.orientation.z = target_attitude(3);
    msg.thrust = cmd(3);
    target_attitude_pub_.publish(msg);
}

void mrotorCtrl::debugRateCommands(const Eigen::Vector4d &cmd, const Eigen::Vector4d &target_attitude) {
    mavros_msgs::AttitudeTarget msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.body_rate.x = cmd(0);
    msg.body_rate.y = cmd(1);
    msg.body_rate.z = cmd(2);
    if(rate_ctrl_enabled_){
        msg.type_mask = 128;  // Ignore orientation messages
    }
    else {
        msg.type_mask = 1|2|4;
    }
    msg.orientation.w = target_attitude(0);
    msg.orientation.x = target_attitude(1);
    msg.orientation.y = target_attitude(2);
    msg.orientation.z = target_attitude(3);
    msg.thrust = cmd(3);
    target_attitude_debug_pub_.publish(msg);
}

void mrotorCtrl::updateReference(){
    double t = ros::Time::now().toSec();
    double sp_x = c_x_ + r_x_ * std::sin(fr_x_ * t + ph_x_);
    double sp_y = c_y_ + r_y_ * std::sin(fr_y_ * t + ph_y_);
    double sp_z = c_z_ + r_z_ * std::sin(fr_z_ * t + ph_z_);
    double sp_x_dt = r_x_ * fr_x_ * std::cos(fr_x_ * t + ph_x_);
    double sp_y_dt = r_y_ * fr_y_ * std::cos(fr_y_ * t + ph_y_);
    double sp_z_dt = r_z_ * fr_z_ * std::cos(fr_z_ * t + ph_z_);

    // Entering tracking
    if(!last_tracking_state_ && traj_tracking_enabled_ && (mav_num_>1 && (std::abs(mavPos_(0)-sp_x)>0.1 || std::abs(mavPos_(1)-sp_y)>0.1))) {
        targetPos_ << pos_x_0_, pos_y_0_, pos_z_0_;  // Initial Position
        targetVel_ << 0.0, 0.0, 0.0;
        targetJerk_ = Eigen::Vector3d::Zero(); // Not used so just set it to zero  
        // ROS_INFO("1");
    }    

    // Running tracking
    else if(last_tracking_state_ && traj_tracking_enabled_) {
        targetPos_ << sp_x, sp_y, sp_z;
        targetVel_ << sp_x_dt, sp_y_dt, sp_z_dt;
        last_tracking_state_ = traj_tracking_enabled_;
        // ROS_INFO("2");
    }

    // Exiting from tracking
    else if(last_tracking_state_ && (mav_num_>1 && (std::abs(mavPos_(0)-pos_x_0_)>tracking_exit_min_error_ || std::abs(mavPos_(1)-pos_y_0_)>tracking_exit_min_error_))) {
        targetPos_ << sp_x, sp_y, sp_z;
        targetVel_ << sp_x_dt, sp_y_dt, sp_z_dt;   
        // ROS_INFO("3");     
    }

    // Initial Point
    else {
        targetPos_ << pos_x_0_, pos_y_0_, pos_z_0_;  // Initial Position
        targetVel_ << 0.0, 0.0, 0.0;
        targetJerk_ = Eigen::Vector3d::Zero(); // Not used so just set it to zero  
        last_tracking_state_ = traj_tracking_enabled_;
        // ROS_INFO("4");
    }
    
}

void mrotorCtrl::exeControl() {
    if(init_complete_){
        Eigen::Vector3d desired_acc;
        desired_acc = applyIOSFBLCtrl(targetPos_, targetVel_);
        computeBodyRateCmd(cmdBodyRate_, desired_acc);
        if(ctrl_enabled_){
            pubRateCommands(cmdBodyRate_, q_des);
        }
        else{
            pubTargetPose(pos_x_0_, pos_y_0_, pos_z_0_);
            debugRateCommands(cmdBodyRate_, q_des);
        }
        updateReference();
    }
}


/*================================================================= SLS Controller =================================================================*/

mrotorSlsCtrl::mrotorSlsCtrl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private): mrotorCtrl(nh, nh_private) {
    // ROS_INFO_STREAM(gravity_acc_);

    /* Subscribers */
    gazebo_link_state_sub_ = nh_.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 1000, &mrotorSlsCtrl::gazeboLinkStateCb, this, ros::TransportHints().tcpNoDelay());
    switch(mav_id_) {
        case 1:
            vicon_sub_ = nh_.subscribe<geometry_msgs::TransformStamped> ("/vicon/px4vision_1/px4vision_1", 1000, &mrotorSlsCtrl::viconCb, this, ros::TransportHints().tcpNoDelay()); 
            break;
        case 2:
            vicon_sub_ = nh_.subscribe<geometry_msgs::TransformStamped> ("/vicon/px4vision_2/px4vision_2", 1000, &mrotorSlsCtrl::viconCb, this, ros::TransportHints().tcpNoDelay()); 
            break;
        default: 
            break;
    }
    vicon_load_sub_ = nh_.subscribe<geometry_msgs::TransformStamped> ("/vicon/load_1/load_1", 1000, &mrotorSlsCtrl::viconLoadCb, this, ros::TransportHints().tcpNoDelay()); 

    /* Publishers */
    sls_state_pub_ = nh_.advertise<mrotor_sls_controller::SlsState> ("mrotor_sls_controller/sls_state", 10);
    
    nh_private_.param<double>("cable_length", cable_length_, 0.85);
    nh_private_.param<double>("load_mass", load_mass_, 0.25);
    nh_private_.param<double>("Kp_x", Kpos_x_, 24.0);
    nh_private_.param<double>("Kp_y", Kpos_y_, 24.0);
    nh_private_.param<double>("Kp_z", Kpos_z_, 2.0);
    nh_private_.param<double>("Kv_x", Kvel_x_, 50.0);
    nh_private_.param<double>("Kv_y", Kvel_y_, 50.0);
    nh_private_.param<double>("Kv_z", Kvel_z_, 3.0); 

    last_stage_ = ros::Time::now();
}

mrotorSlsCtrl::~mrotorSlsCtrl(){
    // Destructor 
}

void mrotorSlsCtrl::gazeboLinkStateCb(const gazebo_msgs::LinkStates::ConstPtr& msg){
    // ROS_INFO_STREAM("SLS gazebo");
    diff_t_ = ros::Time::now().toSec() - gazebo_last_called_.toSec(); 
    gazebo_last_called_ = ros::Time::now();
    /* Read Gazebo Link States*/
    // Drone
    mavPos_ = toEigen(msg -> pose[2].position);
    mavVel_ = toEigen(msg -> twist[2].linear);
    mavAtt_(0) = msg -> pose[2].orientation.w;
    mavAtt_(1) = msg -> pose[2].orientation.x;
    mavAtt_(2) = msg -> pose[2].orientation.y;
    mavAtt_(3) = msg -> pose[2].orientation.z;    
    mavRate_ = toEigen(msg -> twist[2].angular);
    // Load 
    loadPos_ = toEigen(msg -> pose[10].position);
    loadVel_ = toEigen(msg -> twist[10].linear);
    // Pendulum
    computePendState();


    if(qsf_geo_ctrl_enabled_) {
        sls_state_.header.stamp = ros::Time::now();
        sls_state_.sls_geo_state[0] = mavPos_(0);
        sls_state_.sls_geo_state[1] = -mavPos_(1);
        sls_state_.sls_geo_state[2] = -mavPos_(2);
        sls_state_.sls_geo_state[3] = pendAngle_q_(0);
        sls_state_.sls_geo_state[4] = -pendAngle_q_(1);
        sls_state_.sls_geo_state[5] = -pendAngle_q_(2);
        sls_state_.sls_geo_state[6] = mavVel_(0);
        sls_state_.sls_geo_state[7] = -mavVel_(1);
        sls_state_.sls_geo_state[8] = -mavVel_(2); 
        sls_state_.sls_geo_state[9] = pendRate_q_(0);
        sls_state_.sls_geo_state[10] = -pendRate_q_(1);
        sls_state_.sls_geo_state[11] = -pendRate_q_(2);

    }

    else if(!qsf_geo_ctrl_enabled_) {
        sls_state_.header.stamp = ros::Time::now();
        sls_state_.sls_state[0] = mavPos_(0);
        sls_state_.sls_state[1] = -mavPos_(1);
        sls_state_.sls_state[2] = -mavPos_(2);
        sls_state_.sls_state[3] = pendAngle_(0);
        sls_state_.sls_state[4] = pendAngle_(1);
        sls_state_.sls_state[5] = mavVel_(0);
        sls_state_.sls_state[6] = -mavVel_(1);
        sls_state_.sls_state[7] = -mavVel_(2);   
        sls_state_.sls_state[8] = pendRate_(0);
        sls_state_.sls_state[9] = pendRate_(1); 
        sls_state_pub_.publish(sls_state_);        
    }

    
    /* Publish Control Commands*/
    exeSlsControl();

    // exeMission();
}

void mrotorSlsCtrl::computePendState() {
    if(!qsf_geo_ctrl_enabled_)  {
        double Lx = loadPos_(0) - mavPos_(0) ;
        double Ly = -loadPos_(1) - (-mavPos_(1));
        double Lz = -loadPos_(2) - (-mavPos_(2));
        pendAngle_ = ToPendAngles(Lx, Ly, -Lz); // in the paper the definition of n3 are opposite to the Z axis of gazebo
        if(std::isnan(pendAngle_(0)) || std::isnan(pendAngle_(1))) {
            pendAngle_ = pendAngle_last_;
        }
        else {
            pendAngle_last_ = pendAngle_;
        }
        // gamma beta
        pendRate_(1) = ((loadVel_(0)) - (mavVel_(0))) / (cable_length_ * std::cos(pendAngle_(1)));
        // gamma alpha
        pendRate_(0) =  ((-loadVel_(1)) - (-mavVel_(1)) - std::sin(pendAngle_(0))*std::sin(pendAngle_(1))*pendRate_(1)*cable_length_)/(-std::cos(pendAngle_(0))*std::cos(pendAngle_(1))*cable_length_);
        if(std::isnan(pendRate_(0)) || std::isnan(pendRate_(1))) {
            pendRate_ = pendRate_last_;
        }
        else {
            pendRate_last_ = pendRate_;
        }
    }

    else if(qsf_geo_ctrl_enabled_) {
        pendAngle_q_ = loadPos_ - mavPos_;
        pendRate_q_ = (pendAngle_q_ - pendAngle_q_prev_) / diff_t_;
        pendAngle_q_prev_ = pendAngle_q_;
    }

}

Eigen::Vector2d mrotorSlsCtrl::ToPendAngles(double Lx,double Ly,double Lz) { //x=base.x
    Eigen::Vector2d  angle;

    /* beta (y-axis rotation) */ 
    double sinbeta = Lx / cable_length_;
    double cosbeta = Lz / (cable_length_*std::cos(angle(0)));
    angle(1) = std::asin(sinbeta);

    /* alpha (x-axis rotation) */ 
    double cosa_cosb = Lz / cable_length_;
    double sina_cosb = Ly / -cable_length_;
    angle(0) = std::asin(sina_cosb/std::cos(angle(1)));

    return angle;
}

Eigen::Vector3d mrotorSlsCtrl::applyQuasiSlsCtrl(double target_pose_x, double target_pose_y, double target_pose_z) {
    // double K[12] = {2.2361,    3.1623, 3.1623,   3.0777,    8.4827,    8.4827,  0,    18.7962,    18.7962,  0,    17.4399,    17.4399};
    double K[12] = {Kvel_z_,    3.1623, 3.1623,   Kpos_z_,    8.4827,    8.4827,  0,    18.7962,    18.7962,  0,    17.4399,    17.4399};
    // double K[12] = {17.4399, 17.4399, 3.077, 18.7962, 18.7962, 2.2361, 8.4827, 8.4827, 0,  3.1623, 3.1623, 0};
    // double K[12] = {3.0777,    5.4399,    5.4399,    2.2361,    9.7962,    9.7962,         0,    8.4827,    8.4827,         0,    3.1623,    3.1623};
    double target_force_ned[3];
    double target_pose_ned[3] = {target_pose_x, target_pose_y, target_pose_z};
    double param[4] = {mav_mass_, load_mass_, cable_length_, gravity_acc_};
    double sls_state_array[10];
    for(int i=0; i<10;i++){
       sls_state_array[i] = sls_state_.sls_state[i];
    } 
    StabController(sls_state_array, K, param, target_pose_ned, target_force_ned);
    Eigen::Vector3d a_des;
    a_des(0) = target_force_ned[0] / mav_mass_;
    a_des(1) = -target_force_ned[1] / mav_mass_;
    a_des(2) = -target_force_ned[2] / mav_mass_;
    
    return a_des;
}

Eigen::Vector3d mrotorSlsCtrl::applyQuasiSlsTrackingCtrl() {
    double K[12] = {Kvel_z_,    3.1623, 3.1623,   Kpos_z_,    8.4827,    8.4827,  0,    18.7962,    18.7962,  0,    17.4399,    17.4399};
    // double K[12] = {2.2361,    3.1623, 3.1623,   3.0777,    8.4827,    8.4827,  0,    18.7962,    18.7962,  0,    17.4399,    17.4399};
    // double K[12] = {17.4399, 17.4399, 3.077, 18.7962, 18.7962, 2.2361, 8.4827, 8.4827, 0,  3.1623, 3.1623, 0};
    // double K[12] = {3.0777,    5.4399,    5.4399,    2.2361,    9.7962,    9.7962,         0,    8.4827,    8.4827,         0,    3.1623,    3.1623};
    double target_force_ned[3];
    double param[4] = {mav_mass_, load_mass_, cable_length_, gravity_acc_};
    double sls_state_array[10];
    for(int i=0; i<10;i++){
       sls_state_array[i] = sls_state_.sls_state[i];
    } 
    const double t = ros::Time::now().toSec() - last_stage_.toSec();
    TracController(sls_state_array, K, param, t, target_force_ned);
    Eigen::Vector3d a_des;
    a_des(0) = target_force_ned[0] / mav_mass_;
    a_des(1) = -target_force_ned[1] / mav_mass_;
    a_des(2) = -target_force_ned[2] / mav_mass_;
    
    return a_des;
}

Eigen::Vector3d mrotorSlsCtrl::applyQuasiSlsGeoCtrl() {
    double target_force_ned[3];
    double K[10] = {Kpos_x_, Kvel_x_, Kacc_x_, Kjer_x_, Kpos_y_, Kvel_y_, Kacc_y_, Kacc_z_, Kpos_z_, Kvel_z_};
    double param[4] = {load_mass_, mav_mass_, cable_length_, gravity_acc_};
    double ref[12] = {r_x_, c_x_, fr_x_, ph_x_, r_y_, c_y_, fr_y_, ph_y_, r_z_, c_z_, fr_z_, ph_z_,};
    double sls_state_array[12];
    for(int i=0; i<12;i++){
       sls_state_array[i] = sls_state_.sls_geo_state[i];
    }
    const double t = ros::Time::now().toSec() - last_stage_.toSec();
    QSFGeometricController(sls_state_array, K, param, ref, t, target_force_ned);
    Eigen::Vector3d a_des;
    a_des(0) = target_force_ned[0] / mav_mass_;
    a_des(1) = -target_force_ned[1] / mav_mass_;
    a_des(2) = -target_force_ned[2] / mav_mass_;
    
    return a_des;
}

void mrotorSlsCtrl::exeMission(void) {
    if(init_complete_){
        Eigen::Vector3d desired_acc;
        switch(stage){
            case 0:
                desired_acc = applyQuasiSlsCtrl(0, 0, -0.4);
                if(ros::Time::now().toSec() - last_stage_.toSec() >= 10){
                    stage += 1;
                    last_stage_ = ros::Time::now();
                    ROS_INFO_STREAM("case 0 completed!");
                }
                break;
            case 1:
                desired_acc = applyQuasiSlsCtrl(0, 0, -0.4);
                if(ros::Time::now().toSec() - last_stage_.toSec() >= 10){
                    stage += 1;
                    last_stage_ = ros::Time::now();
                    ROS_INFO_STREAM("case 1 completed!");
                }
                break;
            case 2:
                desired_acc = applyQuasiSlsCtrl(1.0, 0.5, -0.6);
                if(ros::Time::now().toSec() - last_stage_.toSec() >= 10){
                    stage += 1;
                    last_stage_ = ros::Time::now();
                    ROS_INFO_STREAM("case 2 completed!");
                }
                break;
            case 3:
                desired_acc = applyQuasiSlsCtrl(-1.0, 0.0, -0.3);
                if(ros::Time::now().toSec() - last_stage_.toSec() >= 10){
                    stage += 1;
                    last_stage_ = ros::Time::now();
                    ROS_INFO_STREAM("case 3 completed!");
                }
                break;
            case 4:
                desired_acc = applyQuasiSlsCtrl(0.0, 0.0, -0.3);
                if(ros::Time::now().toSec() - last_stage_.toSec() >= 10){
                    stage += 1;
                    last_stage_ = ros::Time::now();
                    ROS_INFO_STREAM("case 4 completed!");
                }
                break;
            case 5:
                desired_acc = applyQuasiSlsTrackingCtrl();
                if(ros::Time::now().toSec() - last_stage_.toSec() >= 32){
                    stage += 1;
                    last_stage_ = ros::Time::now();
                    ROS_INFO_STREAM("case 5 completed!");
                }
                break;
            default: 
                desired_acc = applyQuasiSlsCtrl(0, 0, -0.4);
                break;
        }
        
        computeBodyRateCmd(cmdBodyRate_, desired_acc);
        pubRateCommands(cmdBodyRate_, q_des);
    }
}

void mrotorSlsCtrl::exeSlsControl() {
    if(init_complete_){
        
        Eigen::Vector3d desired_acc;
        if(qsf_geo_ctrl_enabled_) {
            
        }

        else if(!traj_tracking_enabled_) {
            desired_acc = applyQuasiSlsCtrl(0, 0, -0.4);
        }

        else {
            desired_acc = applyQuasiSlsTrackingCtrl();
        }
        
        computeBodyRateCmd(cmdBodyRate_, desired_acc);

        if(ctrl_enabled_){
            pubRateCommands(cmdBodyRate_, q_des);
        }
        else{
            pubTargetPose(pos_x_0_, pos_y_0_, pos_z_0_);
            debugRateCommands(cmdBodyRate_, q_des);
        }
    }
}

void mrotorSlsCtrl::viconCb(const geometry_msgs::TransformStamped::ConstPtr& msg){
    diff_t_ = ros::Time::now().toSec() - vicon_last_called_.toSec(); 
    vicon_last_called_ = ros::Time::now();
    mavPos_ = toEigen(msg -> transform.translation);
    if(diff_t_ > 0) {
        mavVel_ = (mavPos_ - mavPos_prev_) / diff_t_;
        loadVel_ = (loadPos_ - loadPos_prev_) / diff_t_;
    }
    mavPos_prev_ = mavPos_;
    loadPos_prev_ = loadPos_;

    // MAV Attitude
    mavAtt_(0) = msg -> transform.rotation.w;
    mavAtt_(1) = msg -> transform.rotation.x;
    mavAtt_(2) = msg -> transform.rotation.y;
    mavAtt_(3) = msg -> transform.rotation.z;    

    sls_state_.header.stamp = ros::Time::now();
    sls_state_.sls_state[0] = mavPos_(0);
    sls_state_.sls_state[1] = -mavPos_(1);
    sls_state_.sls_state[2] = -mavPos_(2);
    sls_state_.sls_state[3] = pendAngle_(0);
    sls_state_.sls_state[4] = pendAngle_(1);
    sls_state_.sls_state[5] = mavVel_(0);
    sls_state_.sls_state[6] = -mavVel_(1);
    sls_state_.sls_state[7] = -mavVel_(2);   
    sls_state_.sls_state[8] = pendRate_(0);
    sls_state_.sls_state[9] = pendRate_(1); 
    sls_state_pub_.publish(sls_state_);
    
    /* Publish Control Commands*/
    exeSlsControl();
}

void mrotorSlsCtrl::viconLoadCb(const geometry_msgs::TransformStamped::ConstPtr& msg){
    // Load 
    loadPos_ = toEigen(msg -> transform.translation);
}