#include <plan_manage/kino_manage.h>
#include <algorithm>


using namespace std;

void KINOManage::init(ros::NodeHandle &nh)
{
  ROS_INFO_STREAM("KINOManage::init\n");

  exec_state_ = FSM_EXEC_STATE::INIT;

  int sim_odom_type;
  nh.param("kino/sim_odom_type", sim_odom_type, 1);
  nh.param("kino/ext_noise_bound", ext_noise_bound_, 0.5);

  init_pt_ = Eigen::VectorXd::Zero(6);//pos + euler_degree
  init_yaw_pt_ = Eigen::VectorXd::Zero(6);//pos + euler_degree
  cur_Odom_ = Eigen::VectorXd::Zero(9);
  stateOdomPrevious_ = Eigen::VectorXd::Zero(9);
  stateMpc_ = Eigen::VectorXd::Zero(9);

  /*  map intial  */
  env_ptr_.reset(new OccMap);
  env_ptr_->init(nh);

  kino_solver_.initROS(nh, env_ptr_);

  goal_sub_ = nh.subscribe("/goal_topic", 1, &KINOManage::goalCallback, this);

  // publishers
  //path_pub_ = nh.advertise<nav_msgs::Path>("kino_path", 1, true);
  path_pub_ = nh.advertise<nav_msgs::Path>("kino_path", 1);
  goal_point_pub_ = nh.advertise<visualization_msgs::Marker>("goal_point", 2);

  reference_state_pub_ = nh.advertise<resilient_planner::kinoRoutePoint>("/kinoroutepoint", 1, true);
  init_yaw_state_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/inityawstate", 1, true);
  
  // subscribers

  if (sim_odom_type == 1)
  { // use the imu
    odom_sub_ = nh.subscribe("/odom_world", 1, &KINOManage::odometryCallback, this);
  }
  else
  { // use rotorS(velocity is in the body frame)
    odom_sub_ = nh.subscribe("/odom_world", 1, &KINOManage::odometryTransCallback, this);
  }

  extforce_sub_ = nh.subscribe("/forces", 1, &KINOManage::extforceCallback, this);


  target_reached_sub_ = nh.subscribe("/target_reached", 1, &KINOManage::targetReachedCallback, this);

  // /forces_estimation/thrust
  exec_timer_ = nh.createTimer(ros::Duration(0.01), &KINOManage::execFSMCallback, this);
  safety_timer_ = nh.createTimer(ros::Duration(0.05), &KINOManage::checkReplanCallback, this);
  //mpc_timer_ = nh.createTimer(ros::Duration(0.05), &KINOManage::mpcCallback, this); // 50ms
}


static string state_str[8] = {"INIT", "WAIT_TARGET", "INIT_YAW", "GENERATE_NEW_TRAJ", "EXEC_TRAJ", "REPLAN_TRAJ"};

void KINOManage::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
{
  int pre_s = int(exec_state_);
  exec_state_ = new_state;
  cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
}


void KINOManage::execFSMCallback(const ros::TimerEvent &e)
{
  // trigger is for the goal
  // have goal is for the planning
  // if the goal changes, then we should set trigger as true and update new goal

  static int fsm_num = 0;
  fsm_num++;
  if (fsm_num == 100)
  {
    cout << "78 [FSM]: state: " + state_str[int(exec_state_)] << endl;
    if (!have_odom_)
      cout << "no odom." << endl;
    if (!have_target_)
      cout << "wait for goal." << endl;
    fsm_num = 0;
  }

  switch (exec_state_)
  {
    case INIT:
    {
      //std::cout<<"-------------------Current STATE: INIT------------------------"<<std::endl;
      if (!have_odom_)
      {
        return;
      } 
      //std::cout<<"95 altitude error: "<<abs(cur_Odom_(2) - 1.2)<<std::endl;
      if (have_target_)
      {
        changeFSMExecState(WAIT_TARGET, "FSM");
      }
      else if (fsm_num == 50 && abs(cur_Odom_(2) - 1.2) > 0.02)
      {
        // init_pt_ << cur_Odom_(0), cur_Odom_(1), 1.2,  //position
        //            0, 0, 0;                          //acceleration

        init_pt_ << 0 - 0.137, 0, 1.2 + 0.048,  //position
                   0, 0, 0;                          //acceleration
        sendInitYawState(init_pt_);
        
        have_init_ = true;
      }
      break;
    }

    case WAIT_TARGET:
    {
      std::cout<<"-------------------Current STATE: WAIT_TARGET------------------------"<<std::endl;
      if (!have_target_)
      {
        consider_force_ = false;
        return;
      }
      else
      { 
        changeFSMExecState(INIT_YAW, "FSM");
        call_init_yaw_ = true;
      }
      break;
    }

    case INIT_YAW:
    {
      std::cout<<"-------------------Current STATE: INIT_YAW------------------------"<<std::endl;
      Eigen::Vector3d delta_pos = end_pt_ - cur_Odom_.segment(0, 3);
      init_yaw_ = atan2(delta_pos(1), delta_pos(0));

      std::cout<<"cur_Odom_: "<<cur_Odom_.segment(0, 3)<<std::endl;
      std::cout<<"init_yaw_: "<<init_yaw_<<std::endl;

      init_yaw_pt_ << 0 - 0.137, 0, 1.2 + 0.048,  //position
                  0, 0, 3.11; //init_yaw_
      sendInitYawState(init_yaw_pt_);
      
      // if (call_init_yaw_)
      // {
      //   Eigen::Vector3d delta_pos = end_pt_ - cur_Odom_.segment(0, 3);
      //   init_yaw_ = atan2(delta_pos(1), delta_pos(0));

      //   if (abs(cur_Odom_(8) - init_yaw_) >= 0.08)
      //   {
      //     init_yaw_pt_ << 0 - 0.137, 0, 1.2 + 0.048,  //position
      //              0, 0, init_yaw_; 
      //     sendInitYawState(init_yaw_pt_);
      //     //kino_solver_.callInitYaw(cur_Odom_, init_yaw_);
      //     //ALEX send to MPC 
      //   }
      //   call_init_yaw_ = false;
      //   break;
      // }

      // if (abs(cur_Odom_(8) - init_yaw_) < 0.08)
      // {
      //   init_yaw_pt_ << 0 - 0.137, 0, 1.2 + 0.048,  //position
      //              0, 0, init_yaw_; 

      //   sendInitYawState(init_yaw_pt_);

      //   consider_force_ = true;
      //   std::cout << "[resilient_planner] Finish init yaw ! The odom is: " << cur_Odom_(8) << ", yaw is : " << init_yaw_ << std::endl;
      //   //changeFSMExecState(GENERATE_NEW_TRAJ, "FSM");
      // }

      break;
    }

    case GENERATE_NEW_TRAJ:
    {
      std::cout<<"-------------------Current STATE: GENERATE_NEW_TRAJ------------------------"<<std::endl;
      exec_mpc_ = false;

      std::cout << "[resilient_planner] Plan_fail_count_ ：" << plan_fail_count_ << std::endl;
      if (plan_fail_count_ > 3){
        have_target_ = false;
        changeFSMExecState(WAIT_TARGET, "FSM");
        plan_fail_count_ = 0;
        break;
      }

      if (kino_solver_.getKinoPath(cur_Odom_, end_pt_, external_acc_, false))
      {
        std::cout << "[resilient_planner] Kino plan success!" << std::endl;
        have_traj_ = true;
        trigger_ = false;
        exec_mpc_ = true;
        replan_force_surpass_ = false;
        kino_solver_.getKinoTraj(kino_path_);
        //displayPath();
        sendRefState();

        plan_fail_count_ = 0;
        //changeFSMExecState(EXEC_TRAJ, "FSM");
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        plan_fail_count_ += 1;
        changeFSMExecState(GENERATE_NEW_TRAJ, "FSM");
      }

      break;
    }

    case REPLAN_TRAJ:
    {
      std::cout<<"-------------------Current STATE: REPLAN_TRAJ------------------------"<<std::endl;
      std::cout << "[resilient_planner] Replan_fail_count_ ：" << plan_fail_count_ << std::endl;
      exec_mpc_ = false;

      if (plan_fail_count_ > 3){
        have_target_ = false;
        changeFSMExecState(WAIT_TARGET, "FSM");
        plan_fail_count_ = 0;
        break;
      }

      if (kino_solver_.getKinoPath(cur_Odom_, end_pt_, external_acc_, true)){
        std::cout << "[resilient_planner] Kino replan success!" << std::endl;
        have_traj_ = true;
        trigger_ = false;
        exec_mpc_ = true;
        replan_force_surpass_ = false;
        kino_solver_.getKinoTraj(kino_path_);
        //displayPath();
        sendRefState();
        
        plan_fail_count_ = 0;
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        plan_fail_count_ += 1;
        changeFSMExecState(GENERATE_NEW_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      std::cout<<"-------------------Current STATE: EXEC_TRAJ------------------------"<<std::endl;
      // change the target
      
      // if (trigger_ && exec_mpc_)
      // {
        
      //   cout << "exec_mpc_." << exec_mpc_ << endl;
      //   changeFSMExecState(REPLAN_TRAJ, "FSM");
      // }
      
      //Eigen::Vector3d delta_pos = end_pt_ - cur_Odom_.segment(0, 3);
      //if(delta_pos.norm() < 1)
      if(target_reached)
      {
        cout << "252 EXEC target reached, move to WAIT_TARGET"<< endl;
        changeFSMExecState(WAIT_TARGET, "FSM");
      }
      break;
    }

  }
}

void KINOManage::sendInitYawState(Eigen::VectorXd init_yaw_)
{
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.pose.position.x = init_yaw_(0);
  pose_msg.pose.position.y = init_yaw_(1);
  pose_msg.pose.position.z = init_yaw_(2);
  //pose_msg.pose.position = init_yaw_.segment(0, 3);

  //From Euler to Quaterniond
  Eigen::Quaterniond q = Eigen::AngleAxisd(init_yaw_(3), Eigen::Vector3d::UnitX()) 
                  * Eigen::AngleAxisd(init_yaw_(4), Eigen::Vector3d::UnitY()) 
                  * Eigen::AngleAxisd(init_yaw_(5), Eigen::Vector3d::UnitZ());
  
  pose_msg.pose.orientation.w = q.w();
  pose_msg.pose.orientation.x = q.x();
  pose_msg.pose.orientation.y = q.y();
  pose_msg.pose.orientation.z = q.z();
  init_yaw_state_pub_.publish(pose_msg);

  //std::cout<<"265 send init_yaw_state: "<<pose_msg.pose.position<<std::endl;
}


void KINOManage::displayPath()
{
  nav_msgs::Path path_msg;
  for (const auto &it : kino_path_)
  {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = it(0);
    pose.pose.position.y = it(1);
    pose.pose.position.z = it(2);
    pose.pose.orientation.w = 1.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;

    path_msg.poses.push_back(pose);
  }

  path_msg.header.frame_id = "world";
  path_pub_.publish(path_msg);
}

void KINOManage::sendRefState()
{
  resilient_planner::kinoRoutePoint ref_kino_msgs;

  vector<double> trajectory_;
  vector<double> attitude;

  double t_pre;

  int size_ = 0;
  int  horizon = 5;
  if(kino_path_.size()>horizon)
    size_ = kino_path_.size();
  else
    size_ = kino_path_.size();
  for (int i = 0; i < size_;i++)
  {
    //check if time is in correct order
    
    for(int j=0; j < 3;j++)
      trajectory_.push_back(kino_path_[i](j));

    for(int j=3; j < 6;j++)
      attitude.push_back(kino_path_[i](j));
  }
  ref_kino_msgs.trajectory = trajectory_;
  ref_kino_msgs.attitude = attitude;
  
  reference_state_pub_.publish(ref_kino_msgs);


  // resilient_planner::ReferenceTrajectory ref_traj_msgs;

  // //Eigen::VectorXd trajectory_ = Eigen::VectorXd::Zero(13);
  // vector<double> trajectory_;
  // vector<double> t;
  // //Eigen::VectorXd input_u_ = Eigen::VectorXd::Zero(3);
  // vector<double> input_u_;

  // double t_pre;

  // int size_ = 0;
  // int  horizon = 5;
  // if(kino_path_.size()>horizon)
  //   size_ = kino_path_.size();
  // else
  //   size_ = kino_path_.size();
  // for (int i = 0; i < size_;i++)
  // {
  //   //check if time is in correct order
  //   if(i != 0 && kino_path_[i-1](13)>= kino_path_[i](13))
  //     kino_path_[i](13) = kino_path_[i](13) + kino_path_[i-1](13);
    
  //   for(int j=0; j < 13;j++)
  //     trajectory_.push_back(kino_path_[i](j));
  //   // trajectory_ << kino_path_[i](0), kino_path_[i](1), kino_path_[i](2),        //pos
  //   //                 kino_path_[i](3), kino_path_[i](4), kino_path_[i](5), kino_path_[i](6),//q
  //   //                 kino_path_[i](7), kino_path_[i](8), kino_path_[i](9),       //v
  //   //                 kino_path_[i](10), kino_path_[i](11), kino_path_[i](12);       //rate
  //   t.push_back(kino_path_[i](13));

  //   for(int j=14; j < 17;j++)
  //     input_u_.push_back(kino_path_[i](j));
  // }
  // ref_traj_msgs.seq_len = size_;
  // ref_traj_msgs.traj_name = "kino_ref_traj";
  // ref_traj_msgs.v_input = 2;
  // ref_traj_msgs.trajectory = trajectory_;
  // ref_traj_msgs.dt = t;
  // ref_traj_msgs.inputs = input_u_;
  
  // reference_state_pub_.publish(ref_traj_msgs);

  std::cout<<"ref_traj_msgs has been send!!!"<<std::endl;
}


void KINOManage::checkReplanCallback(const ros::TimerEvent &e)
{
  // step one: check the target
  // 1. the local target is or not collision free
  if (have_target_)
  {
    if (!env_ptr_->checkPosSurround(end_pt_, 1.2))
    {
      /* try to find a max distance goal around */
      bool new_goal = false;
      const double dr = 0.2, dtheta = 30, dz = 0.2;
      double new_x, new_y, new_z, max_dist = -1.0;
      Eigen::Vector3d goal;

      for (double r = dr; r <= 5 * dr + 1e-3; r += dr){
        for (double theta = -90; theta <= 270; theta += dtheta){
          for (double nz = 1.0; nz <= 1.6; nz += dz){

            new_x = end_pt_(0) + r * cos(theta);
            new_y = end_pt_(1) + r * sin(theta);
            new_z = nz;

            Eigen::Vector3d new_pt(new_x, new_y, new_z);
            if (env_ptr_->checkPosSurround(new_pt, 1.5)){
              end_pt_ = new_pt;
              have_target_ = true;
              displayGoalPoint();
              break;
            }
          }
        }
      }

      if (exec_state_ == EXEC_TRAJ){
        ROS_WARN("[checkReplan] Change goal, replan.");
        changeFSMExecState(REPLAN_TRAJ, "SAFETY");
      }else{
        have_target_ = false;
        ROS_WARN("[checkReplan] Goal near collision, stop.");
        changeFSMExecState(WAIT_TARGET, "SAFETY");
      }
    }
  }

  if (have_traj_)
  {
    for (int i = 0; i < kino_path_.size(); i+= 5)
    {  
      Eigen::Vector3d kino_temp = kino_path_[i].head(3);
      if (!env_ptr_->checkPosSurround(kino_temp, 1.2))
      {
        ROS_WARN("[checkReplan] Trajectory collides, replan.");
        changeFSMExecState(REPLAN_TRAJ, "SAFETY");
        break;
      }
    }
  }
}

void KINOManage::displayGoalPoint()
{
  visualization_msgs::Marker sphere;
  sphere.header.frame_id = "world";
  sphere.header.stamp = ros::Time::now();
  sphere.type = visualization_msgs::Marker::SPHERE;
  sphere.action = visualization_msgs::Marker::ADD;

  sphere.pose.orientation.w = 1.0;
  sphere.color.r = 0.0;
  sphere.color.g = 0.0;
  sphere.color.b = 0.0;
  sphere.color.a = 1.0;
  sphere.scale.x = 0.3;
  sphere.scale.y = 0.3;
  sphere.scale.z = 0.3;
  sphere.pose.position.x = end_pt_(0);
  sphere.pose.position.y = end_pt_(1);
  sphere.pose.position.z = end_pt_(2);

  goal_point_pub_.publish(sphere);
}

void KINOManage::extforceCallback(const geometry_msgs::WrenchStamped &msg)
{
  return;
  // the sub is mass normolized value
  if (consider_force_)
  {
    double diverse = max(max(abs(msg.wrench.force.x), abs(msg.wrench.force.y)), abs(msg.wrench.force.z));
    if (diverse <= ext_noise_bound_)
    {
      external_acc_.setZero();
      last_external_acc_ << msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z; // revise as zero
      surpass_count_ = 0;
    }
    else
    {
      external_acc_ << msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z; // revise as zero if unstale

      Eigen::Vector3d diff = last_external_acc_ - external_acc_;
      double surpass = max(max(abs(diff(0)), abs(diff(1))), abs(diff(2)));

      if (surpass > ext_noise_bound_)
      { 
        ROS_INFO_STREAM("[resilient_planner]: External forces varies too large, need to replan ! ");
        ROS_INFO_STREAM("last_external_acc_: \n"  << last_external_acc_);
        ROS_INFO_STREAM("external_acc: \n"    << external_acc_);

        surpass_count_ += 1;

        if (surpass_count_ >= 1)
        {
          // after replan, set it false
          replan_force_surpass_ = true;
          last_external_acc_ << msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z;

          if ( have_target_){
            ROS_WARN("[checkReplan] External forces varies upper bound, replan !");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }

          if (surpass > 10.0)
          {
            ROS_WARN("[extforceCallback] The external forces is too fierce ! stop !");
            have_target_ = false;
            changeFSMExecState(WAIT_TARGET, "SAFETY");
          }
        }
      }
      else
      {
        surpass_count_ = 0;
      }
    }
  }
}

// get odom for planning
void KINOManage::odometryCallback(const nav_msgs::Odometry &msg)
{
  // in real world practice(we use the celocity in the world frame)
  // std::cout<<"odometryCallback In !!"<<std::endl;

  odomT = (msg.header.stamp - tOdom).toSec();
  tOdom = msg.header.stamp;
  stateOdomPrevious_ = cur_Odom_;

  cur_Odom_(0) = msg.pose.pose.position.x;
  cur_Odom_(1) = msg.pose.pose.position.y;
  cur_Odom_(2) = msg.pose.pose.position.z;

  cur_Odom_(3) = msg.twist.twist.linear.x;
  cur_Odom_(4) = msg.twist.twist.linear.y;
  cur_Odom_(5) = msg.twist.twist.linear.z;

  Eigen::Quaterniond quaternion;
  quaternion.w() = msg.pose.pose.orientation.w;
  quaternion.x() = msg.pose.pose.orientation.x;
  quaternion.y() = msg.pose.pose.orientation.y;
  quaternion.z() = msg.pose.pose.orientation.z;

  Eigen::Vector3d temp2;
  mav_msgs::getEulerAnglesFromQuaternion(quaternion, &temp2);
  cur_Odom_.segment(6, 3) = temp2;

  have_odom_ = true;
  // std::cout<<"odometryCallback Out !!"<<std::endl;
  //std::cout<<"537 odometry: "<<cur_Odom_.segment(0,3)<<std::endl;
}

/*
* For vio and planners in our lab systems, we directly use the global velocity in odometry
* and in Rotors Simulator the ground truth velocity is given in the body frame\
* http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html
*/

void KINOManage::odometryTransCallback(const nav_msgs::Odometry &msg)
{
  //std::cout<<"odometryTransCallback In !!"<<std::endl;
  // use RotorS sim
  odomT = (msg.header.stamp - tOdom).toSec();
  tOdom = msg.header.stamp;
  stateOdomPrevious_ = cur_Odom_;

  mav_msgs::EigenOdometry odometry;
  mav_msgs::eigenOdometryFromMsg(msg, &odometry);
  cur_Odom_(0) = odometry.position_W(0);
  cur_Odom_(1) = odometry.position_W(1);
  cur_Odom_(2) = odometry.position_W(2);
  const Eigen::Matrix3d R_W_I = odometry.orientation_W_B.toRotationMatrix();
  Eigen::Vector3d velocity_W = R_W_I * odometry.velocity_B;
  cur_Odom_(3) = velocity_W(0);
  cur_Odom_(4) = velocity_W(1);
  cur_Odom_(5) = velocity_W(2);
  Eigen::Vector3d temp2;
  mav_msgs::getEulerAnglesFromQuaternion(odometry.orientation_W_B, &temp2);
  cur_Odom_.segment(6, 3) = temp2;

  have_odom_ = true;

  std::cout<<"569 odometry: "<<cur_Odom_.segment(0,3)<<std::endl;
}

// revise from fast planner
void KINOManage::goalCallback(const geometry_msgs::PoseStamped &msg)
{

  if (msg.pose.position.z < -0.1) return;
  //cout << "Triggered!" << endl;
  trigger_ = true;
  have_target_ = true;

  end_pt_ << msg.pose.position.x, msg.pose.position.y, 1.2; // fix z around 1-1.5
  //ROS_INFO_STREAM("[resilient_planner] The end point is : \n"  << end_pt_);
  displayGoalPoint();

}

void KINOManage::targetReachedCallback(const std_msgs::Bool &msg)
{
  target_reached = msg.data;
  std::cout<<"target_reached??? "<<target_reached<<std::endl;
}