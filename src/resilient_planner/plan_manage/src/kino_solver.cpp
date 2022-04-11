#include <plan_manage/kino_utils.h>

constexpr double PI = 3.1415926;

namespace resilient_planner
{

  KINOSolver::KINOSolver()
  {
    //construct model matrices
    At_.setZero();
    Bt_.setZero();
    Dt_.setZero();

    // some pre initializations of the matrix
    At_(0, 3) = 1.0;
    At_(1, 4) = 1.0;
    At_(2, 5) = 1.0;

    Bt_(6, 0) = 1.0;
    Bt_(7, 1) = 1.0;
    Bt_(8, 2) = 1.0;

    Dt_(3, 0) = 1.0;
    Dt_(4, 1) = 1.0;
    Dt_(5, 2) = 1.0;

    Kt_ <<  -2.0,  5.0,  0.0, -1.0,  4.0,  0.0, -8.0,  0.0,  0.0,
            -5.0, -2.0,  0.0, -4.0, -1.0,  0.0,  0.0, -8.0,  0.0,
            -2.0, -2.0,  0.0, -1.0, -1.0,  0.0,  0.0,  0.0, -8.0,
             0.0,  0.0, -8.0,  0.0,  0.0, -6.0,  0.0,  0.0,  0.0;

    external_acc_.setZero();
    end_v_.setZero();

    cmd_status_ = CMD_STATUS::INIT_POSITION;
  }

  void KINOSolver::initROS(ros::NodeHandle &nh, OccMap::Ptr &env_ptr_)
  {
    /*  kino a* intial  */
    kino_path_finder_.reset(new KinodynamicAstar);
    kino_path_finder_->setParam(nh);
    kino_path_finder_->init();
    kino_path_finder_->intialGridMap(env_ptr_);
    ellipsoid_pub_ = nh.advertise<decomp_ros_msgs::EllipsoidArray>("ellipsoid_array", 1, true);
    poly_pub_ = nh.advertise<decomp_ros_msgs::PolyhedronArray>("polyhedron_array", 1, true);
    ref_marker_pub_ = nh.advertise<visualization_msgs::Marker>("ref_traj", 1, true);
    kino_marker_pub_ = nh.advertise<visualization_msgs::Marker>("kino_traj", 1, true);
    cmd_vis_pub_ = nh.advertise<visualization_msgs::Marker>("position_cmd_vis", 10, true);
    cloud_sub_ = nh.subscribe("/occ_map/local_view_cloud", 1, &KINOSolver::cloudCallback, this);

    double ego_r, ego_h, drag_coeff, ext_noise_bound;

    /* weight */
    double w_stage_wp, w_stage_input, w_terminal_wp, w_terminal_input,
        w_input_rate, w_final_terminal_wp, w_final_terminal_input,
        w_final_stage_wp, w_final_stage_input;

    nh.param("kino/w_stage_wp", w_stage_wp, 15.0);
    nh.param("kino/w_stage_input", w_stage_input, 3.0);
    nh.param("kino/w_terminal_wp", w_terminal_wp, 15.0);
    nh.param("kino/w_terminal_input", w_terminal_input, 0.0);
    nh.param("kino/w_input_rate", w_input_rate, 80.0);
    nh.param("kino/w_final_stage_wp", w_final_stage_wp, 20.0);
    nh.param("kino/w_final_stage_input", w_final_stage_input, 5.0);
    nh.param("kino/w_final_terminal_wp", w_final_terminal_wp, 20.0);
    nh.param("kino/w_final_terminal_input", w_final_terminal_input, 5.0);
    nh.param("kino/ego_r", ego_r, 0.27);
    nh.param("kino/ego_h", ego_h, 0.0425);
    nh.param("kino/g_acc", g, 9.81);
    nh.param("kino/mass", mass, 0.74);
    nh.param("kino/drag_coefficient", drag_coeff, 0.33);
    nh.param("kino/ext_noise_bound", ext_noise_bound, 0.5);

    nh.param("search/max_tau", max_tau_, 0.5);

    ROS_INFO_STREAM("ego_r: " << ego_r);
    ROS_INFO_STREAM("ego_h: " << ego_h);
    ROS_INFO_STREAM("max_tau: " << max_tau_);
    ROS_INFO_STREAM("mass: " << mass);

    // use rotors simulator
    cmd_timer_ = nh.createTimer(ros::Duration(0.01), &KINOSolver::cmdTrajCallback, this); // 5ms
    traj_cmd_pub_ = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/pos_cmd", 50);

    // ego info
    ego_size_ << ego_r * ego_r, 0.0, 0.0,
        0.0, ego_r * ego_r, 0.0,
        0.0, 0.0, ego_h * ego_h;

    //drag_coefficient_matrix
    drag_coefficient_matrix_ << drag_coeff, 0.0, 0.0,
        0.0, drag_coeff, 0.0,
        0.0, 0.0, 0.0;
    // disturbance vector
    w_ << ext_noise_bound, ext_noise_bound, ext_noise_bound;

    ROS_INFO_STREAM("[KINOSolver]Finish the intialization of the kino solver! \n");
  }

  // use kinodynamic a* to generate a path
  bool KINOSolver::getKinoPath(Eigen::VectorXd &stateOdom, Eigen::Vector3d end_pt,
                               Eigen::Vector3d external_acc, bool replan)
  {
    end_pt_ = end_pt;
    external_acc_ = external_acc;

    start_pt_ = stateOdom.segment(0, 3);
    start_v_ = stateOdom.segment(3, 3);
    start_a_.setZero();

    kino_path_finder_->reset();
    kino_path_finder_->updateExternalAcc(external_acc_);

    int status;
    
    std::cout<<"Kino Route Search"<<std::endl;
    status = kino_path_finder_->search(start_pt_, start_v_, start_a_, end_pt_, end_v_, true);
    

    if (status == KinodynamicAstar::NO_PATH)
    {
      std::cout << "[kino replan]: kinodynamic search fail!" << std::endl;

      // retry searching with discontinuous initial state
      kino_path_finder_->reset();
      status = kino_path_finder_->search(start_pt_, start_v_, start_a_, end_pt, end_v_, false);
      if (status == KinodynamicAstar::NO_PATH)
      {
        std::cout << "[kino replan]: Can't find path." << std::endl;
        return false;
      }
      else
      {
        std::cout << "[kino replan]: retry search success." << std::endl;
      }
    }
    else
    {
      std::cout << "[kino replan]: kinodynamic search success." << std::endl;
    }

    kino_path_ = kino_path_finder_->getKinoTraj(Ts_);
    kino_size_ = kino_path_.size();

    switch_to_final = false;
    kino_start_time_ = ros::Time::now();
    ROS_INFO_STREAM("kino_start_time_ : \n"
                    << kino_start_time_);
    cmd_status_ = CMD_STATUS::PUB_TRAJ;
    pub_end_ = false;

    return true;
  }

  void KINOSolver::callInitYaw(Eigen::VectorXd odom, double init_yaw)
  {

    ROS_INFO_STREAM("[KINOSolver] callInitYaw");
    realOdom_ = odom;
    init_yaw_ = init_yaw;
    pub_end_ = false;
    change_yaw_time_ = ros::Time::now();

    init_yaw_dot_ = init_yaw_ - realOdom_(8);

    if (init_yaw_dot_ > PI)
    {
      init_yaw_dot_ = 2 * PI - init_yaw_dot_;
    }
    else if (init_yaw_dot_ < -PI)
    {
      init_yaw_dot_ = init_yaw_dot_ + 2 * PI;
    }

    double max_yaw_dot = 0.4 * PI;

    if (init_yaw_dot_ > max_yaw_dot)
    {
      init_yaw_dot_ = max_yaw_dot;
    }
    else if (init_yaw_dot_ < -max_yaw_dot)
    {
      init_yaw_dot_ = -max_yaw_dot;
    }

    ROS_INFO_STREAM("init_yaw_dot_ : \n"
                    << init_yaw_dot_);
    cmd_status_ = CMD_STATUS::ROTATE_YAW;
  }


  void KINOSolver::initMPCOutput()
  {
    mpc_output_.clear();

    Eigen::VectorXd mpc_row(var);
    // control input is
    // state is: position, velocity and euler angle
    mpc_row << 0.0, 0.0, 0.0, real_thrust_c_,
        0.0, 0.0, 0.0, real_thrust_c_,
        stateMpc_(0), stateMpc_(1), stateMpc_(2), // position
        stateMpc_(3), stateMpc_(4), stateMpc_(5), // velocity
        stateMpc_(6), stateMpc_(7), stateMpc_(8); // euler angle

    for (int i = 0; i < planning_horizon_ + 1; i++)
    {
      mpc_output_.push_back(mpc_row);
    }

    initialized_output_ = true;
    pre_mpc_output_ = mpc_output_;
    ROS_INFO_STREAM("[KINOSolver::initMPCOutput] Finish the intialization of mpc output! ");
  }

  int KINOSolver::getSikangConst(Eigen::Matrix3d E)
  {
    int index = poly_constraints_.size();
    if (index >= 1)
    {
      //directly check the reference ellipsoid shape within the polytope
      auto temp_A = (poly_constraints_.back()).A();
      auto temp_b = (poly_constraints_.back()).b();
      bool flag = true;

      for (unsigned int j = 0; j < temp_b.size(); j++)
      {
        Eigen::MatrixXd temp_b_addition = E * (temp_A.row(j)).transpose();
        // with little inflation
        if (temp_A.row(j) * ref_pos_ - (temp_b(j) - 1.1 * temp_b_addition.norm()) > 0)
        {
          flag = false;
          break;
        }
      }

      // the point in the polytope
      if (flag)
        return index - 1; // same as last time poly
    }

    vec_Vec3f seed_path;
    seed_path.push_back(ref_pos_);

    Vec3f seed_point2;
    seed_point2 << ref_pos_[0] + 0.1*cos(ref_yaw_), ref_pos_[1] + 0.1*sin(ref_yaw_), ref_pos_[2];
    seed_path.push_back(seed_point2);

    EllipsoidDecomp3D decomp_util;
    decomp_util.set_obs(vec_obs_);
    decomp_util.set_local_bbox(Vec3f(2, 2, 1));
    decomp_util.dilate(seed_path);
    auto seed_poly = decomp_util.get_polyhedrons();

    vec_E<LinearConstraint3D> css = decomp_util.get_constraints();
    polys.push_back(seed_poly[0]);
    poly_constraints_.push_back(css[0]);

    return index;
  }

  void KINOSolver::displayPoly()
  {
    decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(polys);
    poly_msg.header.frame_id = "world";
    poly_pub_.publish(poly_msg);
  }

  void KINOSolver::getKinoTraj(std::vector<Eigen::VectorXd> &kino_path)
  {
    kino_path = kino_path_;
  }


  Eigen::Matrix3d KINOSolver::eulerToRot(Eigen::Vector3d &odom_euler)
  {

    Eigen::Vector3d rpy(odom_euler(0), odom_euler(1), odom_euler(2));
    Eigen::Quaternion<double> qx(cos(rpy(0) / 2), sin(rpy(0) / 2), 0, 0);
    Eigen::Quaternion<double> qy(cos(rpy(1) / 2), 0, sin(rpy(1) / 2), 0);
    Eigen::Quaternion<double> qz(cos(rpy(2) / 2), 0, 0, sin(rpy(2) / 2));
    Eigen::Matrix3d R = Eigen::Matrix3d(qz * qy * qx);

    return R;
  }

  // calculate the disturbance ellipsoid
  Eigen::Matrix3d KINOSolver::getDistrEllipsoid(double t, Eigen::MatrixXd &Q_origin)
  {

    Eigen::MatrixXd Nt(nx, nx);
    Eigen::MatrixXd Array_Q(nx, nx);
    Eigen::MatrixXd temp_Q = Eigen::MatrixXd::Zero(nx, nx);
    double temp;
    Eigen::MatrixXd B = Phi_.transpose();
    //ROS_INFO_STREAM(" B : \n" << B );
    Eigen::ComplexSchur<Eigen::MatrixXd> SchurA(Phi_);
    Eigen::MatrixXcd R = SchurA.matrixT();
    Eigen::MatrixXcd U = SchurA.matrixU();
    Eigen::ComplexSchur<Eigen::MatrixXd> SchurB(B);
    Eigen::MatrixXcd S = SchurB.matrixT();
    Eigen::MatrixXcd V = SchurB.matrixU();
    // refer to this: https://stackoverflow.com/questions/56929966/implementing-the-bartels-stewart-algorithm-in-eigen3
    // ROS_INFO_STREAM(" R : \n" << R);
    // ROS_INFO_STREAM(" U : \n" << U );
    // ROS_INFO_STREAM(" S : \n" << S );
    // ROS_INFO_STREAM(" V : \n" << V );
    Eigen::MatrixXd X = Eigen::MatrixXd::Zero(nx, nx);

    for (int i = 0; i < nw; ++i)
    {
      Nt = t * w_(i) * w_(i) * Dt_.col(i) * (Dt_.col(i)).transpose(); // nx * nx
      Array_Q = Nt - (-Phi_ * t).exp() * Nt * (-Phi_.transpose() * t).exp(); // nx * nx
      // Phi*X + X*Phi' = W
      Eigen::MatrixXcd F = (U.adjoint() * Array_Q) * V;
      Eigen::MatrixXcd Y = Eigen::internal::matrix_function_solve_triangular_sylvester(R, S, F);
      Eigen::MatrixXd X = ((U * Y) * V.adjoint()).real();
      temp += sqrt(X.trace());
      temp_Q += X / sqrt(X.trace());
    }

    Eigen::MatrixXd Qd = temp * temp_Q; // nx * nx
    double beta = sqrt(Q_origin.trace() / Qd.trace());
    Eigen::MatrixXd Q_update = (1 + 1 / beta) * Q_origin + (1 + beta) * Qd; // nx * nx
    Eigen::MatrixXd position_Q = (Phi_ * t).exp() * Q_update * (Phi_.transpose() * t).exp();

    // update Q_origin
    Q_origin = Q_update;

    return position_Q.block(0, 0, 3, 3);

  }


  // odom_euler:  roll pitch yaw
  Eigen::Matrix3d KINOSolver::updateMatrix(Eigen::Vector3d &odom_euler, Eigen::Vector3d &odom_vel, double thrust_c)
  {
    // update At matrix
    double roll = odom_euler(0), pitch = odom_euler(1), yaw = odom_euler(2);
    double v1 = odom_vel(0), v2 = odom_vel(1), v3 = odom_vel(2);
    double comb0 = thrust_c * 1.0 / mass;

    double comb5 = cos(pitch)*sin(pitch);

    double comb6 = cos(pitch)*sin(roll);
    double comb7 = cos(pitch)*cos(roll);
    double comb8 = sin(pitch)*cos(roll);
    double comb9 = sin(pitch)*sin(roll);
        
    double comb1 = cos(roll)*sin(yaw) - comb9*cos(yaw);
    double comb2 = sin(roll)*cos(yaw) - comb8*sin(yaw);
    double comb3 = cos(roll)*cos(yaw) + comb9*sin(yaw);
    double comb4 = sin(roll)*sin(yaw) + comb8*cos(yaw);

    // roll
    At_(3, 6) =  comb0*comb1;
    At_(4, 6) = -comb0*comb3;
    At_(5, 6) = -comb0*comb6;
    // pitch
    At_(3, 7) =  comb0*comb7*cos(yaw);
    At_(4, 7) =  comb0*comb7*sin(yaw);
    At_(5, 7) = -comb0*comb8;
    // yaw
    At_(3, 8) =  comb0*comb2;
    At_(4, 8) =  comb0*comb4;

    // add gradients with drag coefficient term (in some case we can neglect this term)
    Eigen::Matrix3d R_cur = eulerToRot(odom_euler);

    At_.block(3, 3, 3, 3) = R_cur * drag_coefficient_matrix_ * R_cur.transpose(); //transpose does not change
    
    double drag = drag_coefficient_matrix_(0, 0);
    double cos_pitch_sq = cos(pitch)*cos(pitch);
    double sin_pitch_sq = sin(pitch)*sin(pitch);
    double cos_yaw_sq   = cos(yaw)*cos(yaw);
    double sin_yaw_sq   = sin(yaw)*sin(yaw);
    double sin_roll_sq  = sin(roll)*sin(roll);
    double temp_sqaure1 = pow(comb3, 2);
    double temp_sqaure2 = pow(comb1, 2);

    Eigen::Vector3d temp1, temp2;
    Eigen::Vector4d temp3;

    temp1(0) = comb6*comb4 - comb7*comb1;
    temp1(1) = comb3*comb4 + comb1*comb2;
    temp1(2) = comb6*comb2 - comb7*comb3;
 
    // roll
    At_(3, 6) +=  drag*(v3*temp1(0) + v2*temp1(1) - 2*v1*comb4*comb1);
    At_(4, 6) +=  drag*(v1*temp1(1) - v3*temp1(2) - 2*v2*comb3*comb2);
    At_(5, 6) +=  drag*(v1*temp1(0) - v2*temp1(2) + 2*v3*comb7*comb6);
 
    // pitch
    temp2(0) =   cos(yaw)*(sin_pitch_sq - cos_pitch_sq + cos_pitch_sq*sin_roll_sq) + comb9*comb1;
    temp2(1) =   2*comb5*cos(yaw)*sin(yaw) - comb6*(cos(yaw)*comb3 + sin(yaw)*comb1);
    temp2(2) =   sin(yaw)*(cos_pitch_sq - sin_pitch_sq - cos_pitch_sq*sin_roll_sq) + comb9*comb3;

    At_(3, 7) +=   drag*(v3*temp2(0) - v2*temp2(1) - v1*2*(comb5*cos_yaw_sq + comb6*comb1*cos(yaw)));
    At_(4, 7) += - drag*(v3*temp2(2) - v1*temp2(1) - v2*2*(comb5*sin_yaw_sq - comb6*comb3*sin(yaw)));
    At_(5, 7) +=   drag*(v1*temp2(0) - v2*temp2(2) + v3*2*(comb5 - comb5*sin_roll_sq));
 
    // yaw
    temp3(0) = 2*drag*(comb3*comb1 - cos_pitch_sq*cos(yaw)*sin(yaw));
    temp3(1) =   drag*(comb6*comb3 - comb5*sin(yaw));
    temp3(2) =   drag*(temp_sqaure1 - temp_sqaure2 - cos_pitch_sq*cos_yaw_sq + cos_pitch_sq*sin_yaw_sq);
    temp3(3) =   drag*(comb6*comb1 + comb5*cos(yaw));

    At_(3, 8) +=   v1*temp3(0)  - v3*temp3(1) - v2*temp3(2);
    At_(4, 8) += - v1*temp3(2)  - v3*temp3(3) - v2*temp3(0);
    At_(5, 8) += - v2*temp3(3)  - v1*temp3(1);
 
    // update Bt matrix
    Bt_(3, 3) =  1.0 / mass*comb4;
    Bt_(4, 3) = -1.0 / mass*comb2;
    Bt_(5, 3) =  1.0 / mass*comb7;

    Phi_ = At_ + Bt_ * Kt_;

    return R_cur;
  }

  /*visulizations*/
  void KINOSolver::drawCmd(const Eigen::Vector3d &pos, const Eigen::Vector3d &vec, const int &id,
                           const Eigen::Vector4d &color)
  {
    visualization_msgs::Marker mk_state;
    mk_state.header.frame_id = "world";
    mk_state.header.stamp = ros::Time::now();
    mk_state.id = id;
    mk_state.type = visualization_msgs::Marker::ARROW;
    mk_state.action = visualization_msgs::Marker::ADD;

    mk_state.pose.orientation.w = 1.0;
    mk_state.scale.x = 0.1;
    mk_state.scale.y = 0.2;
    mk_state.scale.z = 0.3;

    geometry_msgs::Point pt;
    pt.x = pos(0);
    pt.y = pos(1);
    pt.z = pos(2);
    mk_state.points.push_back(pt);

    pt.x = pos(0) + vec(0);
    pt.y = pos(1) + vec(1);
    pt.z = pos(2) + vec(2);
    mk_state.points.push_back(pt);

    mk_state.color.r = color(0);
    mk_state.color.g = color(1);
    mk_state.color.b = color(2);
    mk_state.color.a = color(3);

    cmd_vis_pub_.publish(mk_state);
  }

  void KINOSolver::displayKinoPoints()
  {
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.type = visualization_msgs::Marker::LINE_STRIP;
    mk.action = visualization_msgs::Marker::DELETE;

    kino_marker_pub_.publish(mk);
    geometry_msgs::Point pt;
    std_msgs::ColorRGBA pc;

    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.orientation.w = 1.0;
    mk.scale.x = 0.1;

    pc.r = 0.5;
    pc.g = 0;
    pc.b = 0;
    pc.a = 0.6;

    for (int i = 0; i < planning_horizon_; i++)
    {
      
      pt.x = pre_mpc_output_.at(i)(8);
      pt.y = pre_mpc_output_.at(i)(9);
      pt.z = pre_mpc_output_.at(i)(10);

      mk.points.push_back(pt);
      mk.colors.push_back(pc);
    }

    kino_marker_pub_.publish(mk);
  }

  void KINOSolver::displayRefPoints()
  {
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.type = visualization_msgs::Marker::SPHERE_LIST;
    mk.action = visualization_msgs::Marker::DELETE;

    ref_marker_pub_.publish(mk);
    geometry_msgs::Point pt;
    std_msgs::ColorRGBA pc;

    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.orientation.w = 1.0;
    mk.scale.x = 0.2;
    mk.scale.y = 0.2;
    mk.scale.z = 0.2;

    pc.r = 0;
    pc.g = 0;
    pc.b = 0.5;
    pc.a = 0.6;

    for (int i = 0; i < planning_horizon_; i++)
    {

      pt.x = ref_total_pos_.at(i)(0);
      pt.y = ref_total_pos_.at(i)(1);
      pt.z = ref_total_pos_.at(i)(2);

      mk.points.push_back(pt);
      mk.colors.push_back(pc);
    }
    ref_marker_pub_.publish(mk);
  }

  void KINOSolver::displayEllipsoids()
  {
    decomp_ros_msgs::EllipsoidArray ellipsoids;
    for (int i = 0; i < planning_horizon_; i++)
    {
      decomp_ros_msgs::Ellipsoid ellipsoid;
      ellipsoid.d[0] = mpc_output_.at(i)(8);
      ellipsoid.d[1] = mpc_output_.at(i)(9);
      ellipsoid.d[2] = mpc_output_.at(i)(10);

      auto C = ellipsoid_matrices_[i];

      for (int x = 0; x < 3; x++)
      {
        for (int y = 0; y < 3; y++)
        {
          ellipsoid.E[3 * x + y] = C(x, y);
        }
      }
      ellipsoids.ellipsoids.push_back(ellipsoid);
    }

    ellipsoids.header.frame_id = "world";
    ellipsoid_pub_.publish(ellipsoids);
  }

  void KINOSolver::calculate_yaw(Eigen::Vector3d pos, Eigen::Vector3d pos_next)
  {
    double yaw = 0;

    Eigen::Vector3d dir = pos_next - pos;

    double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_; // if dir is some or not change

    if (fabs(yaw_temp - last_yaw_) > PI)
    {
      if (yaw_temp > 0)
      { // -3.14 to 3.1
        yaw = yaw_temp - 2 * PI;
      }
      else
      { // 3.14 to -3.1
        yaw = yaw_temp + 2 * PI;
      }
    }
    else
    {
      yaw = yaw_temp;
    }

    yaw = 0.2 * last_yaw_ + 0.8 * yaw; // nieve LPF

    last_yaw_ = yaw;
    ref_yaw_ = yaw;
  }

  /*** callbacks  ***/
  void KINOSolver::cmdTrajCallback(const ros::TimerEvent &e)
  {
    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
    trajectory_msg.header.stamp = ros::Time::now();
    trajectory_msg.header.frame_id = "world";

    trajectory_msgs::MultiDOFJointTrajectoryPoint point_msg;
    point_msg.transforms.resize(1);
    point_msg.velocities.resize(1);
    point_msg.accelerations.resize(1);

    switch (cmd_status_)
    {
      case INIT_POSITION: // if need any fixed position
      {
        break;
      }

      case ROTATE_YAW:
      {
        double yaw_temp = realOdom_(8) + (ros::Time::now() - change_yaw_time_).toSec() * init_yaw_dot_;
        double desired_yaw = init_yaw_ - realOdom_(8) >= 0 ? min(yaw_temp, init_yaw_) : max(yaw_temp, init_yaw_);

        Eigen::Vector3d desired_position(realOdom_(0), realOdom_(1), realOdom_(2));
        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);
        traj_cmd_pub_.publish(trajectory_msg);

        break;
      }

      case WAIT:
      {
        break;
      }

      case PUB_TRAJ:
      {

        if (!have_mpc_traj_)  break;

        ros::Time time_now = ros::Time::now();
        double t_cur = (time_now - pre_mpc_start_time_).toSec();
        int cur_index = (int)(t_cur / Ts_);

        if (cur_index < planning_horizon_ - 1 && t_cur >= 0.0)
        {
          Eigen::VectorXd mpcq = pre_mpc_output_.at(cur_index) + fmod(t_cur, Ts_) / Ts_ * (pre_mpc_output_.at(cur_index + 1) - pre_mpc_output_.at(cur_index));

          point_msg.transforms[0].translation.x = mpcq(8);
          point_msg.transforms[0].translation.y = mpcq(9);
          point_msg.transforms[0].translation.z = mpcq(10);

          point_msg.velocities[0].linear.x = mpcq(11);
          point_msg.velocities[0].linear.y = mpcq(12);
          point_msg.velocities[0].linear.z = mpcq(13);

          point_msg.velocities[0].angular.x = mpcq(0);
          point_msg.velocities[0].angular.y = mpcq(1);
          point_msg.velocities[0].angular.z = mpcq(2);

          Eigen::Vector3d odom_euler(mpcq(14), mpcq(15), mpcq(16));
          Eigen::Vector3d thrust_b(0.0, 0.0, mpcq(3));
          Eigen::Vector3d thrust_w = eulerToRot(odom_euler) * thrust_b;

          point_msg.accelerations[0].linear.x = thrust_w(0) / mass;
          point_msg.accelerations[0].linear.y = thrust_w(1) / mass;
          point_msg.accelerations[0].linear.z = thrust_w(2) / mass - g;

          Eigen::Quaterniond q = Eigen::AngleAxisd(mpcq(14), Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(mpcq(15), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(mpcq(16), Eigen::Vector3d::UnitZ());

          point_msg.transforms[0].rotation.x = q.x();
          point_msg.transforms[0].rotation.y = q.y();
          point_msg.transforms[0].rotation.z = q.z();
          point_msg.transforms[0].rotation.w = q.w();

          Eigen::Vector3d pos(mpcq(8), mpcq(9), mpcq(10));
          Eigen::Vector3d dir(cos(mpcq(16)), sin(mpcq(16)), 0.0);
          drawCmd(pos, dir, 2, Eigen::Vector4d(1, 1, 0, 0.7));

          trajectory_msg.points.push_back(point_msg);
          traj_cmd_pub_.publish(trajectory_msg);
          finish_mpc_cmd_ = false;
        }
        else
        {
          finish_mpc_cmd_ = true;
          if (pub_end_) cmd_status_ = CMD_STATUS::PUB_END;
        }
        break;
      }

      case PUB_END:
      { 

        point_msg.transforms[0].translation.x = cmd_end_pt_(0);
        point_msg.transforms[0].translation.y = cmd_end_pt_(1);
        point_msg.transforms[0].translation.z = cmd_end_pt_(2);

        point_msg.velocities[0].linear.x = 0;
        point_msg.velocities[0].linear.y = 0;
        point_msg.velocities[0].linear.z = 0;

        double roll = pre_mpc_output_.at(19)(14);
        double pitch = pre_mpc_output_.at(19)(15);
        double yaw = pre_mpc_output_.at(19)(16);

        Eigen::Quaterniond q = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

        point_msg.transforms[0].rotation.x = q.x();
        point_msg.transforms[0].rotation.y = q.y();
        point_msg.transforms[0].rotation.z = q.z();
        point_msg.transforms[0].rotation.w = q.w();

        trajectory_msg.points.push_back(point_msg);

        traj_cmd_pub_.publish(trajectory_msg);
        initialized_output_ = false;
        ROS_INFO_STREAM(" end time is " << ros::Time::now().toSec());
        cmd_status_ = CMD_STATUS::WAIT;
        break;
      }
    }
  }


  void KINOSolver::cloudCallback(const sensor_msgs::PointCloud2 &msg)
  {
    sensor_msgs::PointCloud out_cloud;
    sensor_msgs::convertPointCloud2ToPointCloud(msg, out_cloud);
    vec_obs_ = DecompROS::cloud_to_vec(out_cloud);
  }

}




  
