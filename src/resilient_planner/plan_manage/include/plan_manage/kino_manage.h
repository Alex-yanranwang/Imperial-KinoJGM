#ifndef _KINO_MANAGE_H_
#define _KINO_MANAGE_H_

#include <plan_manage/kino_utils.h>

using namespace resilient_planner;

class KINOManage
{
//1
public:
  KINOManage(){}
  ~KINOManage(){}

  enum FSM_EXEC_STATE
  {
    INIT,//-1
    WAIT_TARGET,//0
    INIT_YAW,//1
    GENERATE_NEW_TRAJ,//2
    EXEC_TRAJ,//3
    REACH_TARGET,//4
    REPLAN_TRAJ//5
  };

  void init(ros::NodeHandle& nh);

private:

  /* front-end map*/
  OccMap::Ptr env_ptr_;

  /* fsm parameters */
  FSM_EXEC_STATE exec_state_;

  /* odom variables */
  Eigen::VectorXd init_pt_, init_yaw_pt_, cur_Odom_, stateOdomPrevious_, stateMpc_; //P v euler;
  Eigen::Vector3d end_pt_, start_acc_, end_acc_, external_acc_, last_external_acc_; 
  
  /* for yaw settings */
  double init_yaw_;
  bool call_init_yaw_ = false;
  bool target_reached = false;

  /* decision variables */
  bool have_init_ = false;
  bool have_init_yaw_ = false;

  bool have_traj_ = false;
  bool have_target_ = false;
  bool have_odom_ = false;
  bool have_extforce_  = false;

  bool trigger_ = false;
  bool exec_mpc_ = false;

  bool replan_force_surpass_ = false;
  bool consider_force_ = false;

  int surpass_count_ = 0;
  int plan_fail_count_ = 0;
  
  /* Kino solver */
  KINOSolver kino_solver_;

  double ext_noise_bound_;
  double odomT;
  ros::Time tOdom, tMpc;
  std::vector<Eigen::VectorXd> kino_path_;

  /* ROS  related */
  ros::Subscriber odom_sub_, goal_sub_, extforce_sub_, target_reached_sub_;
  ros::Timer exec_timer_, safety_timer_, adjust_yaw_timer_, mpc_timer_;
  ros::Publisher path_pub_, poly_pub_, pos_cmd_pub_, nmpc_point_pub_ , goal_point_pub_, reference_state_pub_, init_yaw_state_pub_;

  /* FSM related */ 
  void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
  void execFSMCallback(const ros::TimerEvent& e);
  void checkReplanCallback(const ros::TimerEvent& e);
  void adjustYawCallback(const ros::TimerEvent& e);
  //void mpcCallback(const ros::TimerEvent& e);

  /* ROS callbacks */
  void odometryCallback(const nav_msgs::Odometry& msg);
  void odometryTransCallback(const nav_msgs::Odometry& msg);
  void goalCallback(const geometry_msgs::PoseStamped &msg);
  void extforceCallback(const geometry_msgs::WrenchStamped& msg);

  void targetReachedCallback(const std_msgs::Bool &msg);

  /* visualization */
  void displayPath();  
  void displayGoalPoint();

  void sendRefState();
  void sendInitYawState(Eigen::VectorXd init_yaw_);


};



#endif
