#include "bspline_opt/uniform_bspline.h"
#include "MPC.hpp"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "ego_planner/Bspline.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt8MultiArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_listener.h"
#include "tf/transform_datatypes.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>
#include "time.h"

#define PI 3.1415926
#define N 15

// [新增参数] 车辆物理参数 (Autolabor M2)
#define CAR_MIN_TURN_RADIUS 1.2 // 最小转弯半径 1.2m
#define CAR_WHEELBASE 0.65      // 轴距 0.65m

ros::Publisher vel_cmd_pub;
geometry_msgs::Twist cmd;

using ego_planner::UniformBspline;

bool receive_traj_ = false;
bool is_orientation_init = false;
vector<UniformBspline> traj_;
double traj_duration_;
ros::Time start_time_;
int traj_id_;

Eigen::Vector3d odom_pos_,odom_vel_;
Eigen::Quaterniond odom_orient_;
double roll, pitch, yaw; // 当前欧拉角
double yaw_cur_; // 保存当前偏航角

MPC_controller mpc_controller;
geometry_msgs::PoseStamped pose_cur;
tf::Quaternion quat;
std_msgs::UInt8 dir;
enum DIRECTION {POSITIVE=0,NEGATIVE=1};
double t_step;
std_msgs::UInt8 stop_command;

clock_t start_clock,end_clock;
double duration;

void bsplineCallback(ego_planner::BsplineConstPtr msg)
{
  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());
  Eigen::VectorXd knots(msg->knots.size());
  for (size_t i = 0; i < msg->knots.size(); ++i) knots(i) = msg->knots[i];
  for (size_t i = 0; i < msg->pos_pts.size(); ++i) {
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }

  UniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  start_time_ = msg->start_time;
  traj_id_ = msg->traj_id;

  traj_.clear();
  traj_.push_back(pos_traj);
  traj_.push_back(traj_[0].getDerivative());
  traj_.push_back(traj_[1].getDerivative());

  traj_duration_ = traj_[0].getTimeSum();
  receive_traj_ = true;
  stop_command.data = 0;
}

void dirCallback(const std_msgs::UInt8ConstPtr& msg)
{
    dir = *msg;
}

// MPC 计算核心
void MPC_calculate(double &t_cur, Eigen::Vector2d &cmd_out)
{
    std::vector<Eigen::Vector3d> X_r;
    std::vector<Eigen::Vector2d> U_r;
    Eigen::MatrixXd u_k;
    Eigen::Vector3d pos_r, pos_final, v_r_1, X_k;
    Eigen::Vector2d u_r;
    Eigen::Vector3d x_r;
    double t_k;
    
    pos_final = traj_[0].evaluateDeBoor(traj_duration_);

    static double last_valid_yaw_ref = yaw; 
    is_orientation_init=true;

    for(int i=0;i<N;i++)
    {
        t_k = t_cur + i*t_step;
        pos_r = traj_[0].evaluateDeBoor(t_k);
        v_r_1 = traj_[1].evaluateDeBoor(t_k);
        v_r_1(2)=0;

        x_r(0) = pos_r(0);
        x_r(1) = pos_r(1);

        // [优化] 航向角计算
        if((t_k - traj_duration_) >= 0)
        {
            x_r(2) = atan2((pos_r - pos_final)(1), (pos_r - pos_final)(0));
        }
        else
        {
             // 只有速度足够大才更新 Yaw，防止起步抖动
             if (v_r_1.norm() > 0.05) {
                double temp_yaw = atan2(v_r_1(1), v_r_1(0));
                x_r(2) = temp_yaw;
                last_valid_yaw_ref = temp_yaw;
            } else {
                x_r(2) = last_valid_yaw_ref;
            }
        }

        u_r(0) = v_r_1.norm();
        u_r(1) = 0; 

        X_r.push_back(x_r);
        U_r.push_back(u_r);
    }

    X_k(0) = odom_pos_(0);
    X_k(1) = odom_pos_(1);
    
    // Yaw 角去卷绕处理
    double diff = yaw - X_r[0](2);
    if (diff > PI) X_k(2) = yaw - 2*PI;
    else if (diff < -PI) X_k(2) = yaw + 2*PI;
    else X_k(2) = yaw;

    u_k = mpc_controller.MPC_Solve_qp(X_k, X_r, U_r, N);

    cmd_out(0) = u_k.col(0)(0); // 线速度 m/s
    cmd_out(1) = u_k.col(0)(1); // 角速度 rad/s
}

void stopCallback(std_msgs::UInt8ConstPtr msg)
{
    stop_command = *msg;
}

void odometryCallback(const nav_msgs::OdometryConstPtr &msg)
{
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    tf::quaternionMsgToTF(msg->pose.pose.orientation,quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    yaw_cur_ = yaw; 

    if(dir.data==NEGATIVE)
    {
        if(yaw>0) yaw -= PI;
        else if(yaw<0) yaw += PI;
    }
}

void cmdCallback(const ros::TimerEvent &e)
{
    if (stop_command.data==1) {
        cmd.linear.x = 0; cmd.angular.z = 0;
        vel_cmd_pub.publish(cmd);
        return;
    }
    if (!receive_traj_) return;

    ros::Time time_s = ros::Time::now();
    double t_cur = (time_s - start_time_).toSec();
    
    Eigen::Vector2d cmd_mpc(0,0); // m/s, rad/s

    // 获取终点和距离
    Eigen::Vector3d final_goal = traj_[0].evaluateDeBoor(traj_duration_);
    double dist_to_goal = (final_goal.head(2) - odom_pos_.head(2)).norm();

    // ==========================================
    // 逻辑 1: 正常 MPC 跟踪 (允许超时 2s，直到接近终点)
    // ==========================================
    if (t_cur < traj_duration_ || (dist_to_goal > 0.3 && t_cur < traj_duration_ + 2.0))
    {
        double t_calc = (t_cur > traj_duration_) ? traj_duration_ : t_cur;
        MPC_calculate(t_calc, cmd_mpc);
    }
    // ==========================================
    // 逻辑 2: 停车伺服 (Servo) - [修正版] 适配阿克曼
    // ==========================================
    else if (dist_to_goal > 0.05) 
    {
        // 计算目标方向
        Eigen::Vector2d dir_vec = final_goal.head(2) - odom_pos_.head(2);
        double target_yaw = atan2(dir_vec(1), dir_vec(0));
        
        if (dir.data == NEGATIVE) {
            if (target_yaw > 0) target_yaw -= PI; else target_yaw += PI;
        }

        double yaw_err = target_yaw - yaw_cur_;
        while(yaw_err > PI) yaw_err -= 2*PI;
        while(yaw_err < -PI) yaw_err += 2*PI;

        // [核心修改] 阿克曼车禁止原地转向 (v=0)
        double creep_speed = 0.2; // 基础蠕动速度
        if (dist_to_goal < 0.2) creep_speed = 0.1; // 极近距离减速

        cmd_mpc(0) = creep_speed; 

        // 转向控制：简单的 P 控制
        double w_cmd = yaw_err * 1.0; 
        
        // 阿克曼约束
        double max_w_servo = creep_speed / 1.2;
        if (w_cmd > max_w_servo * 1.5) w_cmd = max_w_servo * 1.5;
        if (w_cmd < -max_w_servo * 1.5) w_cmd = -max_w_servo * 1.5;

        cmd_mpc(1) = w_cmd;

        ROS_INFO_THROTTLE(1.0, "[Servo] Creeping... Dist: %.2f, YawErr: %.2f", dist_to_goal, yaw_err);
    }
    // ==========================================
    // 逻辑 3: 到达
    // ==========================================
    else
    {
        cmd_mpc(0) = 0.0; cmd_mpc(1) = 0.0;
        receive_traj_ = false;
    }

    // ==========================================
    // 后处理 1: 倒车处理
    // ==========================================
    if(dir.data == NEGATIVE) cmd_mpc(0) = -cmd_mpc(0);

    // ==========================================
    // 后处理 2: 死区补偿
    // ==========================================
    double min_move = 0.15; 
    if (std::abs(cmd_mpc(0)) > 0.01 && std::abs(cmd_mpc(0)) < min_move) {
        cmd_mpc(0) = (cmd_mpc(0) > 0) ? min_move : -min_move;
    }

    // ==========================================
    // 后处理 3: [关键] 阿克曼约束滤波器
    // ==========================================
    double current_v = std::abs(cmd_mpc(0)); // 物理速度 m/s
    double max_w_ackermann = current_v / CAR_MIN_TURN_RADIUS;
    
    if (current_v < 0.1) max_w_ackermann = 0.4; 

    if (cmd_mpc(1) > max_w_ackermann) cmd_mpc(1) = max_w_ackermann;
    if (cmd_mpc(1) < -max_w_ackermann) cmd_mpc(1) = -max_w_ackermann;

    // ==========================================
    // 后处理 4: 输出
    // ==========================================
    cmd.linear.x = cmd_mpc(0); 
    cmd.angular.z = cmd_mpc(1);

    vel_cmd_pub.publish(cmd);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle node("~");

  std::string cmd_topic,pose_topic;
  node.getParam("/ego_planner_node/fsm/pose_topic",pose_topic);
  node.getParam("/ego_planner_node/fsm/vel_topic",cmd_topic);

  ros::Subscriber bspline_sub = node.subscribe("/planning/bspline", 10, bsplineCallback);
  ros::Subscriber odom_sub = node.subscribe("/state_estimation", 10, odometryCallback);
  ros::Subscriber stop_sub = node.subscribe("/emergency_stop",10,stopCallback);
  ros::Subscriber command_sub = node.subscribe("/direction",10,dirCallback);

  mpc_controller.MPC_init(node);
  vel_cmd_pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 50);
  stop_command.data = 0;
  dir.data = POSITIVE;
  
  // [修改] 恢复短步长，保证紧贴轨迹，防止切弯
  t_step = 0.03; 

  ros::Timer cmd_timer = node.createTimer(ros::Duration(0.03), cmdCallback);
  ros::Duration(1.0).sleep();
  ROS_WARN("[Traj server]: Ackermann Logic Ready.");
  ros::spin();
  return 0;
}