
#include <plan_manage/ego_replan_fsm.h>
#include <plan_env/param_utils.hpp>
#include <rclcpp/create_timer.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#define PI 3.1415926

namespace ego_planner
{

  void EGOReplanFSM::init(const rclcpp::Node::SharedPtr &nh)
  {
    node_ = nh;
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;
    // These flags were left uninitialized in the ROS 1 version, which made the
    // initial FSM state depend on garbage memory.
    trigger_ = false;
    have_new_target_ = false;
    flag_escape_emergency_ = false;
    is_pos_update = false;

    /*  fsm param  */
    target_type_ = plan_env::getParam<int>(node_, "fsm.flight_type", -1);
    replan_thresh_ = plan_env::getParam<double>(node_, "fsm.thresh_replan", -1.0);
    no_replan_thresh_ = plan_env::getParam<double>(node_, "fsm.thresh_no_replan", -1.0);
    planning_horizen_ = plan_env::getParam<double>(node_, "fsm.planning_horizon", -1.0);
    planning_horizen_time_ = plan_env::getParam<double>(node_, "fsm.planning_horizen_time", -1.0);
    emergency_time_ = plan_env::getParam<double>(node_, "fsm.emergency_time_", 1.0);
    max_tracking_error_ = plan_env::getParam<double>(node_, "fsm.max_tracking_error", -1.0);
    forward_only_ = plan_env::getParam<bool>(node_, "fsm.forward_only", false);
    inflation_escape_ = plan_env::getParam<bool>(node_, "fsm.inflation_escape", false);
    escape_speed_ = plan_env::getParam<double>(node_, "fsm.escape_speed", 0.06);
    escape_hard_radius_ = plan_env::getParam<double>(node_, "fsm.escape_hard_radius", 0.21);
    w_adjust = plan_env::getParam<double>(node_, "fsm.w_adjust_", 1.0);
    yaw_error_threshold = plan_env::getParam<double>(node_, "fsm.yaw_tolerance", 25.0/180*PI);
    odom_twist_in_body_frame_ = plan_env::getParam<bool>(node_, "odom_twist_in_body_frame", false);

    waypoint_num_ = plan_env::getParam<int>(node_, "fsm.waypoint_num", -1);
    for (int i = 0; i < waypoint_num_; i++)
    {
      waypoints_[i][0] = plan_env::getParam<double>(node_, "fsm.waypoint" + to_string(i) + "_x", -1.0);
      waypoints_[i][1] = plan_env::getParam<double>(node_, "fsm.waypoint" + to_string(i) + "_y", -1.0);
      waypoints_[i][2] = plan_env::getParam<double>(node_, "fsm.waypoint" + to_string(i) + "_z", -1.0);
    }

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new EGOPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);
    dir = POSITIVE;
      goal_last << 0,0,0;

    /* callback
     * The timers run on the node clock (i.e. on simulation time when
     * use_sim_time is set), exactly like ros::Timer did in the ROS 1 version. */
    exec_timer_ = rclcpp::create_timer(node_, node_->get_clock(), std::chrono::milliseconds(10),
                                       std::bind(&EGOReplanFSM::execFSMCallback, this));
    safety_timer_ = rclcpp::create_timer(node_, node_->get_clock(), std::chrono::milliseconds(50),
                                         std::bind(&EGOReplanFSM::checkCollisionCallback, this));

    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/odom_map", 1, std::bind(&EGOReplanFSM::odometryCallback, this, std::placeholders::_1));

    bspline_pub_ = node_->create_publisher<ego_planner::msg::Bspline>("/planning/bspline", 10);
    data_disp_pub_ = node_->create_publisher<ego_planner::msg::DataDisp>("/planning/data_display", 100);
    // cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/twd_velocity_controller/cmd_vel",100);
    cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",100);
    adjust_cmd_pub_ = node_->create_publisher<std_msgs::msg::UInt8>("/is_adjust_yaw",100);
    odom_adjust_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("/odom_adjust",100);
    dir_pub = node_->create_publisher<std_msgs::msg::UInt8>("/direction",100);
    stop_pub = node_->create_publisher<std_msgs::msg::UInt8>("/emergency_stop",100);
    if (inflation_escape_)
    {
      planning_frame_ = plan_env::getParam<std::string>(node_, "grid_map.frame_id", "world");
      escape_marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("/planning/escape_direction", 1);
      escape_cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/grid_map/cloud", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
          if (!have_odom_ || msg->header.frame_id != planning_frame_) {return;}
          EscapePoints points;
          try {
            sensor_msgs::PointCloud2ConstIterator<float> x(*msg, "x"), y(*msg, "y");
            for (; x != x.end(); ++x, ++y) {
              if (std::isfinite(*x) && std::isfinite(*y) &&
                std::hypot(*x-odom_pos_.x(), *y-odom_pos_.y()) < 2.0) {points.emplace_back(*x, *y);}
            }
          } catch (const std::exception &) {return;}
          escape_obstacles_ = std::move(points);
          escape_cloud_stamp_ = msg->header.stamp;
        });
    }

    is_target_receive = false;

    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
      goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/way_point", 1, std::bind(&EGOReplanFSM::goal_callback, this, std::placeholders::_1));
      //waypoint_sub_ = nh.subscribe("/waypoint_generator/waypoints", 1, &EGOReplanFSM::waypointCallback, this);
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      // In ROS 1 this blocked inside `init()` until odometry arrived. Here the global
      // trajectory is generated from the exec timer as soon as odometry is available.
      preset_wps_pending_ = true;
    }
    else
      cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
  }

  void EGOReplanFSM::planGlobalTrajbyGivenWps()
  {
    std::vector<Eigen::Vector3d> wps(waypoint_num_);
    for (int i = 0; i < waypoint_num_; i++)
    {
      wps[i](0) = waypoints_[i][0];
      wps[i](1) = waypoints_[i][1];
      wps[i](2) = waypoints_[i][2];

      end_pt_ = wps.back();
    }
    bool success = planner_manager_->planGlobalTrajWaypoints(odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    for (size_t i = 0; i < (size_t)waypoint_num_; i++)
    {
      visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
    }

    if (success)
    {

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      std::vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      // if (exec_state_ == WAIT_TARGET)
      changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      // else if (exec_state_ == EXEC_TRAJ)
      //   changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory!");
    }
  }

  void EGOReplanFSM::waypointCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg)
  {
    if (msg->poses[0].pose.position.z < -0.1)
      return;

    RCLCPP_INFO(node_->get_logger(), "goal received (waypoint list)");
    trigger_ = true;
    init_pt_ = odom_pos_;


    bool success = false;
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, 1.0;
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else if (exec_state_ == EXEC_TRAJ)
        changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory!");
    }
  }

void EGOReplanFSM::goal_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg)
{
    end_pt_ << msg->pose.position.x, msg->pose.position.y, odom_pos_(2);

    double error_x = msg->pose.position.x - odom_pos_(0);
    double error_y = msg->pose.position.y - odom_pos_(1);

    RCLCPP_INFO(node_->get_logger(), "goal received: %.2f %.2f (direction to target: %.3f rad)",
                msg->pose.position.x, msg->pose.position.y, atan2(error_y, error_x));
    trigger_ = true;

    init_pt_ = odom_pos_;

    bool success = false;


    goal_last = end_pt_;

//    last_state_ = exec_state_;
//    yaw_start = atan2(error_y,error_x);
//    yaw_error = yaw_start-yaw;
//
//    //first step : calculate the yaw error
//    if(abs(yaw_error)>PI)
//    {
//        yaw_error = yaw_error - yaw_error/abs(yaw_error)*2*PI;
//    }
//    if(abs(yaw_error)>PI/2.0)
//    {
//        if(yaw>0)
//        {
//            yaw -= PI;
//        }else if(yaw<0)
//        {
//            yaw += PI;
//        }
//        changeDirection();
//        //yaw_error = - yaw_error/abs(yaw_error)*(PI-abs(yaw_error));
//        yaw_error = yaw_start - yaw;
//
//    }
//    if(abs(yaw_error)>yaw_error_threshold)
//    {
//        cmd_vel.twist.linear.x = 0;
//        cmd_vel.twist.angular.z = yaw_error/abs(yaw_error)*w_adjust;
//        bool success = false;
//        end_pt_ << msg->point.x, msg->point.y, msg->point.z;
//        success = planner_manager_->planGlobalTraj(odom_pos_,odom_vel_, Eigen::Vector3d::Zero(), end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
//        changeFSMExecState(ADJUST_POSE, "TRIG");
//        return;
//    }

    success = planner_manager_->planGlobalTraj(odom_pos_,odom_vel_, Eigen::Vector3d::Zero(), end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {
        /*** display ***/
        constexpr double step_size_t = 0.1;
        int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
        vector<Eigen::Vector3d> gloabl_traj(i_end);
        for (int i = 0; i < i_end; i++)
        {
            gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
        }
        end_vel_.setZero();
        have_target_ = true;
        have_new_target_ = true;

        //goal is too close to current pose

        /*** FSM ***/
        if (forward_only_ && exec_state_ != ESCAPE_INFLATION)
        {
            cmd_pub_->publish(geometry_msgs::msg::Twist());
            changeFSMExecState(GEN_NEW_TRAJ, "GOAL");
            is_target_receive = false;
        }
        else if (exec_state_ == WAIT_TARGET)
        {
            changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
            is_target_receive = true;
        }
        else if (exec_state_ == EXEC_TRAJ)
        {
            changeFSMExecState(REPLAN_TRAJ, "TRIG");
            is_target_receive=true;
        }

        visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);

        // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
        //visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
        RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory!");
    }
}


  void EGOReplanFSM::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    //odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    // nav_msgs/Odometry specifies twist in child_frame_id. Polynomial/B-spline
    // boundary velocities must be expressed in the same frame as the positions.
    if (odom_twist_in_body_frame_)
      odom_vel_ = odom_orient_.normalized() * odom_vel_;

    quat = tf2::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                           msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    if(dir==NEGATIVE)
    {
        if(yaw>0)
        {
            yaw -= PI;
        }else if(yaw<0)
        {
            yaw += PI;
        }
    }
      nav_msgs::msg::Odometry odom_adjust;
      tf2::Quaternion quat_adjust;
      quat_adjust.setRPY(roll, pitch, yaw);
      geometry_msgs::msg::Quaternion quat;
      quat.x = quat_adjust.x();
      quat.y = quat_adjust.y();
      quat.z = quat_adjust.z();
      quat.w = quat_adjust.w();
      odom_adjust = *msg;
      odom_adjust.pose.pose.orientation = quat;
      odom_adjust_pub_->publish(odom_adjust);

    have_odom_ = true;
  }

  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[8] = {"INIT", "WAIT_TARGET","ADJUST_POSE","GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "ESCAPE_INFLATION"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    // Keep control ownership consistent even if a safety transition interrupts
    // ADJUST_POSE before its normal completion branch.
    is_adjust_pose.data = new_state == ADJUST_POSE || new_state == ESCAPE_INFLATION ||
      (forward_only_ && new_state != EXEC_TRAJ) ? 1 : 0;
    adjust_cmd_pub_->publish(is_adjust_pose);
    if (forward_only_ && new_state != EXEC_TRAJ) {cmd_pub_->publish(geometry_msgs::msg::Twist());}
    RCLCPP_INFO(node_->get_logger(), "[%s]: from %s to %s", pos_call.c_str(),
                state_str[pre_s].c_str(), state_str[int(new_state)].c_str());
  }

  void EGOReplanFSM::checkYawError() {
      yaw_error = yaw_start-yaw;
      //first step : find the real yaw error
      if(abs(yaw_error)>yaw_error_threshold)
      {
          //if yaw error is larger than PI,it means the symbol between current yaw and target yaw is different
          if(abs(yaw_error)>PI)
          {
              //calculate the real yaw error with symbol
              yaw_error = yaw_error - yaw_error/abs(yaw_error)*2*PI;
              //if real yaw error larger than PI/2, change the direction of robot
              if(abs(yaw_error)>PI/2)
              {
                  changeDirection();

              }
          }
          else
          {
              cmd_vel.linear.x = 0;
              cmd_vel.angular.z = yaw_error/abs(yaw_error)*w_adjust;

          }
      }
  }

  double EGOReplanFSM::calculateYawError(double yaw_cur,double yaw_target) {
      double error = yaw_target - yaw_cur;
      if(abs(error)>PI)
      {
          error = error - error/abs(error)*2*PI;
          return error;
      }
      else
      {
          return error;
      }
  }

  void EGOReplanFSM::changeDirection() {
      if (forward_only_) {dir = POSITIVE; return;}
      if(dir == POSITIVE)
      {
          dir = NEGATIVE;
      }else
      {
          dir = POSITIVE;
      }
      std_msgs::msg::UInt8 dir_new;
      dir_new.data = dir;
      dir_pub->publish(dir_new);
  }



  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void EGOReplanFSM::printFSMExecState()
  {
    static string state_str[8] = {"INIT", "WAIT_TARGET","ADJUST_POSE","GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "ESCAPE_INFLATION"};

    RCLCPP_INFO(node_->get_logger(), "[FSM]: state: %s", state_str[int(exec_state_)].c_str());
  }

  void EGOReplanFSM::execFSMCallback()
  {

    if (inflation_escape_ && have_odom_ && have_target_ && exec_state_ != ESCAPE_INFLATION)
    {
      if (planner_manager_->grid_map_->getInflateOccupancy2d(odom_pos_.head<2>()) == 1)
      {
        if (inflation_since_.nanoseconds() == 0) {inflation_since_ = node_->now();}
        if ((node_->now()-inflation_since_).seconds() > 0.25)
        {
          escape_target_valid_ = false;
          cmd_pub_->publish(geometry_msgs::msg::Twist());
          changeFSMExecState(ESCAPE_INFLATION, "INFLATION");
          RCLCPP_WARN(node_->get_logger(), "Inflation overlap: turn toward a measured safe exit, then drive forward");
        }
      }
      else {inflation_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);}
    }

    if (preset_wps_pending_ && have_odom_)
    {
      preset_wps_pending_ = false;
      planGlobalTrajbyGivenWps();
    }

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        RCLCPP_INFO(node_->get_logger(), "no odom yet.");
      if (!trigger_)
        RCLCPP_INFO(node_->get_logger(), "wait for goal.");
      fsm_num = 0;
    }

    switch (exec_state_)
    {
    case ESCAPE_INFLATION:
      escapeInflation();
      break;
    case INIT:
    {
      if (!have_odom_)
      {
        return;
      }
      if (!trigger_)
      {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_)
        return;
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case ADJUST_POSE:
    {
//        if(last_state_!=WAIT_TARGET&&last_state_!=EMERGENCY_STOP)
//        {
//            callEmergencyStop(odom_pos_);
//        }
        yaw_error = yaw_start-yaw;
        //first step : calculate the yaw error
        if(abs(yaw_error)>PI)
        {
            yaw_error = yaw_error - yaw_error/abs(yaw_error)*2*PI;
        }
        static int count=0;
        if(count%100==0)
        {
            RCLCPP_INFO(node_->get_logger(), "ADJUST_POSE: yaw=%.3f yaw_start=%.3f yaw_error=%.3f", yaw, yaw_start, yaw_error);
            count=0;
        }
        count+=1;
        if(abs(yaw_error)>yaw_error_threshold)
        {
            is_adjust_pose.data = 1;
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = yaw_error/abs(yaw_error)*w_adjust;
            cmd_pub_->publish(cmd_vel);
            adjust_cmd_pub_->publish(is_adjust_pose);
        }else
        {
            is_adjust_pose.data = 0;
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z =0;
            cmd_pub_->publish(cmd_vel);
            changeFSMExecState(EXEC_TRAJ, "FSM");
            adjust_cmd_pub_->publish(is_adjust_pose);
            auto info = &planner_manager_->local_data_;
            info->start_time_ = node_->now();
            publishBspline();
        }
        break;
    }

    case GEN_NEW_TRAJ:
    {
      if (forward_only_) {cmd_pub_->publish(geometry_msgs::msg::Twist());}
      start_pt_ = odom_pos_;
      start_vel_ << 0,0,0;
      start_acc_.setZero();

      // Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      // start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      // start_yaw_(1) = start_yaw_(2) = 0.0;

      bool flag_random_poly_init;
      if (timesOfConsecutiveStateCalls().first == 1)
        flag_random_poly_init = false;
      else
        flag_random_poly_init = true;

      bool success = callReboundReplan(true, flag_random_poly_init);
      if (success)
      {
          Eigen::Vector3d vel_start = planner_manager_->local_data_.velocity_traj_.evaluateDeBoor(0.1);
          yaw_start = atan2(vel_start(1),vel_start(0));
          RCLCPP_INFO(node_->get_logger(), "yaw start : %.3f", yaw_start);
          yaw_error = yaw_start-yaw;

          //first step : calculate the yaw error
          if(abs(yaw_error)>PI)
          {
              yaw_error = yaw_error - yaw_error/abs(yaw_error)*2*PI;
          }
          RCLCPP_INFO(node_->get_logger(), "yaw error : %.3f", yaw_error);
          if(!forward_only_ && abs(yaw_error)>PI/2.0)
          {
              if(yaw>0)
              {
                  yaw -= PI;
              }else if(yaw<0)
              {
                  yaw += PI;
              }
              changeDirection();
              //yaw_error = - yaw_error/abs(yaw_error)*(PI-abs(yaw_error));
              yaw_error = yaw_start - yaw;

          }
          if(abs(yaw_error)>yaw_error_threshold)
          {
              cmd_vel.linear.x = 0;
              cmd_vel.angular.z = yaw_error/abs(yaw_error)*w_adjust;
              changeFSMExecState(ADJUST_POSE, "TRIG");
              last_state_ = GEN_NEW_TRAJ;
              is_target_receive=false;
              return;
          }
          RCLCPP_INFO(node_->get_logger(), "yaw error after adjust : %.3f", yaw_error);
        auto info = &planner_manager_->local_data_;
        info->start_time_ = node_->now();
        publishBspline();
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
      }
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {

      if (planFromCurrentTraj())
      {
          Eigen::Vector3d vel_start = planner_manager_->local_data_.velocity_traj_.evaluateDeBoor(0.1);
          yaw_start = atan2(vel_start(1),vel_start(0));
          yaw_error = yaw_start-yaw;

          auto info = &planner_manager_->local_data_;
          info->start_time_ = node_->now();
          std_msgs::msg::UInt8 stop_cmd;
          stop_cmd.data = 0;
          stop_pub->publish(stop_cmd);
          publishBspline();
          changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
          if (forward_only_) {cmd_pub_->publish(geometry_msgs::msg::Twist());}
          changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      rclcpp::Time time_now = node_->now();
      double t_cur = (time_now - info->start_time_).seconds();
      t_cur = min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      // A ground chassis can lag behind or be stopped by its collision guard.
      // Replanning from a point the robot never reached compounds that error.
      if (max_tracking_error_ > 0 && (odom_pos_-pos).head<2>().norm() > max_tracking_error_)
      {
        changeFSMExecState(REPLAN_TRAJ, "TRACKING");
        return;
      }

      /* && (end_pt_ - pos).norm() < 0.5 */
      if (t_cur > info->duration_ - 1e-2)
      {
        if (max_tracking_error_ > 0 && (odom_pos_-end_pt_).head<2>().norm() > 0.25)
        {
          changeFSMExecState(GEN_NEW_TRAJ, "TRACKING");
          return;
        }
        have_target_ = false;

        changeFSMExecState(WAIT_TARGET, "FSM");
        return;
      }
      else if ((end_pt_ - pos).norm() < no_replan_thresh_)
      {
        // cout << "near end" << endl;
        return;
      }
      else if ((info->start_pos_ - pos).norm() < replan_thresh_)
      {
        // cout << "near start" << endl;
        return;
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case EMERGENCY_STOP:
    {

      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    data_disp_.header.stamp = node_->now();
    data_disp_pub_->publish(data_disp_);
  }

  void EGOReplanFSM::escapeInflation()
  {
    geometry_msgs::msg::Twist command;
    const double age = (node_->now()-escape_cloud_stamp_).seconds();
    if (escape_cloud_stamp_.nanoseconds() == 0 || age < -0.05 || age > 0.5 || escape_obstacles_.empty())
    {
      cmd_pub_->publish(command);
      return;
    }
    const Eigen::Vector2d position = odom_pos_.head<2>();
    auto map = planner_manager_->grid_map_;
    if (map->getInflateOccupancy2d(position) == 0 &&
      obstacleDistance(position, escape_obstacles_) > 0.33)
    {
      cmd_pub_->publish(command);
      if (!planner_manager_->planGlobalTraj(odom_pos_, Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(), end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero())) {return;}
      init_pt_ = odom_pos_;
      have_new_target_ = true;
      is_target_receive = false;
      inflation_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      escape_target_valid_ = false;
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = planning_frame_; marker.ns = "inflation_exit";
      marker.id = 0; marker.action = visualization_msgs::msg::Marker::DELETE;
      escape_marker_pub_->publish(marker);
      RCLCPP_INFO(node_->get_logger(), "Inflation exit reached; replan original goal from measured position");
      changeFSMExecState(GEN_NEW_TRAJ, "ESCAPED");
      return;
    }
    if (escape_target_valid_ &&
      (!escapeSegmentClear(position, escape_target_, escape_obstacles_, escape_hard_radius_) ||
       map->getInflateOccupancy2d(escape_target_) != 0)) {escape_target_valid_ = false;}
    if (!escape_target_valid_)
    {
      escape_target_valid_ = findInflationExit(position, yaw, end_pt_.head<2>(), escape_obstacles_,
        [map](const Eigen::Vector2d & p) {return map->isInMap2d(p) && map->getInflateOccupancy2d(p) == 0;},
        escape_hard_radius_, escape_target_);
    }
    if (!escape_target_valid_)
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
        "No measured collision-free inflation exit; holding position");
      cmd_pub_->publish(command);
      return;
    }
    const auto input = inflationExitCommand(position, yaw, escape_target_, escape_speed_, w_adjust);
    command.linear.x = input.x(); command.angular.z = input.y();
    cmd_pub_->publish(command);

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = planning_frame_; marker.header.stamp = node_->now();
    marker.ns = "inflation_exit"; marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    geometry_msgs::msg::Point from, to;
    from.x = position.x(); from.y = position.y(); from.z = 0.25;
    to.x = escape_target_.x(); to.y = escape_target_.y(); to.z = 0.25;
    marker.points = {from, to}; marker.scale.x = 0.04; marker.scale.y = 0.10; marker.scale.z = 0.13;
    marker.color.r = 1.0; marker.color.g = 0.85; marker.color.a = 1.0;
    marker.lifetime = rclcpp::Duration::from_seconds(0.3);
    escape_marker_pub_->publish(marker);
  }

  bool EGOReplanFSM::planFromCurrentTraj()
  {

    LocalTrajData *info = &planner_manager_->local_data_;
    rclcpp::Time time_now = node_->now();
    double t_cur = std::clamp((time_now - info->start_time_).seconds(), 0.0, info->duration_);

    //cout << "info->velocity_traj_=" << info->velocity_traj_.get_control_points() << endl;

    start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);
    const bool reset_to_odometry = max_tracking_error_ > 0 &&
      (odom_pos_-start_pt_).head<2>().norm() > max_tracking_error_;
    if (reset_to_odometry)
    {
      start_pt_ = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();
    }

    yaw_start = atan2((end_pt_-odom_pos_)(1),(end_pt_-odom_pos_)(0));
    yaw_error = yaw_start-yaw;

    if(is_target_receive)
    {
        if(abs(yaw_error)>PI)
        {
            yaw_error = yaw_error - yaw_error/abs(yaw_error)*2*PI;
        }
        if(!forward_only_ && abs(yaw_error)>PI/2.0)
        {
            if(yaw>0)
            {
                yaw -= PI;
            }else if(yaw<0)
            {
                yaw += PI;
            }
            changeDirection();
            //yaw_error = - yaw_error/abs(yaw_error)*(PI-abs(yaw_error));
            yaw_error = yaw_start - yaw;
            if (!reset_to_odometry) {
              start_vel_ <<-start_vel_(0),-start_vel_(1),0;
              start_acc_ <<-start_acc_(0),-start_acc_(1),0;
            }
            //callEmergencyStop(odom_pos_);
            std_msgs::msg::UInt8 stop_cmd;
            stop_cmd.data = 1;
            stop_pub->publish(stop_cmd);

        }
        is_target_receive = false;
    }
    //first step : calculate the yaw error


    bool success = callReboundReplan(reset_to_odometry, false);

    if (!success)
    {
      success = callReboundReplan(true, false);
      //changeFSMExecState(EXEC_TRAJ, "FSM");
      if (!success)
      {
        success = callReboundReplan(true, true);
        if (!success)
        {
          return false;
        }
      }
    }

    return true;
  }

  void EGOReplanFSM::checkCollisionCallback()
  {
    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;

    if (exec_state_ == WAIT_TARGET || exec_state_ == ESCAPE_INFLATION ||
        (forward_only_ && exec_state_ != EXEC_TRAJ) || info->start_time_.seconds() < 1e-5)
      return;

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    double t_cur = (node_->now() - info->start_time_).seconds();
    double t_2_3 = info->duration_ * 2 / 3;
    // Guard against an inconsistent time base (e.g. a stamp coming from another
    // clock): a t_cur of ~-1e9 would keep the loop below busy for days.
    if (!std::isfinite(t_cur) || !std::isfinite(info->duration_) || t_cur < 0.0 ||
        t_cur > info->duration_ + 1.0)
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "[safety] inconsistent trajectory timing (t_cur=%.3f, duration=%.3f), "
                           "skipping the check - use_sim_time set for every node?",
                           t_cur, info->duration_);
      return;
    }
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      if (t_cur < t_2_3 && t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
        break;

      Eigen::Vector3d pos_cur = info->position_traj_.evaluateDeBoorT(t);
      Eigen::Vector2d pos_cur2d;
      pos_cur2d << pos_cur(0),pos_cur(1);
      if (map->getInflateOccupancy2d(pos_cur2d))
      {
        if (planFromCurrentTraj()) // Make a chance
        {
          // A collision-triggered replan must reach the trajectory server too.
          // Updating local_data_ alone leaves the MPC executing the old spline.
          info->start_time_ = node_->now();
          std_msgs::msg::UInt8 stop_cmd;
          stop_cmd.data = 0;
          stop_pub->publish(stop_cmd);
          publishBspline();
          RCLCPP_INFO(node_->get_logger(), "Collision replan published trajectory %d", info->traj_id_);
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          return;
        }
        else
        {
          if (t - t_cur < emergency_time_) // 0.8s of emergency time
          {
            RCLCPP_WARN(node_->get_logger(), "Suddenly discovered obstacles. emergency stop! time=%f", t - t_cur);
            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          else
          {
            //RCLCPP_WARN(node_->get_logger(), "current traj in collision, replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          return;
        }
        break;
      }
    }
    RCLCPP_DEBUG(node_->get_logger(), "[safety] check finished");
  }

  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    getLocalTarget();
    start_pt_(2) = odom_pos_(2);
    start_vel_(2) = 0;
    start_acc_(2) = 0;
    local_target_pt_(2) = odom_pos_(2);
    local_target_vel_(2) = 0;

    bool plan_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
    have_new_target_ = false;

    cout << "final_plan_success=" << plan_success << endl;

    if (plan_success)
    {
      auto info = &planner_manager_->local_data_;
      //publishBspline();
      Eigen::MatrixXd control_points = info->position_traj_.get_control_points();
      for(int i=0;i<control_points.cols();i++) control_points.col(i)(2) = odom_pos_(2);
      visualization_->displayOptimalList(control_points, 0);
    }

    return plan_success;
  }

  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    ego_planner::msg::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    bspline_pub_->publish(bspline);

    return true;
  }

  void EGOReplanFSM::getLocalTarget()
  {
    double t;

    double t_step = planning_horizen_ / 20 / planner_manager_->pp_.max_vel_;
    double dist_min = 9999, dist_min_t = 0.0;
    for (t = planner_manager_->global_data_.last_progress_time_; t < planner_manager_->global_data_.global_duration_; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      double dist = (pos_t - start_pt_).norm();

      if (t < planner_manager_->global_data_.last_progress_time_ + 1e-5 && dist > planning_horizen_)
      {
        // todo
        RCLCPP_ERROR(node_->get_logger(), "last_progress_time_ ERROR !!!!!!!!!");
        RCLCPP_ERROR(node_->get_logger(), "last_progress_time_ ERROR !!!!!!!!!");
        RCLCPP_ERROR(node_->get_logger(), "last_progress_time_ ERROR !!!!!!!!!");
        RCLCPP_ERROR(node_->get_logger(), "last_progress_time_ ERROR !!!!!!!!!");
        RCLCPP_ERROR(node_->get_logger(), "last_progress_time_ ERROR !!!!!!!!!");
        return;
      }
      if (dist < dist_min)
      {
        dist_min = dist;
        dist_min_t = t;
      }
      if (dist >= planning_horizen_)
      {
        local_target_pt_ = pos_t;
        planner_manager_->global_data_.last_progress_time_ = dist_min_t;
        break;
      }
    }
    if (t > planner_manager_->global_data_.global_duration_) // Last global point
    {
      local_target_pt_ = end_pt_;
    }

    if ((end_pt_ - local_target_pt_).norm() < (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2 * planner_manager_->pp_.max_acc_))
    {
      // local_target_vel_ = (end_pt_ - init_pt_).normalized() * planner_manager_->pp_.max_vel_ * (( end_pt_ - local_target_pt_ ).norm() / ((planner_manager_->pp_.max_vel_*planner_manager_->pp_.max_vel_)/(2*planner_manager_->pp_.max_acc_)));
      // cout << "A" << endl;
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(t);
      // cout << "AA" << endl;
    }
  }

  void EGOReplanFSM::publishBspline() {

      auto info = &planner_manager_->local_data_;
      info->start_time_ = node_->now();
      /* publish traj */
      ego_planner::msg::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      //cout<<"optimal point : "<<endl<<pos_pts<<endl;
      bspline.pos_pts.reserve(pos_pts.cols());
      Eigen::Vector3d point_temp;
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
          geometry_msgs::msg::Point pt;
          pt.x = pos_pts(0, i);
          pt.y = pos_pts(1, i);
          pt.z = odom_pos_(2);
          bspline.pos_pts.push_back(pt);
          point_temp<<pt.x,pt.x,pt.x;
          //cout<<"point : "<<point_temp<<endl;
      }

      Eigen::VectorXd knots = info->position_traj_.getKnot();
      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
          bspline.knots.push_back(knots(i));
      }

      bspline_pub_->publish(bspline);
  }

} // namespace ego_planner
