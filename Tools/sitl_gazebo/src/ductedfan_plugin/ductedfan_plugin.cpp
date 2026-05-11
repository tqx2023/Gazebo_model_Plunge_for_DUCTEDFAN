
#include "ductedfan_plugin/ductedfan_plugin.h" //ductedfan_plugin/ductedfan_plugin.h实际上用的这个
#include <ignition/math.hh>  // Ignition Math 数学库，提供 Vector3、Pose3 等
#include "ductedfan_plugin/spline_ppval.h" // 包含样条插值相关的定义和PpvalSpline函数

// 以下两个为测试时打印变量使用的头文件，以后可以直接删去==
#include <iostream> // 确保包含了 iostream 头文件
#include <chrono> // 如果还没有，请加上这一行
//====================================================

namespace gazebo {

// 析构函数：断开更新事件连接，并关闭 PID 模式
DuctedFanModel::~DuctedFanModel() {
  updateConnection_->~Connection(); // 显式断开连接（实际可以简单 reset）
  use_pid_ = false;
}

// 初始化参数虚函数，这里空实现
void DuctedFanModel::InitializeParams() {}

// 发布电机实际转速（TODO: 当前发布功能被注释掉，防止队列警告）
void DuctedFanModel::Publish() {
  // 获取关节当前速度（rad/s）
  turning_velocity_msg_.set_data(joint_->GetVelocity(0));
  // FIXME: 为防止警告“queue limit reached”，发布被注释。
  // motor_velocity_pub_->Publish(turning_velocity_msg_); //turning_velocity_msg_ 存储要发布的转速消息
}

// 装载插件：解析 SDF 参数，获取仿真对象，初始化通信和滤波器
void DuctedFanModel::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  model_ = _model;

  namespace_.clear();
  // 读取机器人命名空间
  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[ductedfan_plugin model] Please specify a robotNamespace.\n";

  // 初始化 Gazebo 通信节点
  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  // ---------- 读取 SDF 中配置的关节和链接名称 ----------
  if (_sdf->HasElement("jointName"))
    joint_name_ = _sdf->GetElement("jointName")->Get<std::string>();
  else
    gzerr << "[ductedfan_plugin model] Please specify a jointName, where the rotor is attached.\n";
  joint_ = model_->GetJoint(joint_name_);
  if (joint_ == NULL)
    gzthrow("[ductedfan_plugin model] Couldn't find specified joint \"" << joint_name_ << "\".");

  // 配置关节 PID 控制器（可选，用于力控制模式下跟踪期望转速）
  if (_sdf->HasElement("joint_control_pid")) {
    sdf::ElementPtr pid = _sdf->GetElement("joint_control_pid");
    double p = 0.1;
    if (pid->HasElement("p")) p = pid->Get<double>("p");
    double i = 0;
    if (pid->HasElement("i")) i = pid->Get<double>("i");
    double d = 0;
    if (pid->HasElement("d")) d = pid->Get<double>("d");
    double iMax = 0;
    if (pid->HasElement("iMax")) iMax = pid->Get<double>("iMax");
    double iMin = 0;
    if (pid->HasElement("iMin")) iMin = pid->Get<double>("iMin");
    double cmdMax = 3;
    if (pid->HasElement("cmdMax")) cmdMax = pid->Get<double>("cmdMax");
    double cmdMin = -3;
    if (pid->HasElement("cmdMin")) cmdMin = pid->Get<double>("cmdMin");
    pid_.Init(p, i, d, iMax, iMin, cmdMax, cmdMin);
    use_pid_ = true;
  } else {
    use_pid_ = false;  // 默认不使用 PID，直接设定关节速度
  }

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[ductedfan_plugin model] Please specify a linkName of the rotor.\n";
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[ductedfan_plugin model] Couldn't find specified link \"" << link_name_ << "\".");

  // 电机编号
  if (_sdf->HasElement("motorNumber"))
    motor_number_ = _sdf->GetElement("motorNumber")->Get<int>();
  else
    gzerr << "[ductedfan_plugin model] Please specify a motorNumber.\n";

  // 旋转方向
  if (_sdf->HasElement("turningDirection")) {
    std::string turning_direction = _sdf->GetElement("turningDirection")->Get<std::string>();
    if (turning_direction == "cw")
      turning_direction_ = turning_direction::CW;
    else if (turning_direction == "ccw")
      turning_direction_ = turning_direction::CCW;
    else
      gzerr << "[ductedfan_plugin model] Please only use 'cw' or 'ccw' as turningDirection.\n";
  } else {
    gzerr << "[ductedfan_plugin model] Please specify a turning direction ('cw' or 'ccw').\n";
  }

  // 是否允许反转
  if(_sdf->HasElement("reversible")) {
    reversible_ = _sdf->GetElement("reversible")->Get<bool>();
  }

  // ---------- 加载涵道气动样条系数（dt, dn） ----------
  if (_sdf->HasElement("ductSplineDtFile")) {
      std::string dt_path = _sdf->GetElement("ductSplineDtFile")->Get<std::string>();
      if (!LoadSplineFromCsv(dt_path, spline_dt_)) {
          gzerr << "[ductedfan_plugin] 加载涵道拉力系数样条失败，文件: " << dt_path << "\n";
      }
  } else {
      gzerr << "[ductedfan_plugin] 未指定 ductSplineDtFile，将不使用样条修正推力\n";
  }

  if (_sdf->HasElement("ductSplineDnFile")) {
      std::string dn_path = _sdf->GetElement("ductSplineDnFile")->Get<std::string>();
      if (!LoadSplineFromCsv(dn_path, spline_dn_)) {
          gzerr << "[ductedfan_plugin] 加载涵道侧向力系数样条失败，文件: " << dn_path << "\n";
      }
  } else {
      gzerr << "[ductedfan_plugin] 未指定 ductSplineDnFile，侧向力将不采用样条\n";
  }

  // ---------- 加载机翼气动样条系数（wl, wd, wm） ----------
  if (_sdf->HasElement("wingSplineWlFile")) {
      std::string wl_path = _sdf->GetElement("wingSplineWlFile")->Get<std::string>();
      if (!LoadSplineFromCsv(wl_path, wing_spline_wl_))
          gzerr << "[ductedfan_plugin] 加载机翼升力样条失败！\n";
  } else {
      gzerr << "[ductedfan_plugin] 未指定 wingSplineWlFile\n";
  }
  // 同样加载 wd 和 wm
  if (_sdf->HasElement("wingSplineWdFile")) {
      std::string wd_path = _sdf->GetElement("wingSplineWdFile")->Get<std::string>();
      if (!LoadSplineFromCsv(wd_path, wing_spline_wd_))
          gzerr << "[ductedfan_plugin] 加载机翼阻力样条失败！\n";
  } else {
      gzerr << "[ductedfan_plugin] 未指定 wingSplineWdFile\n";
  }
  if (_sdf->HasElement("wingSplineWmFile")) {
      std::string wm_path = _sdf->GetElement("wingSplineWmFile")->Get<std::string>();
      if (!LoadSplineFromCsv(wm_path, wing_spline_wm_))
          gzerr << "[ductedfan_plugin] 加载机翼俯仰力矩样条失败！\n";
  } else {
      gzerr << "[ductedfan_plugin] 未指定 wingSplineWmFile\n";
  }

  // ---------- 使用 common.h 中的辅助函数读取可选参数 ----------
  getSdfParam<std::string>(_sdf, "commandSubTopic", command_sub_topic_, command_sub_topic_);
  getSdfParam<std::string>(_sdf, "motorSpeedPubTopic", motor_speed_pub_topic_, motor_speed_pub_topic_);

  getSdfParam<double>(_sdf, "rotorDragCoefficient", rotor_drag_coefficient_, rotor_drag_coefficient_);
  getSdfParam<double>(_sdf, "rollingMomentCoefficient", rolling_moment_coefficient_, rolling_moment_coefficient_);
  getSdfParam<double>(_sdf, "maxRotVelocity", max_rot_velocity_, max_rot_velocity_);
  getSdfParam<double>(_sdf, "motorConstant", motor_constant_, motor_constant_);
  getSdfParam<double>(_sdf, "momentConstant", moment_constant_, moment_constant_);

  getSdfParam<double>(_sdf, "timeConstantUp", time_constant_up_, time_constant_up_);
  getSdfParam<double>(_sdf, "timeConstantDown", time_constant_down_, time_constant_down_);
  getSdfParam<double>(_sdf, "rotorVelocitySlowdownSim", rotor_velocity_slowdown_sim_, 10);

  getSdfParam<double>(_sdf, "thrustVelocityCoupling", k_Th_, 0.0);
  getSdfParam<double>(_sdf, "thrustAoaCoupling", k_Ts_, 0.0);
  getSdfParam<double>(_sdf, "sideForceVelocityCoupling", k_Ns_, 0.0);
  getSdfParam<double>(_sdf, "sideForceArmZ", l_cpz_, 0.0);
  getSdfParam<double>(_sdf, "thrustCenterOffsetCoeff", k_cpx_, 0.0);


  fly = false;  // 某个未使用的标志

  // 在 Gazebo 5 以前的版本，设置关节最大力（以后版本不再使用此接口）
#if GAZEBO_MAJOR_VERSION < 5
  joint_->SetMaxForce(0, max_force_);
#endif

  // 连接世界更新事件：每个仿真步调用 OnUpdate
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&DuctedFanModel::OnUpdate, this, _1));

  // 订阅命令电机转速话题（话题格式：~/模型名/command_sub_topic_）
  command_sub_ = node_handle_->Subscribe<mav_msgs::msgs::CommandMotorSpeed>(
      "~/" + model_->GetName() + command_sub_topic_, &DuctedFanModel::VelocityCallback, this);

  // 订阅电机故障话题
  motor_failure_sub_ = node_handle_->Subscribe<msgs::Int>(
      motor_failure_sub_topic_, &DuctedFanModel::MotorFailureCallback, this);

  // 发布实际电机转速话题（当前发布被注释，但发布者对象仍可创建）
  // motor_velocity_pub_ = node_handle_->Advertise<std_msgs::msgs::Float>("~/" + model_->GetName() + motor_speed_pub_topic_, 1);

  // 订阅世界风话题
  wind_sub_ = node_handle_->Subscribe("~/" + wind_sub_topic_, &DuctedFanModel::WindVelocityCallback, this);

  // 初始化一阶滤波器：使用加速/减速时间常数，初始参考转速为0
  rotor_velocity_filter_.reset(new FirstOrderFilter<double>(time_constant_up_, time_constant_down_, ref_motor_rot_vel_));
}

// ======== 以下为各种回调 ========

// 世界更新开始回调
void DuctedFanModel::OnUpdate(const common::UpdateInfo& _info) {
  // 计算采样时间（当前仿真时间 - 上一仿真时间）
  sampling_time_ = _info.simTime.Double() - prev_sim_time_;
  prev_sim_time_ = _info.simTime.Double();

  // 执行力和力矩计算（物理更新）
  UpdateForcesAndMoments();
  // 检查电机故障并处理
  UpdateMotorFail();
  // 发布实际转速
  Publish();
}

// 接收电机转速命令
void DuctedFanModel::VelocityCallback(CommandMotorSpeedPtr &rot_velocities) {
  // 检查数组大小是否足够
  if(rot_velocities->motor_speed_size() < motor_number_) {
    std::cout << "You tried to access index " << motor_number_
              << " of the MotorSpeed message array which is of size "
              << rot_velocities->motor_speed_size() << "." << std::endl;
  } else {
    // 将期望转速限制在最大转速内，然后赋值给 ref_motor_rot_vel_
    ref_motor_rot_vel_ = std::min(
        static_cast<double>(rot_velocities->motor_speed(motor_number_)),
        static_cast<double>(max_rot_velocity_));
  }
}

// 接收电机故障消息
void DuctedFanModel::MotorFailureCallback(const boost::shared_ptr<const msgs::Int> &fail_msg) {
  motor_Failure_Number_ = fail_msg->data();
}

// 接收风速消息
void DuctedFanModel::WindVelocityCallback(WindPtr& msg) {
  wind_vel_ = ignition::math::Vector3d(
      msg->velocity().x(),
      msg->velocity().y(),
      msg->velocity().z());
}

// ======== 核心：更新作用在旋翼上的力和力矩 ========
void DuctedFanModel::UpdateForcesAndMoments() {
  // 获取当前关节角速度（rad/s）
  motor_rot_vel_ = joint_->GetVelocity(0);

  // 检查混叠风险：如果仿真步长太大导致转速过高，可能无法准确捕捉旋转
  if (motor_rot_vel_ / (2 * M_PI) > 1 / (2 * sampling_time_)) {
    gzerr << "Aliasing on motor [" << motor_number_
          << "] might occur. Consider making smaller simulation time steps or raising the rotor_velocity_slowdown_sim_ param.\n";
  }

  // 将关节速度放大，映射回物理真实转速。因为仿真中为了避免过高频率而减慢了旋转速度。
  double real_motor_velocity = motor_rot_vel_ * rotor_velocity_slowdown_sim_;  //风扇转速O
  double O = real_motor_velocity;

  // 获取旋翼刚体在世界坐标系下的线速度
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d body_velocity = link_->WorldLinearVel();
  ignition::math::Vector3d joint_axis = joint_->GlobalAxis(0);
  ignition::math::Pose3d link_pose = link_->WorldPose();
#else
  ignition::math::Vector3d body_velocity = ignitionFromGazeboMath(link_->GetWorldLinearVel());
  ignition::math::Vector3d joint_axis = ignitionFromGazeboMath(joint_->GetGlobalAxis(0));
  ignition::math::Pose3d link_pose = ignitionFromGazeboMath(link_->GetWorldPose());
#endif

  // 计算相对风速 = 机体速度 - 世界风速
  ignition::math::Vector3d relative_wind_velocity = body_velocity - wind_vel_;
  // 机体系下的相对风速（旋翼坐标系下的风速）Va = Rn2b*[ (Vx - Dx) ; (Vy - Dy) ; (Vz - Dz) ];
  ignition::math::Vector3d wind_body = link_pose.Rot().RotateVectorReverse(relative_wind_velocity); 
  // （ENU坐标系下的Va速度）
  
  double u_enu = wind_body.X();   // 本体前向x
  double v_enu = wind_body.Y();   // 本体侧向y
  double w_enu = wind_body.Z();   // 本体轴向z (z向上为正)
  
  //  转换为 MATLAB 坐标系（x→v, y→u, z→-w）
  double u_mat = v_enu;   //侧向 x
  double v_mat = u_enu;   //前向 y
  double w_mat = -w_enu;  //轴向 z(z轴向下为正)

  //--------------------------------------------------------
    //空速、迎角（机体）
    
  double Vvw_mat = sqrt(v_mat*v_mat + w_mat*w_mat);  // yz-plane空速分量
  double Vuv_mat = sqrt(u_mat*u_mat + v_mat*v_mat);  // xy-plane空速分量
  const double Vuv_mat_min = 1e-12;

  double AOA = 0.0;
  if (Vvw_mat <= 1.0) { 
    AOA = M_PI / 2.0;
  } else {
    double cosA = -w_mat / Vvw_mat;
    cosA = ignition::math::clamp(cosA, -1.0, 1.0);
    AOA = acos(cosA);
  }
  double Sduct = 0.0, Cduct = 1.0;
  if (Vuv_mat > Vuv_mat_min) {
    Sduct = u_mat / Vuv_mat;
    Cduct = v_mat / Vuv_mat;
  }
  //--------------------------------------------------------


  // =================Ducted_Fan_FnM函数部分=====================
  // 输入：转速 O，来流速度分量 u_mat, v_mat, w_mat
  // 输出：推力 Ducted_T，侧向力 Ducted_N，俯仰力矩 Ducted_M，反扭矩 Ducted_Q
  
  
  // 计算DuctFAN_AOA迎角（论文中的a_df）
  double Ducted_Vuv = sqrt(u_mat*u_mat + v_mat*v_mat);  // 气流沿涵道平面横向扫过的速度大小
  double Ducted_Va_body  = sqrt( u_mat*u_mat + v_mat*v_mat + w_mat*w_mat);   // 机体系下相对速度的总大小

  const double Ducted_Va_min = 1e-6;  // 避免除零

  double Ducted_csAOA, Ducted_sAOA, Ducted_AOA;

   if (Ducted_Va_body <= Ducted_Va_min) {
    Ducted_AOA   = M_PI / 2.0;
    Ducted_csAOA = 0.0;
    Ducted_sAOA  = 1.0;
  } else {
    Ducted_csAOA = -w_mat / Ducted_Va_body;
    Ducted_csAOA = ignition::math::clamp(Ducted_csAOA, -1.0, 1.0); // 防止数值误差导致 acos 出错
    Ducted_AOA   = acos(Ducted_csAOA);                             // 相对速度方向与机体 z 轴方向之间的夹角 来流角a_df
    Ducted_sAOA  = Ducted_Vuv / Ducted_Va_body;
  }

  // ========== 样条插值获取 dt, dn ==========
  double dt = 0.0, dn = 0.0;
  if (!spline_dt_.empty()) dt = PpvalSpline(spline_dt_, Ducted_AOA, true);
  if (!spline_dn_.empty()) dn = PpvalSpline(spline_dn_, Ducted_AOA, true);

  // ========== 力与力矩计算===================
  double Ducted_O2  = O * O;
  double Ducted_VaO = Ducted_Va_body * O;

  // 推力 T (MATLAB y(1))，沿机体 z 轴（注意：Gazebo 中转子局部 z 轴向上）
  double Ducted_T = motor_constant_ * Ducted_O2                     // k_T0 * ω²
           + Ducted_VaO * (k_Th_ + k_Ts_ * Ducted_csAOA)           // 速度‑迎角耦合项
           + Ducted_Va_body * Ducted_Va_body * dt;                 // 纯气动项

  // 如果不允许反向推力，取绝对值（保证推力向上）
  if(!reversible_) {
    Ducted_T = std::abs(Ducted_T);
  }

  // 侧向力 N (MATLAB y(2))，大小，方向在 xy 平面内与 (u_mat, v_mat) 一致
  double Ducted_N = Ducted_VaO * k_Ns_ * Ducted_sAOA + Ducted_Va_body * Ducted_Va_body * dn;

   // 俯仰力矩 M (MATLAB y(3))，在 MATLAB 坐标系下绕 x 轴（侧向轴）
  double Ducted_M = 0.0;
  if (O > 0.0) {
    Ducted_M = Ducted_N * l_cpz_ + Ducted_T * k_cpx_ * Ducted_Va_body / O * Ducted_sAOA;
  } else {
    Ducted_M = Ducted_N * l_cpz_;
  }

 // 反扭矩 Q (MATLAB y(4))，绕旋转轴（z 轴）
  double Ducted_Q = moment_constant_ * Ducted_O2;   // moment_constant_ 即 k_Q0
  
  
  
  // =================Ducted_Fan_FnM函数部分=====================



  // =================Wing_FnM函数部分===========================
  // 机翼样条插值
  double wl = 0.0, wd = 0.0, wm = 0.0;
  if (!wing_spline_wl_.empty()) wl = PpvalSpline(wing_spline_wl_, AOA, true);
  if (!wing_spline_wd_.empty()) wd = PpvalSpline(wing_spline_wd_, AOA, true);
  if (!wing_spline_wm_.empty()) wm = PpvalSpline(wing_spline_wm_, AOA, true);

  double Vvw2 = Vvw_mat * Vvw_mat;
  double Wing_Fy = Vvw2 * wl;   // 力在MATLAB的前向 y 轴分量
  double Wing_Fz = Vvw2 * wd;   // 力在MATLAB的轴向 z 轴分量 (向下为正)
  double Wing_My = Vvw2 * wm;   // 力矩绕MATLAB的侧向 x 轴
// =================Wing_FnM函数部分===========================




  // ================= 调试打印部分 =========================
  // 使用 static 保证变量在多次调用中保持值，用于记录上次打印时间
  // DEBUG_PRINT 可以手动控制是否启用调试打印
  const bool DEBUG_PRINT = true; // 手动控制
  if (DEBUG_PRINT){
    static std::chrono::steady_clock::time_point last_print_time = std::chrono::steady_clock::now();
    auto current_time = std::chrono::steady_clock::now();

    // 计算流逝时间（毫秒）
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_print_time).count();

    // 设定间隔阈值：1000 毫秒 (1秒)
    if (elapsed_ms >= 1000) {
        std::cout << "--- Debug Info [Motor " << motor_number_ << "] ---" << std::endl;
        std::cout << "motor_rot_vel_: " << motor_rot_vel_ << "\n"
          << "real_motor_velocity: " << real_motor_velocity << "\n"
          << "AOA(deg): " << AOA * 180.0 / M_PI
          << " wl:" << wl << " wd:" << wd << " wm:" << wm << "\n"
          << "Ducted_AOA(deg): " << Ducted_AOA * 180.0 / M_PI
          << " dt:" << dt << " dn:" << dn << "\n"
          << "Ducted_T: " << Ducted_T << " Ducted_N: " << Ducted_N
          << " Ducted_M: " << Ducted_M << " Ducted_Q: " << Ducted_Q << "\n"
          << "Wing_Fy: " << Wing_Fy << " Wing_Fz: " << Wing_Fz << " Wing_My: " << Wing_My << std::endl;
        std::cout << "----------------------------------" << std::endl;

        // 更新时间戳
        last_print_time = current_time;
    }
  }
  // =======================================================
}

// ======== 电机故障模拟 ========
void DuctedFanModel::UpdateMotorFail() {
  // 如果故障编号等于本电机编号+1（故障编号从1开始），则将该电机转速强制设为0
  if (motor_number_ == motor_Failure_Number_ - 1) {
    // 另一种可能的方法是将电机常数设为0，但这里直接锁定关节速度
    joint_->SetVelocity(0, 0);
    if (screen_msg_flag) {
      // 以防消息刷屏，只打印一次故障信息
      std::cout << "Motor number [" << motor_Failure_Number_ << "] failed!  [Motor thrust = 0]" << std::endl;
      tmp_motor_num = motor_Failure_Number_;
      screen_msg_flag = 0; // 已打印过故障信息
    }
  } else if (motor_Failure_Number_ == 0 && motor_number_ == tmp_motor_num - 1) {
    // 当故障编号重新置零且本电机就是之前故障的电机，打印恢复信息
    if (!screen_msg_flag) {
      std::cout << "Motor number [" << tmp_motor_num << "] running! [Motor thrust = (default)]" << std::endl;
      screen_msg_flag = 1; // 恢复标志，允许下次再打印
    }
  }
}

// 注册插件：让 Gazebo 知道这个模型插件可以加载
GZ_REGISTER_MODEL_PLUGIN(DuctedFanModel);
} // namespace gazebo
