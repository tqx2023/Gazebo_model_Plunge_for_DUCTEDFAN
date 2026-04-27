
#include "ductedfan_plugin/ductedfan_plugin.h" //ductedfan_plugin/ductedfan_plugin.h实际上用的这个
#include <ignition/math.hh>  // Ignition Math 数学库，提供 Vector3、Pose3 等

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
  double real_motor_velocity = motor_rot_vel_ * rotor_velocity_slowdown_sim_;

  // 计算推力：F = 电机常数 * |omega| * omega（推力与转速平方成正比，方向由正负号决定）
  double force = real_motor_velocity * std::abs(real_motor_velocity) * motor_constant_;

  // 如果不允许反向推力，取绝对值（保证推力向上）
  if(!reversible_) {
    force = std::abs(force);
  }

  // ******************* 推力随前进比衰减 *******************
  // 获取旋翼刚体在世界坐标系下的线速度
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d body_velocity = link_->WorldLinearVel();
  ignition::math::Vector3d joint_axis = joint_->GlobalAxis(0);
#else
  ignition::math::Vector3d body_velocity = ignitionFromGazeboMath(link_->GetWorldLinearVel());
  ignition::math::Vector3d joint_axis = ignitionFromGazeboMath(joint_->GetGlobalAxis(0));
#endif

  // 计算相对风速 = 机体速度 - 世界风速
  ignition::math::Vector3d relative_wind_velocity = body_velocity - wind_vel_;

  // 相对风速投影到旋翼旋转轴上的分量（平行分量）
  ignition::math::Vector3d velocity_parallel_to_rotor_axis =
      (relative_wind_velocity.Dot(joint_axis)) * joint_axis;
  double vel = velocity_parallel_to_rotor_axis.Length();// 沿轴向速度的模长

  // 当轴向速度达到 25 m/s 时，推力线性减小到 0
  double scalar = 1 - vel / 25.0;
  scalar = ignition::math::clamp(scalar, 0.0, 1.0); // 将衰减因子限制在 [0, 1] 范围内

  // 在旋翼刚体上施加推力（相对于刚体坐标系 z 轴向上）
  link_->AddRelativeForce(ignition::math::Vector3d(0, 0, force * scalar));

  // ******************* 旋翼叶片阻力（水平方向） *******************
  // 计算相对风速的垂直分量（垂直于旋转轴）
  ignition::math::Vector3d velocity_perpendicular_to_rotor_axis =
      relative_wind_velocity - velocity_parallel_to_rotor_axis;

  // 空气阻力公式： - |omega| * 阻力系数 * 垂直风速分量
  ignition::math::Vector3d air_drag =
      -std::abs(real_motor_velocity) * rotor_drag_coefficient_ * velocity_perpendicular_to_rotor_axis;
  // 将该阻力直接施加在旋翼刚体上
  link_->AddForce(air_drag);

  // ******************* 扭矩计算 *******************
  // 获取旋翼刚体的父刚体（通常是机臂连接点），因为扭矩通常作用在父刚体上
  physics::Link_V parent_links = link_->GetParentJointsLinks();  // 获取父链接列表，通常只有一个父链接
                                                                 // 也可以通过joint_->GetParent()获取父链接，
                                                                 // 但这里直接获取父链接列表更通用
  // 计算旋翼相对于父刚体的位姿差（用于将旋翼坐标系下的扭矩转换到父刚体坐标系）
  //在 Gazebo 的数学库中，两个 Pose 相减，绝不是简单的坐标分量代数相减，
  //而是运算符重载（Operator Overloading）后的相对位姿解算。
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Pose3d pose_difference = link_->WorldCoGPose() - parent_links.at(0)->WorldCoGPose();
#else
  ignition::math::Pose3d pose_difference = ignitionFromGazeboMath(link_->GetWorldCoGPose() - parent_links.at(0)->GetWorldCoGPose());
#endif
  // 由于旋翼叶片的阻力产生的力矩$\boldsymbol{M}_{D}：与旋转方向相关，用于抵消螺旋桨加速时的反作用力矩
  ignition::math::Vector3d drag_torque(0, 0, -turning_direction_ * force * moment_constant_);

  // 将旋翼本体系的扭矩转换到父刚体坐标系
  ignition::math::Vector3d drag_torque_parent_frame = pose_difference.Rot().RotateVector(drag_torque);
  parent_links.at(0)->AddRelativeTorque(drag_torque_parent_frame);

  // 滚转力矩（由于不对称气流产生的恢复力矩）
  ignition::math::Vector3d rolling_moment;
  rolling_moment = -std::abs(real_motor_velocity) * rolling_moment_coefficient_ * velocity_perpendicular_to_rotor_axis;
  parent_links.at(0)->AddTorque(rolling_moment);

  // ******************* 速度控制（设定关节速度） *******************
  // 通过一阶滤波器平滑期望转速，得到滤波后的参考值
  // 这里的 ref_motor_rot_vel_ 是从命令话题接收的期望转速，
  // 经过滤波器处理后得到更平滑的 ref_motor_rot_vel
  //sampling_time_ 是当前仿真步长，用于滤波器的时间更新

  double ref_motor_rot_vel;
  ref_motor_rot_vel = rotor_velocity_filter_->updateFilter(ref_motor_rot_vel_, sampling_time_);

    // PID 模式被屏蔽，通常直接设置关节速度（近似忽略电机瞬态响应，实际已由滤波器处理）
#if 0 //FIXME: disable PID for now, it does not play nice with the PX4 CI system.
  if (use_pid_) {
    double err = joint_->GetVelocity(0) - turning_direction_ * ref_motor_rot_vel / rotor_velocity_slowdown_sim_;
    double rotorForce = pid_.Update(err, sampling_time_);
    joint_->SetForce(0, rotorForce);
  } else {
    // Gazebo 7+ 直接设置速度
    joint_->SetVelocity(0, turning_direction_ * ref_motor_rot_vel / rotor_velocity_slowdown_sim_);
  }
#else
  // 最终关节速度 = 旋转方向 * 滤波后转速 / 仿真减速比
  joint_->SetVelocity(0, turning_direction_ * ref_motor_rot_vel / rotor_velocity_slowdown_sim_);
#endif

  // 调试打印（当 if (0) 时不执行，可手动设为 1 查看）
  /*
  if (0) {
    gzdbg << "motor_rot_vel_: " << motor_rot_vel_ << "\n";
    gzdbg << "real_motor_velocity: " << real_motor_velocity << "\n";
    gzdbg << "force * scalar: " << force * scalar << "\n";
    gzdbg << "air_drag: " << air_drag << "\n";
    gzdbg << "drag_torque: " << drag_torque << "\n";
    gzdbg << "rolling_moment: " << rolling_moment << "\n";
    gzdbg << "rolling_moment_coefficient_: " << rolling_moment_coefficient_ << "\n";
    gzdbg << "rotor_drag_coefficient_: " << rotor_drag_coefficient_ << "\n";
    gzdbg << "motor_constant_: " << motor_constant_ << "\n";
    gzdbg << "moment_constant_: " << moment_constant_ << "\n";
    gzdbg << "scalar: " << scalar << "\n";
  }
  */
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
                  << "force * scalar: " << force * scalar << "\n"
                  << "air_drag: " << air_drag << "\n"
                  << "drag_torque: " << drag_torque << "\n"
                  << "rolling_moment: " << rolling_moment << "\n"
                  << "scalar: " << scalar << std::endl;
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
