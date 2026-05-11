### 1. 级联结构

PX4 多旋翼控制器可概括为四层级联：

`trajectory_setpoint -> position P -> velocity PID -> thrust vector -> attitude quaternion P -> rate PID -> torque/thrust setpoint -> allocator -> motors`

上传源码中，`MulticopterPositionControl` 先运行 `_control.update(dt)`，再发布 `vehicle_local_position_setpoint` 与 `vehicle_attitude_setpoint`；位置环增益、速度环 PID 增益、水平推力裕度、横纵加速度解耦选项均由参数写入 `PositionControl`。 

### 2. 位置环：位置误差生成速度设定

设 `p` 为当前位置，`p_sp` 为位置设定，`v_ff` 为轨迹前馈速度，`v_sp` 为速度设定，`K_pos` 为位置比例增益。PX4 的位置环为纯 P 控制：

$$
v_p = K_{pos}(p_{sp} - p)
$$

$$
v_{sp} = sat_v(v_{ff} + v_p)
$$

其中水平面采用 `constrainXY`，优先保留由位置误差产生的速度分量；竖直方向限制在上升/下降速度限幅内。源码对应 `_positionControl()` 中 `vel_sp_position = (_pos_sp - _pos).emult(_gain_pos_p)`，随后将位置 P 项叠加到速度设定，并进行水平与竖直速度约束。

### 3. 速度环：PID 生成期望加速度

设 `v` 为估计速度，`v_dot` 为估计加速度，`i_v` 为速度积分项，`a_ff` 为轨迹前馈加速度，`a_sp` 为期望加速度。速度误差为：

$$
e_v = v_{sp} - v
$$

速度 PID 给出：

$$
a_{pid} = K_{vp}e_v + i_v - K_{vd}v_{dot}
$$

$$
a_{sp} = a_{ff} + a_{pid}
$$

积分更新为：

$$
i_v^+ = i_v + K_{vi}e_v dt
$$

源码中 `_velocityControl()` 对竖直积分项作重力尺度限幅，随后计算 `vel_error` 与 `acc_sp_velocity`，并将其叠加到 `_acc_sp`；推力饱和后，竖直方向冻结积分，水平方向采用 tracking anti-windup 修正误差。 

### 4. 加速度到推力向量：外环的关键几何映射

PX4 使用 NED 坐标，`z` 向下，机体系推力 `thrust_body[2]` 为负值。设 `g` 为重力加速度，`T_h` 为悬停推力参数，`b_z` 为期望机体系 z 轴在世界系中的方向，`T_sp` 为归一化推力向量。源码中的核心映射为：

$$
s_z = -g + \beta a_{sp,z}
$$

$$
b_z = \frac{[-a_{sp,x}, -a_{sp,y}, -s_z]}{||[-a_{sp,x}, -a_{sp,y}, -s_z]||}
$$

$$
T_z = a_{sp,z}\frac{T_h}{g} - T_h
$$

$$
u_T = min(\frac{T_z}{e_3^T b_z}, -T_{min})
$$

$$
T_{sp} = u_T b_z
$$

其中 `beta = 1` 表示横纵加速度耦合，`beta = 0` 表示启用横纵解耦；`e_3 = [0,0,1]`。源码对应 `_accelerationControl()`：先构造 `body_z`，再经 `limitTilt` 限制倾角，最后由悬停推力模型计算 `thrust_ned_z`、`collective_thrust` 与 `_thr_sp`。 倾角限制本质为把期望机体 z 轴投影到世界 z 轴附近的圆锥内：夹角 `angle = acos(body_unit dot world_unit)`，再以最大倾角截断。

### 5. 推力向量到姿态设定

给定推力向量 `T_sp` 和偏航设定 `psi_sp`，PX4 先令期望机体 z 轴与反推力方向一致：

$$
b_{z,d} = \frac{-T_{sp}}{||T_{sp}||}
$$

再由偏航方向构造：

$$
y_c = [-sin(psi_{sp}), cos(psi_{sp}), 0]
$$

$$
b_{x,d} = \frac{y_c \times b_{z,d}}{||y_c \times b_{z,d}||}
$$

$$
b_{y,d} = b_{z,d} \times b_{x,d}
$$

$$
R_d = [b_{x,d}, b_{y,d}, b_{z,d}]
$$

$$
q_d = quat(R_d)
$$

源码中 `thrustToAttitude()` 调用 `bodyzToAttitude(-thr_sp, yaw_sp, att_sp)`，并设置 `thrust_body[2] = -||thr_sp||`；`bodyzToAttitude()` 由 `body_z`、`y_C`、叉乘构造 `body_x/body_y`，再填充旋转矩阵并转为四元数。 

### 6. 姿态环：四元数误差生成角速度设定

设当前姿态四元数为 `q`，期望姿态为 `q_d`，角速度设定为 `omega_sp`。PX4 先用当前推力轴 `e_z = R(q)e_3` 与期望推力轴 `e_z_d = R(q_d)e_3` 构造 reduced attitude，以优先控制 roll/pitch；随后提取剩余 yaw 分量，用 `yaw_weight` 缩放，再重组成新的 `q_d`。

四元数误差为：

$$
q_e = q^{-1}q_d
$$

姿态误差向量取 canonical quaternion 的虚部：

$$
e_q = 2 q_{e,xyz}
$$

角速度设定为：

$$
omega_{sp} = K_q e_q + R(q)^T e_3 \dot{\psi}_{sp}
$$

随后逐轴限幅。源码中 `eq = 2.f * qe.canonical().imag()`，`rate_setpoint = eq.emult(_proportional_gain)`，偏航角速度前馈以 `q.inversed().dcm_z() * _yawspeed_setpoint` 加入机体系角速度设定。

### 7. 角速度环：角速度误差到力矩设定

角速度环输入为 `rates`、`rates_setpoint`、角加速度估计 `angular_accel` 与 `dt`，输出为 `torque_setpoint`。上传的 `MulticopterRateControl.cpp` 显示 PX4 将 `MC_*RATE_K` 与 P/I/D 参数相乘后传入 `_rate_control.setPidGains()`，并设置积分限幅与前馈增益；注释说明该增益 `K` 用于把并联 PID 形式转换为理想 PID 形式。

其抽象形式为：

$$
e_\omega = \omega_{sp} - \omega
$$

$$
\tau_{sp} = K_{\omega p}e_\omega + i_\omega - K_{\omega d}\dot{\omega} + K_{\omega ff}\omega_{sp}
$$

控制分配反馈会设置正/负饱和标志，用于角速度环 anti-windup；随后 `_rate_control.update(...)` 输出 `torque_setpoint`，yaw 轴力矩再经低通滤波。 最终发布 `vehicle_thrust_setpoint` 与 `vehicle_torque_setpoint`，控制分配模块再把总推力与三轴力矩映射到电机。

### 8. 手动稳定模式的姿态设定

手动稳定模式下，摇杆 roll/pitch 被映射为一个二维倾角向量 `v = [roll, -pitch] * tilt_max`，其模长对应倾角，方向对应水平运动方向；超过最大倾角时按模长缩放。随后以 `AxisAngle(v_x, v_y, 0)` 构造 roll/pitch 四元数，再左乘 yaw 四元数得到姿态设定。 油门杆通过分段插值映射为归一化推力，可选以悬停推力参数或悬停推力估计值作为中点。

### 小结

PX4 多旋翼控制器的数学核心是“平动控制给出期望比力，期望比力定义推力方向，推力方向与偏航角定义姿态，姿态四元数误差定义角速度，角速度 PID 定义机体系力矩”。其外环近似为线性 P/PID，姿态环采用四元数几何控制，执行层使用归一化推力与归一化力矩接口，后续由控制分配器处理电机混控。
