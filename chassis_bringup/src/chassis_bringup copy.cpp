#include "turn_on_wheeltec_robot/chassis_bringup.h"
#include "turn_on_wheeltec_robot/Quaternion_Solution.h"

#include <algorithm>
#include <cmath>

sensor_msgs::msg::Imu Mpu6050; // 实例化IMU对象 

using std::placeholders::_1;
using namespace std;
rclcpp::Node::SharedPtr node_handle = nullptr;

/**
 * @brief 主函数，ROS2初始化，创建Robot_control对象并开始循环。
 */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv); // ROS2初始化
  node_handle = std::make_shared<turn_on_robot>(); // 以shared_ptr方式创建节点
  auto robot_control = std::dynamic_pointer_cast<turn_on_robot>(node_handle);
  robot_control->Control();     // 循环执行数据采集和发布话题
  return 0;
}

/**
 * @brief 将两个八位数据合并为十六位有符号短整型（常用于IMU等原始数据转换）。
 * @param Data_High 高八位数据
 * @param Data_Low 低八位数据
 * @return short 转换后的16位有符号整数
 */
short turn_on_robot::IMU_Trans(uint8_t Data_High, uint8_t Data_Low)
{
  // 使用位移和按位或运算合并数据，并强制转换为short以保留符号位
  return (short)((Data_High << 8) | Data_Low);
}

/**
 * @brief 将来自下位机的里程计数据（两个字节）转换为m/s单位的浮点数。
 * @details 原始数据单位为mm/s。
 * @param Data_Low 低八位数据
 * @param Data_High 高八位数据
 * @return float 转换后的速度，单位m/s
 */
float turn_on_robot::Odom_Trans(uint8_t Data_High, uint8_t Data_Low)
{
  // 1. 将位操作结果强制转换为有符号短整型以保留正负号
  short data_short = (short)((Data_High << 8) | Data_Low);
  
  // 2. 原始单位 mm/s 转换为 m/s，直接除以 1000.0f
  float data_float = data_short / 1000.0f;
  
  return data_float;
}

/**
 * @brief 速度话题订阅回调函数，根据订阅的指令通过串口控制下位机。
 * @param twist_aux 订阅到的Twist消息指针
 */
void turn_on_robot::Cmd_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux)
{
  // 回调只更新缓存和时间戳，真正的串口发送由 watchdog 周期统一裁决。
  latest_cmd_vel_ = Sanitize_Command(*twist_aux);
  last_cmd_time_ = steady_clock_.now();
  has_cmd_vel_ = true;
}

geometry_msgs::msg::Twist turn_on_robot::Sanitize_Command(const geometry_msgs::msg::Twist & cmd_vel)
{
  geometry_msgs::msg::Twist safe_cmd = cmd_vel;
  auto sanitize_axis = [](double value, double limit) {
    // 上层异常时可能出现 NaN/Inf，这类值不能进入串口协议编码。
    if (!std::isfinite(value)) {
      return 0.0;
    }
    // 限幅参数 <= 0 时表示不限制该轴，便于按车型逐步开启。
    if (limit > 0.0) {
      return std::max(-limit, std::min(limit, value));
    }
    return value;
  };

  safe_cmd.linear.x = sanitize_axis(cmd_vel.linear.x, max_linear_x_);
  safe_cmd.linear.y = sanitize_axis(cmd_vel.linear.y, max_linear_y_);
  safe_cmd.linear.z = 0.0;
  safe_cmd.angular.x = 0.0;
  safe_cmd.angular.y = 0.0;
  safe_cmd.angular.z = sanitize_axis(cmd_vel.angular.z, max_angular_z_);
  return safe_cmd;
}

void turn_on_robot::Send_Velocity_Command(const geometry_msgs::msg::Twist & cmd_vel)
{
  short transition; // 中间变量

  Send_Data.tx[0] = FRAME_HEADER; // 帧头 0x7B
  Send_Data.tx[1] = 0;            // 预留位
  Send_Data.tx[2] = 0;            // 预留位

  // X轴目标线速度转换 (m/s -> mm/s)
  transition = (short)(cmd_vel.linear.x * 1000);
  Send_Data.tx[4] = transition & 0xFF;        // 低8位
  Send_Data.tx[3] = (transition >> 8) & 0xFF; // 高8位

  // Y轴目标线速度转换 (m/s -> mm/s)
  transition = (short)(cmd_vel.linear.y * 1000);
  Send_Data.tx[6] = transition & 0xFF;
  Send_Data.tx[5] = (transition >> 8) & 0xFF;

  // Z轴目标角速度转换 (rad/s -> mrad/s)
  transition = (short)(cmd_vel.angular.z * 1000);
  Send_Data.tx[8] = transition & 0xFF;
  Send_Data.tx[7] = (transition >> 8) & 0xFF;

  Send_Data.tx[9] = Check_Sum(9, SEND_DATA_CHECK); // BCC校验
  Send_Data.tx[10] = FRAME_TAIL; // 帧尾 0x7D

  try {
    if(Stm32_Serial.isOpen()) {
      Stm32_Serial.write(Send_Data.tx, sizeof(Send_Data.tx));
    }
  }
  catch (serial::IOException& e) {
    RCLCPP_ERROR(this->get_logger(), "无法通过串口发送速度指令");
  }
}

void turn_on_robot::Publish_FailsafeState()
{
  // 该话题用于上层监控和日志判断当前是否因指令超时被强制停车。
  std_msgs::msg::Bool msg;
  msg.data = failsafe_active_;
  failsafe_publisher->publish(msg);
}

void turn_on_robot::Update_Command_Watchdog()
{
  const auto now = steady_clock_.now();

  // 控制串口发送频率：正常和超时状态都通过同一个周期出口下发。
  const double send_period = 1.0 / std::max(1.0, cmd_vel_send_rate_);
  if ((now - last_cmd_send_time_).seconds() < send_period) {
    return;
  }
  last_cmd_send_time_ = now;

  geometry_msgs::msg::Twist output_cmd;

  // 启动后未收到指令，或最近指令超过安全阈值，均视为上层控制失效。
  const bool command_expired =
    !has_cmd_vel_ || ((now - last_cmd_time_).seconds() > cmd_vel_timeout_);

  if (command_expired) {
    if (!failsafe_active_) {
      RCLCPP_WARN(this->get_logger(), "cmd_vel watchdog timeout, forcing zero velocity");
    }
    failsafe_active_ = true;
  } else {
    if (failsafe_active_) {
      RCLCPP_INFO(this->get_logger(), "cmd_vel watchdog recovered");
    }
    failsafe_active_ = false;
    // 只有新鲜指令才允许进入底盘；否则 output_cmd 保持默认零向量。
    output_cmd = latest_cmd_vel_;
  }

  // failsafe_active_ 为 true 时 output_cmd 是零向量，并会持续周期性下发。
  Send_Velocity_Command(output_cmd);
  Publish_FailsafeState();
}

/**
 * @brief 发布IMU感测数据话题。
 */
void turn_on_robot::Publish_ImuSensor()
{
  sensor_msgs::msg::Imu Imu_Data_Pub;
  Imu_Data_Pub.header.stamp = rclcpp::Node::now(); 
  Imu_Data_Pub.header.frame_id = gyro_frame_id; 

  Imu_Data_Pub.orientation = Mpu6050.orientation; 
  Imu_Data_Pub.orientation_covariance[0] = 1e6;
  Imu_Data_Pub.orientation_covariance[4] = 1e6;
  Imu_Data_Pub.orientation_covariance[8] = 1e-6;

  Imu_Data_Pub.angular_velocity = Mpu6050.angular_velocity;
  Imu_Data_Pub.angular_velocity_covariance[0] = 1e6;
  Imu_Data_Pub.angular_velocity_covariance[4] = 1e6;
  Imu_Data_Pub.angular_velocity_covariance[8] = 1e-6;

  Imu_Data_Pub.linear_acceleration = Mpu6050.linear_acceleration;
  
  imu_publisher->publish(Imu_Data_Pub); 
}

/**
 * @brief 发布里程计话题，包含位置、姿态、速度信息。
 */
void turn_on_robot::Publish_Odom()
{
    tf2::Quaternion q;
    q.setRPY(0, 0, Robot_Pos.Z);
    geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(q);
    
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = rclcpp::Node::now(); 
    odom.header.frame_id = odom_frame_id; 
    odom.pose.pose.position.x = Robot_Pos.X; 
    odom.pose.pose.position.y = Robot_Pos.Y;
    odom.pose.pose.position.z = 0.0; // 2D平面
    odom.pose.pose.orientation = odom_quat; 

    odom.child_frame_id = robot_frame_id; 
    odom.twist.twist.linear.x = Robot_Vel.X; 
    odom.twist.twist.linear.y = Robot_Vel.Y; 
    odom.twist.twist.angular.z = Robot_Vel.Z; 

    // 根据运动状态选择不同的协方差矩阵
    if(Robot_Vel.X == 0 && Robot_Vel.Y == 0 && Robot_Vel.Z == 0) {
      memcpy(&odom.pose.covariance, odom_pose_covariance2, sizeof(odom_pose_covariance2));
      memcpy(&odom.twist.covariance, odom_twist_covariance2, sizeof(odom_twist_covariance2));
    } else {
      memcpy(&odom.pose.covariance, odom_pose_covariance, sizeof(odom_pose_covariance));
      memcpy(&odom.twist.covariance, odom_twist_covariance, sizeof(odom_twist_covariance));
    }
    odom_publisher->publish(odom); 
}

/**
 * @brief 发布电池电压信息。
 */
void turn_on_robot::Publish_Voltage()
{
    std_msgs::msg::Float32 voltage_msgs;
    static float Count_Voltage_Pub = 0;
    if(Count_Voltage_Pub++ > 10) {
        Count_Voltage_Pub = 0;  
        voltage_msgs.data = Power_voltage; 
        voltage_publisher->publish(voltage_msgs); 
    }
}

/**
 * @brief 串口通讯校验函数。
 * @param Count_Number 参与校验的数据长度
 * @param mode 0为接收校验，1为发送校验
 * @return unsigned char 校验结果
 */
unsigned char turn_on_robot::Check_Sum(unsigned char Count_Number, unsigned char mode)
{
  unsigned char check_sum = 0, k;
  if(mode == 0) {
    for(k = 0; k < Count_Number; k++) check_sum ^= Receive_Data.rx[k];
  } else {
    for(k = 0; k < Count_Number; k++) check_sum ^= Send_Data.tx[k];
  }
  return check_sum;
}

/**
 * @brief 逐帧读取并校验下位机数据。
 * @return bool 数据校验是否成功并解析
 */
bool turn_on_robot::Get_Sensor_Data_New()
{
  short transition_16 = 0;
  uint8_t check = 0, Receive_Data_Pr[1];
  static int count = 0;
  
  if(Stm32_Serial.available()) {
    Stm32_Serial.read(Receive_Data_Pr, 1);
    Receive_Data.rx[count] = Receive_Data_Pr[0];

    if(Receive_Data.rx[0] == FRAME_HEADER || count > 0) count++;
    else count = 0;

    if(count == 24) {
      count = 0;
      if(Receive_Data.rx[23] == FRAME_TAIL) {
        check = Check_Sum(22, 0);
        if(check == Receive_Data.rx[22]) {
          // 解析速度
          Robot_Vel.X = Odom_Trans(Receive_Data.rx[2], Receive_Data.rx[3]);
          Robot_Vel.Y = Odom_Trans(Receive_Data.rx[4], Receive_Data.rx[5]);
          Robot_Vel.Z = Odom_Trans(Receive_Data.rx[6], Receive_Data.rx[7]);
          
          // 解析IMU
          Mpu6050_Data.accele_x_data = IMU_Trans(Receive_Data.rx[8], Receive_Data.rx[9]);
          Mpu6050_Data.accele_y_data = IMU_Trans(Receive_Data.rx[10], Receive_Data.rx[11]);
          Mpu6050_Data.accele_z_data = IMU_Trans(Receive_Data.rx[12], Receive_Data.rx[13]);
          
          Mpu6050.linear_acceleration.x = Mpu6050_Data.accele_x_data / ACCEl_RATIO;
          Mpu6050.linear_acceleration.y = Mpu6050_Data.accele_y_data / ACCEl_RATIO;
          Mpu6050.linear_acceleration.z = Mpu6050_Data.accele_z_data / ACCEl_RATIO;

          // 解析电压
          transition_16 = (short)((Receive_Data.rx[20] << 8) | Receive_Data.rx[21]);
          Power_voltage = transition_16 / 1000.0f;
          
          return true;
        }
      }
    }
  }
  return false;
}

/**
 * @brief 控制循环：获取下位机数据、积分位置并发布话题。
 */
void turn_on_robot::Control()
{
  _Last_Time = rclcpp::Node::now();
  while(rclcpp::ok())
  {
    _Now = rclcpp::Node::now();
    Sampling_Time = (_Now - _Last_Time).seconds();
    _Last_Time = _Now;

    if (Get_Sensor_Data_New()) 
    {
      // 里程计修正
      Robot_Vel.X *= odom_x_scale;
      Robot_Vel.Y *= odom_y_scale;
      Robot_Vel.Z *= (Robot_Vel.Z >= 0) ? odom_z_scale_positive : odom_z_scale_negative;

      // 航位推算 (Dead Reckoning)
      Robot_Pos.X += (Robot_Vel.X * cos(Robot_Pos.Z) - Robot_Vel.Y * sin(Robot_Pos.Z)) * Sampling_Time;
      Robot_Pos.Y += (Robot_Vel.X * sin(Robot_Pos.Z) + Robot_Vel.Y * cos(Robot_Pos.Z)) * Sampling_Time;
      Robot_Pos.Z += Robot_Vel.Z * Sampling_Time;

      // 发布相关话题
      Publish_Odom();
      Publish_ImuSensor();
      Publish_Voltage();
    }
    rclcpp::spin_some(node_handle); // 处理回调
    Update_Command_Watchdog(); // 每轮循环都检查上层指令是否超时
  }
}


/**
 * @brief 构造函数，执行初始化工作。
 */
turn_on_robot::turn_on_robot()
: rclcpp::Node("chassis_bringup"),
  steady_clock_(RCL_STEADY_TIME),
  last_cmd_time_(steady_clock_.now()),
  last_cmd_send_time_(steady_clock_.now()),
  has_cmd_vel_(false),
  failsafe_active_(true),
  cmd_vel_timeout_(0.5),
  cmd_vel_send_rate_(50.0),
  max_linear_x_(0.0),
  max_linear_y_(0.0),
  max_angular_z_(0.0)
{
  Sampling_Time=0;
  Power_voltage=0;
  // 清空数据结构体
  memset(&Robot_Pos, 0, sizeof(Robot_Pos));
  memset(&Robot_Vel, 0, sizeof(Robot_Vel));
  memset(&Receive_Data, 0, sizeof(Receive_Data)); 
  memset(&Send_Data, 0, sizeof(Send_Data));
  memset(&Mpu6050_Data, 0, sizeof(Mpu6050_Data));

  // 声明并获取参数
  this->declare_parameter<int>("serial_baud_rate", 115200);
  this->declare_parameter<std::string>("usart_port_name", "/dev/ttyACM0");
  this->declare_parameter<std::string>("odom_frame_id", "odom");
  this->declare_parameter<std::string>("robot_frame_id", "base_footprint");
  this->declare_parameter<std::string>("gyro_frame_id", "gyro_link");
  this->declare_parameter<double>("odom_x_scale", 1.0);
  this->declare_parameter<double>("odom_y_scale", 1.0);
  this->declare_parameter<double>("odom_z_scale_positive", 1.0);
  this->declare_parameter<double>("odom_z_scale_negative", 1.0);
  this->declare_parameter<double>("cmd_vel_timeout", 0.5);
  this->declare_parameter<double>("cmd_vel_send_rate", 50.0);
  // 速度限幅默认关闭；如需启用，将对应参数设置为正数。
  this->declare_parameter<double>("max_linear_x", 0.0);
  this->declare_parameter<double>("max_linear_y", 0.0);
  this->declare_parameter<double>("max_angular_z", 0.0);

  this->get_parameter("serial_baud_rate", serial_baud_rate);
  this->get_parameter("usart_port_name", usart_port_name);
  this->get_parameter("odom_frame_id", odom_frame_id);
  this->get_parameter("robot_frame_id", robot_frame_id);
  this->get_parameter("gyro_frame_id", gyro_frame_id);
  this->get_parameter("odom_x_scale", odom_x_scale);
  this->get_parameter("odom_y_scale", odom_y_scale);
  this->get_parameter("odom_z_scale_positive", odom_z_scale_positive);
  this->get_parameter("odom_z_scale_negative", odom_z_scale_negative);
  this->get_parameter("cmd_vel_timeout", cmd_vel_timeout_);
  this->get_parameter("cmd_vel_send_rate", cmd_vel_send_rate_);
  this->get_parameter("max_linear_x", max_linear_x_);
  this->get_parameter("max_linear_y", max_linear_y_);
  this->get_parameter("max_angular_z", max_angular_z_);
  if (cmd_vel_timeout_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "cmd_vel_timeout must be positive, using 0.5s");
    cmd_vel_timeout_ = 0.5;
  }
  if (cmd_vel_send_rate_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "cmd_vel_send_rate must be positive, using 50Hz");
    cmd_vel_send_rate_ = 50.0;
  }

  // 创建发布者
  odom_publisher = create_publisher<nav_msgs::msg::Odometry>("odom", 2);
  imu_publisher = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 2);
  voltage_publisher = create_publisher<std_msgs::msg::Float32>("PowerVoltage", 1);
  failsafe_publisher = create_publisher<std_msgs::msg::Bool>("cmd_vel_failsafe", 10);

  // 创建订阅者
  // 保持原 /cmd_vel 入口不变；安全接管逻辑在本节点内部完成。
  Cmd_Vel_Sub = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 2, std::bind(&turn_on_robot::Cmd_Vel_Callback, this, _1));
    
  RCLCPP_INFO(this->get_logger(), "数据初始化就绪");

  try { 
    Stm32_Serial.setPort(usart_port_name);
    Stm32_Serial.setBaudrate(serial_baud_rate);
    serial::Timeout _time = serial::Timeout::simpleTimeout(2000);
    Stm32_Serial.setTimeout(_time);
    Stm32_Serial.open();
    Stm32_Serial.flushInput();
  }
  catch (serial::IOException& e) {
    RCLCPP_ERROR(this->get_logger(), "无法开启串口，请检查串口线连接！");
  }

  if(Stm32_Serial.isOpen()) {
    RCLCPP_INFO(this->get_logger(), "串口已成功开启");
  }
}

/**
 * @brief 析构函数，关闭前发送停止指令并关闭串口。
 */
turn_on_robot::~turn_on_robot()
{
  // 节点正常退出时主动补发一帧零速；进程崩溃仍需下位机侧 watchdog 兜底。
  geometry_msgs::msg::Twist stop_cmd;
  Send_Velocity_Command(stop_cmd);
  Stm32_Serial.close();
  RCLCPP_INFO(this->get_logger(), "正在关闭节点...");
}
