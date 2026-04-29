#include "chassis_bringup/chassis_bringup.h"
#include "chassis_bringup/Quaternion_Solution.h"
// my_interfaces/msg/data.hpp 已在头文件中包含

sensor_msgs::msg::Imu Mpu6050; // 实例化IMU对象 

using std::placeholders::_1;
using namespace std;
rclcpp::Node::SharedPtr node_handle = nullptr;

// 自动回充使用相关变量
bool check_AutoCharge_data = false;
bool charge_set_state = false;

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
  short transition; // 中间变量

  Send_Data.tx[0] = FRAME_HEADER; // 帧头 0x7B
  Send_Data.tx[1] = AutoRecharge; // 预留位/回充标志
  Send_Data.tx[2] = 0;            // 预留位

  // X轴目标线速度转换 (m/s -> mm/s)
  transition = (short)(twist_aux->linear.x * 1000);
  Send_Data.tx[4] = transition & 0xFF;       // 低8位
  Send_Data.tx[3] = (transition >> 8) & 0xFF; // 高8位

  // Y轴目标线速度转换 (m/s -> mm/s)
  transition = (short)(twist_aux->linear.y * 1000);
  Send_Data.tx[6] = transition & 0xFF;
  Send_Data.tx[5] = (transition >> 8) & 0xFF;

  // Z轴目标角速度转换 (rad/s -> mrad/s)
  transition = (short)(twist_aux->angular.z * 1000);
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
    RCLCPP_ERROR(this->get_logger(), "无法通过串口发送数据");
  }
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
 * @brief 发布红外信号发现状态。
 */
void turn_on_robot::Publish_RED()
{
    std_msgs::msg::UInt8 msg;
    msg.data = Red;
    RED_publisher->publish(msg); 
}

/**
 * @brief 发布充电状态信息。
 */
void turn_on_robot::Publish_Charging()
{
    static bool last_charging = false;
    std_msgs::msg::Bool msg;
    msg.data = Charging;
    Charging_publisher->publish(msg); 
    if(!last_charging && Charging) cout << "机器人正在充电" << endl;
    if(last_charging && !Charging) cout << "机器人充电连接断开" << endl;
    last_charging = Charging;
}

/**
 * @brief 发布充电电流信息。
 */
void turn_on_robot::Publish_ChargingCurrent()
{
    std_msgs::msg::Float32 msg;
    msg.data = Charging_Current;
    Charging_current_publisher->publish(msg);
}

/**
 * @brief 红外引导速度回调函数，用于自动回充时的红外引导控制。
 * @param twist_aux 订阅到的Twist消息指针
 */
void turn_on_robot::Red_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux)
{
    // 自动回充红外引导速度控制，与Cmd_Vel_Callback类似
    short transition;

    Send_Data.tx[0] = FRAME_HEADER;
    Send_Data.tx[1] = AutoRecharge;
    Send_Data.tx[2] = 0;

    transition = (short)(twist_aux->linear.x * 1000);
    Send_Data.tx[4] = transition & 0xFF;
    Send_Data.tx[3] = (transition >> 8) & 0xFF;

    transition = (short)(twist_aux->linear.y * 1000);
    Send_Data.tx[6] = transition & 0xFF;
    Send_Data.tx[5] = (transition >> 8) & 0xFF;

    transition = (short)(twist_aux->angular.z * 1000);
    Send_Data.tx[8] = transition & 0xFF;
    Send_Data.tx[7] = (transition >> 8) & 0xFF;

    Send_Data.tx[9] = Check_Sum(9, SEND_DATA_CHECK);
    Send_Data.tx[10] = FRAME_TAIL;

    try {
        if(Stm32_Serial.isOpen()) {
            Stm32_Serial.write(Send_Data.tx, sizeof(Send_Data.tx));
        }
    }
    catch (serial::IOException& e) {
        RCLCPP_ERROR(this->get_logger(), "Red_Vel: 无法通过串口发送数据");
    }
}

/**
 * @brief 回充标志位回调函数，接收回充状态指令。
 * @param Recharge_Flag 回充标志消息指针
 */
void turn_on_robot::Recharge_Flag_Callback(const std_msgs::msg::Int8::SharedPtr Recharge_Flag)
{
    AutoRecharge = Recharge_Flag->data;
}

/**
 * @brief 设置充电服务回调函数。
 * @param req 服务请求
 * @param res 服务响应
 */
void turn_on_robot::Set_Charge_Callback(
    const shared_ptr<turtlesim::srv::Spawn::Request> req,
    shared_ptr<turtlesim::srv::Spawn::Response> res)
{
    // 通过服务接口设置充电状态
    charge_set_state = true;
    RCLCPP_INFO(this->get_logger(), "收到设置充电请求: x=%.2f, y=%.2f", req->x, req->y);
    res->name = "charge_set";
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
  }
}


/**
 * @brief 构造函数，执行初始化工作。
 */
turn_on_robot::turn_on_robot():rclcpp::Node ("chassis_bringup")
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

  this->get_parameter("serial_baud_rate", serial_baud_rate);
  this->get_parameter("usart_port_name", usart_port_name);
  this->get_parameter("odom_frame_id", odom_frame_id);
  this->get_parameter("robot_frame_id", robot_frame_id);
  this->get_parameter("gyro_frame_id", gyro_frame_id);
  this->get_parameter("odom_x_scale", odom_x_scale);
  this->get_parameter("odom_y_scale", odom_y_scale);
  this->get_parameter("odom_z_scale_positive", odom_z_scale_positive);
  this->get_parameter("odom_z_scale_negative", odom_z_scale_negative);

  // 创建发布者
  odom_publisher = create_publisher<nav_msgs::msg::Odometry>("odom", 2);
  imu_publisher = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 2);
  voltage_publisher = create_publisher<std_msgs::msg::Float32>("PowerVoltage", 1);
  Charging_publisher = create_publisher<std_msgs::msg::Bool>("robot_charging_flag", 10);
  Charging_current_publisher = create_publisher<std_msgs::msg::Float32>("robot_charging_current", 10);
  RED_publisher = create_publisher<std_msgs::msg::UInt8>("robot_red_flag", 10);

  // 创建订阅者
  Red_Vel_Sub = create_subscription<geometry_msgs::msg::Twist>(
      "red_vel", 10, std::bind(&turn_on_robot::Red_Vel_Callback, this, std::placeholders::_1));
  Recharge_Flag_Sub = create_subscription<std_msgs::msg::Int8>(
      "robot_recharge_flag", 10, std::bind(&turn_on_robot::Recharge_Flag_Callback, this,std::placeholders::_1));
  Cmd_Vel_Sub = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 2, std::bind(&turn_on_robot::Cmd_Vel_Callback, this, _1));

  // 创建服务
  SetCharge_Service = this->create_service<turtlesim::srv::Spawn>(
      "/set_charge", std::bind(&turn_on_robot::Set_Charge_Callback, this, std::placeholders::_1, std::placeholders::_2));
    
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
  Send_Data.tx[0] = FRAME_HEADER;
  Send_Data.tx[1] = 0;  
  Send_Data.tx[2] = 0; 

  // 停止所有轴运动
  Send_Data.tx[4] = 0; Send_Data.tx[3] = 0;  
  Send_Data.tx[6] = 0; Send_Data.tx[5] = 0;  
  Send_Data.tx[8] = 0; Send_Data.tx[7] = 0;    
  
  Send_Data.tx[9] = Check_Sum(9, SEND_DATA_CHECK);
  Send_Data.tx[10] = FRAME_TAIL; 

  try {
    Stm32_Serial.write(Send_Data.tx, sizeof(Send_Data.tx));
  }
  catch (serial::IOException& e) {
    RCLCPP_ERROR(this->get_logger(), "析构时无法发送停止指令");
  }
  Stm32_Serial.close();
  RCLCPP_INFO(this->get_logger(), "正在关闭节点...");
}
